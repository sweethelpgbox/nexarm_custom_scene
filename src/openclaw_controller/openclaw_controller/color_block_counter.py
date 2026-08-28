#!/usr/bin/env python3
# encoding: utf-8

import json
import os
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

DEFAULT_IMAGE_TOPIC = '/depth_cam/rgb/image_raw'
DEFAULT_LAB_CONFIG = '/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml'
DEFAULT_TARGET_COLORS = ['red', 'green', 'blue', 'yellow', 'orange']
COLOR_BGR = {
    'red': (0, 0, 255),
    'green': (0, 200, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
    'orange': (0, 165, 255),
}


class SurfaceExtractor:
    def adaptive_threshold(self, gray_image):
        return cv2.adaptiveThreshold(
            gray_image,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            15,
            3,
        )

    def canny_proc(self, gray_image):
        mean_val = float(np.mean(gray_image))
        if mean_val < 60.0:
            low_thr = 15
        elif mean_val < 100.0:
            low_thr = 20
        else:
            low_thr = 25
        high_thr = low_thr * 3
        mask = cv2.Canny(gray_image, low_thr, high_thr, apertureSize=3, L2gradient=True)
        return 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    def gamma_correction(self, image, gamma=1.5):
        table = np.array(
            [((i / 255.0) ** (1.0 / gamma)) * 255 for i in np.arange(0, 256)]
        ).astype('uint8')
        return cv2.LUT(image, table)

    def get_top_surface(self, bgr_image):
        image_gray_check = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        mean_val = float(np.mean(image_gray_check))
        if mean_val < 60.0:
            gamma_val = 1.8
        elif mean_val < 100.0:
            gamma_val = 1.5
        else:
            gamma_val = 1.2

        bright_img = self.gamma_correction(bgr_image, gamma=gamma_val)
        image_scale = cv2.convertScaleAbs(bright_img, alpha=1.3, beta=10)
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_BGR2GRAY)
        image_gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(image_gray)
        image_mb = cv2.medianBlur(image_gray, 3)
        image_gs = cv2.GaussianBlur(image_mb, (3, 3), 3)
        binary = self.adaptive_threshold(image_gs)
        mask = self.canny_proc(image_gs)
        return cv2.bitwise_and(bgr_image, bgr_image, mask=cv2.bitwise_and(binary, mask))


class ColorBlockCounterNode(Node):
    def __init__(self, name='color_block_counter'):
        super().__init__(name)

        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('lab_config_file', DEFAULT_LAB_CONFIG)
        self.declare_parameter('target_colors', DEFAULT_TARGET_COLORS)
        self.declare_parameter('min_area', 200)
        self.declare_parameter('max_area', 7000)
        self.declare_parameter('roi', [])
        self.declare_parameter('detect_period', 0.2)
        self.declare_parameter('enabled_on_start', True)
        self.declare_parameter('publish_result_image', True)

        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.surface_extractor = SurfaceExtractor()
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.lab_config_file = str(self.get_parameter('lab_config_file').value)
        self.target_colors = self._sanitize_target_colors(self.get_parameter('target_colors').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.max_area = int(self.get_parameter('max_area').value)
        self.roi = [int(v) for v in self.get_parameter('roi').value]
        self.detect_period = max(0.05, float(self.get_parameter('detect_period').value))
        self.counting_enabled = bool(self.get_parameter('enabled_on_start').value)
        self.publish_result_image = bool(self.get_parameter('publish_result_image').value)

        self.latest_image = None
        self.latest_frame_index = 0
        self.last_processed_frame_index = -1
        self.last_counts = OrderedDict((color, 0) for color in self.target_colors)
        self.last_result_json = ''
        self.last_result_time = ''

        self.lab_config_mtime = None
        self.color_ranges = {}
        self._load_lab_config(force=True)

        self.result_image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.result_pub = self.create_publisher(String, '~/count_result', 10)

        self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.create_service(Trigger, '~/count_colors', self.count_colors_callback)
        self.create_service(Trigger, '~/get_latest_result', self.get_latest_result_callback)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.timer = self.create_timer(self.detect_period, self.timer_callback)
        self.get_logger().info(
            'Color block counter started: image_topic=%s, target_colors=%s'
            % (self.image_topic, self.target_colors)
        )

    def _sanitize_target_colors(self, colors):
        result = []
        for color in colors:
            color_name = str(color).strip().lower()
            if color_name and color_name not in result:
                result.append(color_name)
        if not result:
            result = list(DEFAULT_TARGET_COLORS)
        return result

    def _load_lab_config(self, force=False):
        try:
            mtime = os.path.getmtime(self.lab_config_file)
        except OSError as exc:
            self.get_logger().error('LAB 阈值文件读取失败: %s' % exc)
            return False

        if not force and self.lab_config_mtime is not None and mtime <= self.lab_config_mtime:
            return True

        try:
            with open(self.lab_config_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error('LAB 阈值文件解析失败: %s' % exc)
            return False

        params = data.get('/**', {}).get('ros__parameters', {})
        color_range_list = params.get('color_range_list', {})
        if not isinstance(color_range_list, dict):
            self.get_logger().error('LAB 阈值文件缺少 color_range_list')
            return False

        self.color_ranges = color_range_list
        self.lab_config_mtime = mtime
        return True

    def get_node_state(self, request, response):
        response.success = True
        response.message = 'ready'
        return response

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('图像转换失败: %s' % exc)
            return

        with self.lock:
            self.latest_image = image
            self.latest_frame_index += 1

    def set_running_callback(self, request, response):
        self.counting_enabled = bool(request.data)
        response.success = True
        response.message = 'running' if self.counting_enabled else 'stopped'
        return response

    def count_colors_callback(self, request, response):
        image, frame_index = self._snapshot_latest_image()
        if image is None:
            response.success = False
            response.message = 'no image received yet'
            return response

        success, result_json = self._count_and_publish(image, frame_index)
        response.success = success
        response.message = result_json
        return response

    def get_latest_result_callback(self, request, response):
        if not self.last_result_json:
            response.success = False
            response.message = 'no counting result available yet'
            return response
        response.success = True
        response.message = self.last_result_json
        return response

    def timer_callback(self):
        if not self.counting_enabled:
            return

        image, frame_index = self._snapshot_latest_image()
        if image is None or frame_index == self.last_processed_frame_index:
            return

        self._count_and_publish(image, frame_index)

    def _snapshot_latest_image(self):
        with self.lock:
            if self.latest_image is None:
                return None, self.latest_frame_index
            return self.latest_image.copy(), self.latest_frame_index

    def _count_and_publish(self, image, frame_index):
        if not self._load_lab_config():
            return False, 'failed to load lab config'

        try:
            counts, draw_image = self._detect_color_blocks(image)
        except Exception as exc:
            self.get_logger().error('颜色统计失败: %s' % exc)
            return False, 'count failed: %s' % exc

        result_json = self._build_result_json(counts)
        self.last_counts = counts
        self.last_result_json = result_json
        self.last_result_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        self.last_processed_frame_index = frame_index

        result_msg = String()
        result_msg.data = result_json
        self.result_pub.publish(result_msg)

        if self.publish_result_image:
            self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(draw_image, encoding='bgr8'))

        return True, result_json

    def _resolve_roi(self, image_shape):
        image_h, image_w = image_shape[:2]
        if len(self.roi) != 4:
            return (0, image_h, 0, image_w)

        y_min, y_max, x_min, x_max = self.roi
        y_min = max(0, min(y_min, image_h))
        y_max = max(0, min(y_max, image_h))
        x_min = max(0, min(x_min, image_w))
        x_max = max(0, min(x_max, image_w))
        if y_max <= y_min or x_max <= x_min:
            return (0, image_h, 0, image_w)
        return (y_min, y_max, x_min, x_max)

    def _detect_color_blocks(self, bgr_image):
        counts = OrderedDict((color, 0) for color in self.target_colors)
        draw_image = bgr_image.copy()
        y_min, y_max, x_min, x_max = self._resolve_roi(bgr_image.shape)

        roi_image = bgr_image[y_min:y_max, x_min:x_max]
        surface_image = self.surface_extractor.get_top_surface(roi_image)
        image_lab = cv2.cvtColor(cv2.GaussianBlur(surface_image, (3, 3), 3), cv2.COLOR_BGR2LAB)

        if (y_min, y_max, x_min, x_max) != (0, bgr_image.shape[0], 0, bgr_image.shape[1]):
            cv2.rectangle(draw_image, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)

        for color in self.target_colors:
            color_range = self.color_ranges.get(color)
            if color_range is None:
                continue

            mask = cv2.inRange(
                image_lab,
                tuple(color_range['min']),
                tuple(color_range['max']),
            )
            eroded = cv2.erode(mask, self.morph_kernel)
            dilated = cv2.dilate(eroded, self.morph_kernel)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour_index = 0
            for contour in contours:
                area = abs(cv2.contourArea(contour))
                if area < self.min_area or area > self.max_area:
                    continue

                counts[color] += 1
                contour_index += 1
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box[:, 0] += x_min
                box[:, 1] += y_min
                center_x, center_y = rect[0]
                center = (int(center_x + x_min), int(center_y + y_min))
                draw_color = COLOR_BGR.get(color, (255, 255, 255))

                cv2.drawContours(draw_image, [np.intp(box)], -1, draw_color, 2, cv2.LINE_AA)
                cv2.circle(draw_image, center, 4, draw_color, -1)
                cv2.putText(
                    draw_image,
                    '%s:%d' % (color, contour_index),
                    (center[0] + 6, max(20, center[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    draw_color,
                    1,
                    cv2.LINE_AA,
                )

        self._draw_summary(draw_image, counts)
        return counts, draw_image

    def _draw_summary(self, draw_image, counts):
        lines = ['%s: %d' % (color, counts[color]) for color in self.target_colors]
        lines.append('total: %d' % sum(counts.values()))

        panel_width = 220
        panel_height = 18 + len(lines) * 24
        cv2.rectangle(draw_image, (10, 10), (10 + panel_width, 10 + panel_height), (30, 30, 30), -1)
        cv2.rectangle(draw_image, (10, 10), (10 + panel_width, 10 + panel_height), (220, 220, 220), 1)

        for idx, line in enumerate(lines):
            y = 32 + idx * 24
            cv2.putText(
                draw_image,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _build_result_json(self, counts):
        payload = {
            'processed_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
            'counts': [{'color': color, 'count': int(counts[color])} for color in self.target_colors],
            'total': int(sum(counts.values())),
        }
        return json.dumps(payload, ensure_ascii=False)


def main():
    rclpy.init()
    node = ColorBlockCounterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
