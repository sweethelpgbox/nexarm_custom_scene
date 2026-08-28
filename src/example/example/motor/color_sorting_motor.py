#!/usr/bin/env python3
# coding: utf8

import os
import cv2
import yaml
import time
import math
import queue
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from sdk import common, fps
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from rclpy.callback_groups import ReentrantCallbackGroup

PLACE_POSITION = {
    # fallback color-based positions when slot mapping is unavailable
    'red': [-60, -230],
    'green': [60, -230],
    'blue': [60, -130],
    'yellow': [-60, -130],
}


SCENE5_SLOT_POSITIONS = [
    [-60, -230],   # slot 0: left-top
    [-60, -130],  # slot 1: right-top
    [60, -230],   # slot 2: left-bottom
    [60, -130],  # slot 3: right-bottom
]


class ColorSortingNode(Node):

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.function = self.get_string_param('function', 'pull')
        self.scene_config_path = self.get_string_param(
            'config_path',
            '/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml',
        )
        self.play_config_path = self.get_string_param(
            'play_config_path',
            '/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml',
        )

        self.running = True
        self.start_count = False
        self.lock = threading.RLock()
        self.image_queue = queue.Queue(maxsize=2)
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.center_imgpts = None
        self.offset = 0.005
        self.pick_pitch = 60
        self.count = 0
        self.data = common.get_yaml_data("/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml")
        self.lab_data = self.data['/**']['ros__parameters']
        self.camera_type = os.environ['CAMERA_TYPE']
        self.min_area = 500
        self.max_area = 15000
        self.target = None
        self.image_sub = None

        self.conveyor_pub = self.create_publisher(Int8, 'ros_robot_controller/conveyor/set', 1)
        self.arm_pub = self.create_publisher(ArmCoords, 'ros_robot_controller/arm/set_coords', 5)
        self.image_sub = self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        # publish result image on a topic matching the node name to avoid conflicts
        result_topic = f'/{self.get_name()}/image_result'
        self.result_image_pub = self.create_publisher(Image, result_topic, 1)
        self.scene5_enabled = False
        self.scene5_active_colors = set(PLACE_POSITION.keys())
        self.scene5_slot_order = list(PLACE_POSITION.keys())
        self.scene5_place_targets = self.load_scene5_place_targets('color')

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, 'controller_manager/init_finish')
        self.client.wait_for_service()

        if self.function == "pull":
            self.speed = -100
        else:
            self.speed = 100

        threading.Thread(target=self.main, daemon=True).start()

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def get_string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

    def load_scene5_place_targets(self, group):
        try:
            config = {}
            if self.play_config_path and os.path.exists(self.play_config_path):
                with open(self.play_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
            targets = (
                config.get('scene5_dual_arm', {})
                .get('arm_b_place_targets', {})
                .get(group, {})
            )
            if not isinstance(targets, dict):
                with open(self.scene_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
                targets = (
                    config.get('scenes', {})
                    .get('scene_5', {})
                    .get('scene5_dual_arm', {})
                    .get('arm_b_place_targets', {})
                    .get(group, {})
                )
            if not isinstance(targets, dict):
                return {}
            clean = {}
            for key, value in targets.items():
                if isinstance(value, (list, tuple)) and len(value) >= 2:
                    clean[str(key)] = [float(value[0]), float(value[1])]
            return clean
        except Exception as ex:
            self.get_logger().warn(f'load scene5 {group} place targets failed: {ex}')
            return {}

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def adaptive_threshold(self, gray_image):
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)
        return binary

    def canny_proc(self, bgr_image):
        mask = cv2.Canny(bgr_image, 9, 41, 9, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))
        return mask

    def get_top_surface(self, rgb_image):
        image_scale = cv2.convertScaleAbs(rgb_image, alpha=2.5, beta=0)
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)
        image_gs = cv2.GaussianBlur(image_mb, (5, 5), 5)
        binary = self.adaptive_threshold(image_gs)
        mask = self.canny_proc(image_gs)
        mask1 = cv2.bitwise_and(binary, mask)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)
        return roi_image_mask

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)

    def image_processing(self):
        rgb_image = self.image_queue.get()
        result_image = np.copy(rgb_image)
        target_list = []
        index = 0
        if self.start_count:
            img_h, img_w = rgb_image.shape[:2]
            rgb_image = self.get_top_surface(rgb_image)
            image_lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
            for i in ['red', 'green', 'blue', 'yellow']:
                if i not in self.scene5_active_colors:
                    continue
                mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][i]['min']), tuple(self.lab_data['color_range_list'][i]['max']))
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)
                contours = map(lambda a_c: a_c[1], filter(lambda a: self.min_area <= a[0] <= self.max_area, contours_area))
                for c in contours:
                    area = math.fabs(cv2.contourArea(c))
                    rect = cv2.minAreaRect(c)
                    (center_x, center_y), _ = cv2.minEnclosingCircle(c)
                    cv2.circle(result_image, (int(center_x), int(center_y)), 8, (0, 0, 0), -1)
                    corners = list(map(lambda p: (p[0], p[1]), cv2.boxPoints(rect)))
                    cv2.drawContours(result_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)
                    index += 1
                    angle = int(round(rect[2]))
                    target_list.append([i, index, (center_x, center_y), angle])
        if result_image is not None:
            try:
                self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, encoding='rgb8'))
            except Exception as ex:
                self.get_logger().warn(f'publish result image failed: {ex}')
        return target_list, result_image

    def get_scene5_place_position(self, color_key):
        # Robust mapping: resolve recognized color to fixed slot positions using scene5_slot_order
        if color_key is None:
            self.get_logger().warn('scene5 color_key is None, fallback to red')
            return [float(v) for v in PLACE_POSITION['red']]

        fixed_pos = self.scene5_place_targets.get(color_key)
        if fixed_pos is not None:
            self.get_logger().info(f'scene5 fixed mapping: color={color_key}, place_pos={fixed_pos}')
            return [float(fixed_pos[0]), float(fixed_pos[1])]

        if isinstance(self.scene5_slot_order, (list, tuple)) and color_key in self.scene5_slot_order:
            try:
                index = self.scene5_slot_order.index(color_key)
                if 0 <= index < len(SCENE5_SLOT_POSITIONS):
                    place_pos = [float(SCENE5_SLOT_POSITIONS[index][0]), float(SCENE5_SLOT_POSITIONS[index][1])]
                    self.get_logger().info(f'scene5 mapping: color={color_key}, slot_index={index}, place_pos={place_pos}')
                    return place_pos
                self.get_logger().warn(f'scene5 color={color_key} index={index} outside slot positions, fallback to color-based position')
            except Exception as ex:
                self.get_logger().warn(f'error resolving slot index for {color_key}: {ex}')

        raw_pos = PLACE_POSITION.get(color_key) or PLACE_POSITION['red']
        try:
            place_pos = [float(raw_pos[0]), float(raw_pos[1])]
        except Exception:
            self.get_logger().error(f'invalid place_pos format for {color_key}: {raw_pos}; using red fallback')
            place_pos = [float(PLACE_POSITION['red'][0]), float(PLACE_POSITION['red'][1])]

        self.get_logger().info(f'scene5 fallback mapping: color={color_key}, place_pos={place_pos}')
        return place_pos

    def on_scene5_enter(self, request, response):
        self.scene5_enabled = True
        self.start_count = False
        self.publish_arm(200, 0, 200, -90, 0, -60, 1500)
        response.success = True
        conveyor_msg = Int8()
        conveyor_msg.data = self.speed
        self.conveyor_pub.publish(conveyor_msg)
        response.message = 'scene5 entered'
        return response

    def on_scene5_exit(self, request, response):
        self.scene5_enabled = False
        self.start_count = False
        self.target = None
        conveyor_msg = Int8()
        conveyor_msg.data = 0
        self.conveyor_pub.publish(conveyor_msg)
        response.success = True
        response.message = 'scene5 exited'
        return response

    def on_scene5_enable_sorting(self, request, response):
        self.scene5_enabled = bool(request.data)
        self.start_count = bool(request.data)
        self.running = bool(request.data)
        response.success = True
        response.message = 'scene5 sorting enabled' if request.data else 'scene5 sorting disabled'
        return response

    def on_scene5_set_target(self, request, response):
        target_key = request.data_str.strip() if hasattr(request, 'data_str') else ''
        if not target_key:
            self.scene5_active_colors = set(PLACE_POSITION.keys())
            response.success = True
            response.message = 'scene5 active colors set to all'
            return response
        if target_key not in PLACE_POSITION:
            response.success = False
            response.message = f'invalid target key: {target_key}'
            return response
        self.scene5_active_colors = {target_key}
        self.get_logger().info(f'scene5 active colors updated: {self.scene5_active_colors}')
        response.success = True
        response.message = f'scene5 active color set to {target_key}'
        return response

    def on_scene5_set_slot_order(self, request, response):
        ordered = list(request.data)
        self.get_logger().info(f'scene5 set slot order request: {ordered}')
        if len(ordered) != len(PLACE_POSITION) or set(ordered) != set(PLACE_POSITION.keys()):
            response.success = False
            response.message = 'invalid slot order'
            return response
        self.scene5_slot_order = ordered
        response.success = True
        response.message = 'scene5 slot order updated'
        return response

    def move(self, center_y):
        if self.running:
            # center_y 在 280-360 区间时，X 线性映射到 200-150
            if 280 <= center_y <= 360:
                x_mm = 200.0 - (center_y - 280) * (50.0 / 80.0)
            else:
                x_mm = 200.0
            y_mm = 15.0
            z_mm = 80.0
            roll_angle = 0.0
            if self.running:
                # 移动到抓取点上方准备
                self.get_logger().info('0: 移动到抓取点上方')
                self.publish_arm(x_mm, y_mm, z_mm + 10, -90, roll_angle, -60, 1500)
                time.sleep(1.8)

            if self.running:
                # 下降并夹取
                self.get_logger().info('1: 下降并闭合夹爪')
                self.publish_arm(x_mm, y_mm, z_mm, -90, roll_angle, -60, 800)
                time.sleep(1.0)
                self.publish_arm(x_mm, y_mm, z_mm, -90, roll_angle, 30, 500)
                time.sleep(0.6)

            if self.running:
                # 抬起
                self.get_logger().info('2: 抬起')
                self.publish_arm(x_mm, y_mm, z_mm + 30, -90, roll_angle, 30, 800)
                time.sleep(1.0)

                self.get_logger().info('3: 移动到中转位置')
                self.publish_arm(200, 0, 200, -90, 0, 30, 1000)
                time.sleep(1.2)

            if self.running:
                # Move to place position
                place_pos = self.get_scene5_place_position(self.target[0])
                self.get_logger().info('5')
                self.publish_arm(place_pos[0], place_pos[1], 200, -90, 0, 30, 1500)
                self.get_logger().info(f"place_pos:{place_pos}")
                time.sleep(1.8)

            if self.running:
                # Lower to place
                self.get_logger().info('6')
                self.publish_arm(place_pos[0], place_pos[1],60.0, -90, 0, 30, 800)
                time.sleep(1.0)
                # Open gripper
                self.publish_arm(place_pos[0], place_pos[1],60.0, -90, 0, -60, 500)
                time.sleep(0.6)

            if self.running:
                # Lift and go home
                self.publish_arm(place_pos[0], place_pos[1],200, -90, 0, -60, 800)
                time.sleep(1.0)
                self.publish_arm(200, 0, 200, -90, 0, -60, 1500)
                time.sleep(1.8)
                self.target = None
                self.start_count = True

                conveyor_msg = Int8()
                conveyor_msg.data = self.speed
                self.conveyor_pub.publish(conveyor_msg)

            if not self.running:
                self.publish_arm(200, 0, 200, -90, 0, -60, 1500)
                time.sleep(1.8)
                self.target = None


    def main(self):
        while rclpy.ok():
            try:
                target_list, result_image = self.image_processing()
                if target_list and self.running:
                    target = target_list[0]
                    self.get_logger().info(f"target:{target[0]}")
                    center_x, center_y = target[2]
                    if self.target is not None:
                        if self.target[0] == target_list[0][0]:
                            self.count += 1
                        else:
                            self.target = target_list[0]
                            self.count = 0
                    else:
                        self.target = target_list[0]
                if self.count > 5 and abs(center_x - 300) <= 20:
                    self.count = 0
                    self.start_count = False
                    # 放置功能
                    if self.function == "pull":
                        conveyor_msg = Int8()
                        conveyor_msg.data = 0
                        self.conveyor_pub.publish(conveyor_msg)
                        threading.Thread(target=self.move, args=(target_list[0][2][1],), daemon=True).start()
                    # 抓取功能
                    else:
                        time.sleep(0.02)
                        threading.Thread(target=self.move, args=(target_list[0][2][1],), daemon=True).start()
                if result_image is not None:
                    self.fps.update()
                    result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
                    cv2.imshow('result_image', result_image)
                    # consume GUI events; no keyboard control
                    cv2.waitKey(1)
            except queue.Empty:
                continue

def main():
    try:
        node = ColorSortingNode('color_sorting')
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
             node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
