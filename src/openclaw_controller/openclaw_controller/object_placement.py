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
from std_srvs.srv import Trigger, SetBool
from interfaces.srv import SetStringBool, SetStringList
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from rclpy.callback_groups import ReentrantCallbackGroup

PLACE_POSITION = {
    '1': [130, -210.0, 200.0],
    '2': [30, -210.0, 200.0],
    '3': [-130, -210.0, 200.0],
    '4': [-230, -210.0, 200.0],
    'left': [30, 210.0, 200.0],
}


class ColorSortingNode(Node):

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)


        self.running = True
        self.start_count = True
        self.lock = threading.RLock()
        self.image_queue = queue.Queue(maxsize=2)
        self.place_position_queue = queue.Queue(maxsize=1)
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
        self.color = ''
        self.placing = False
        self.no_plate_reported = False
        self.place_status = 'idle'
        self.place_status_success = False

        self.place_status_srv = self.create_service(Trigger, '~/place_status', self.get_place_status_callback)


        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.color_sub = self.create_subscription(String, '/object_sorting/place_request', self.get_color_callback, 1)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        # publish result image on a topic matching the node name to avoid conflicts
        result_topic = f'/{self.get_name()}/image_result'
        self.result_image_pub = self.create_publisher(Image, result_topic, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        # self.publish_arm(0, -200, 200, -90, 0, -60, 1500)
        threading.Thread(target=self.place_worker, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()


    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def set_place_status(self, success, text):
        self.place_status_success = bool(success)
        self.place_status = text

    def get_place_status_callback(self, request, response):
        response.success = self.place_status_success
        response.message = self.place_status
        return response

    def submit_place_position(self, position):
        with self.lock:
            if self.placing or not self.place_position_queue.empty():
                self.get_logger().warn('placement already in progress, ignore new request')
                return False
            self.placing = True
            self.place_position_queue.put(position)
            return True

    def place_worker(self):
        while rclpy.ok():
            try:
                position = self.place_position_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.move(position)
            self.place_position_queue.task_done()

    def get_color_callback(self, msg):
        color = msg.data.strip()
        if color == 'left':
            self.color = color
            self.target = None
            self.count = 0
            self.start_count = False
            self.no_plate_reported = False
            self.running = True
            self.set_place_status(True, f"found:{self.color}")
            self.get_logger().info(f'direct placement target: {self.color}')
            place_pos = PLACE_POSITION['left']
            self.submit_place_position(place_pos)
            return
        if self.placing:
            self.get_logger().warn(f'placement already in progress, ignore place color: {color}')
            return
        if color not in self.lab_data['color_range_list']:
            self.get_logger().warn(f'unsupported place color: {color}')
            self.set_place_status(False, f"unsupported:{color}")
            return
        self.color = color
        self.target = None
        self.count = 0
        self.start_count = True
        self.placing = False
        self.no_plate_reported = False
        self.running = True
        self.set_place_status(False, f"not_found:{self.color}")
        self.get_logger().info(f'placement target color: {self.color}')


    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

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

    def draw_reference_lines(self, image):
        img_h, img_w = image.shape[:2]
        if img_h > 130:
            cv2.line(image, (0, 130), (img_w - 1, 130), (255, 255, 0), 2)
        if img_h > 320:
            cv2.line(image, (0, 320), (img_w - 1, 320), (255, 255, 0), 2)

    def image_processing(self):
        rgb_image = self.image_queue.get()
        result_image = np.copy(rgb_image)
        self.draw_reference_lines(result_image)
        target_list = []
        index = 0
        if not self.color or self.color not in self.lab_data['color_range_list']:
            return target_list, result_image

        if self.start_count:
            rgb_image = rgb_image[0:300, :]
            img_h, img_w = rgb_image.shape[:2]
            rgb_image = self.get_top_surface(rgb_image)
            image_lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
            # for i in ['red', 'green', 'blue']:
            mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][self.color]['min']), tuple(self.lab_data['color_range_list'][self.color]['max']))
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
                target_list.append([self.color, index, (center_x, center_y), angle])
        return target_list, result_image


    def move(self, position):
        x_mm, y_mm,z_mm = position
        placed_color = self.color
        claw_open = -60.0
        claw_close = 30.0

        # 放置左边，先回初始姿态，再到左边
        if x_mm == 30 and y_mm == 210.0 and z_mm == 200.0:
            self.publish_arm(200, 0, 200, -90, 0, claw_close, 1000)
            time.sleep(1.2)

        if self.running:
            self.publish_arm(x_mm, y_mm, z_mm, -90, 0.0, claw_close, 1500)
            time.sleep(1.8)
            self.publish_arm(x_mm, y_mm, z_mm - 140.0, -90, 0.0, claw_close, 1000)
            time.sleep(1.2)
            self.publish_arm(x_mm, y_mm, z_mm - 140.0, -90, 0.0, claw_open, 1000)
            time.sleep(1.2)
            self.publish_arm(x_mm, y_mm, z_mm, -90, 0.0, claw_open, 1500)
            time.sleep(1.8)

            self.target = None
            self.start_count = True
            self.color = ''
            self.placing = False
            self.no_plate_reported = False
            if x_mm == 30 and y_mm == 210.0 and z_mm == 200.0:
                self.publish_arm(200, 0, 200, -90, 0, -60, 1000)
            else:
                self.publish_arm(0, -200, 200, -90, 0, -60, 1000)
            time.sleep(1.2)
            self.set_place_status(True, f"placed:{placed_color}")
            self.running = False

    def main(self):
        while rclpy.ok():
            try:
                if self.running: 
                    target_list, result_image = self.image_processing()            
                    if target_list and not self.placing:
                        target = target_list[0]
                        self.get_logger().info(f"target:{target[0]}")
                        self.set_place_status(True, f"found:{self.color}")
                        self.no_plate_reported = False
                        center_x, center_y = target[2]
                        if self.target is not None:
                            if self.target[0] == target_list[0][0]:
                                self.count += 1
                            else:
                                self.target = target_list[0]
                                self.count = 0
                        else:
                            self.target = target_list[0]
                    else:
                        if self.color and self.start_count and not self.placing and not target_list and not self.no_plate_reported:
                            self.set_place_status(False, f"not_found:{self.color}")
                            self.no_plate_reported = True

                    if self.count > 5 :
                        self.count = 0
                        self.start_count = False
                        self.no_plate_reported = False
                        self.get_logger().info(f"Final target: {target_list[0][2][1]}")
                        center_x, center_y = target_list[0][2]
                        if center_x > 0 and center_x <= 160:
                            place_pos = PLACE_POSITION['1']
                        elif center_x > 160 and center_x <= 320:
                            place_pos = PLACE_POSITION['2']
                        elif center_x > 320 and center_x <= 480:
                            place_pos = PLACE_POSITION['3']
                        elif center_x > 480 and center_x <= 640:
                            place_pos = PLACE_POSITION['4']
                        self.submit_place_position(place_pos)
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
        node = ColorSortingNode('object_placement')
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
