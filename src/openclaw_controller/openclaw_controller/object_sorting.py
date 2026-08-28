#!/usr/bin/env python3
# coding: utf8
# 目标分拣
import os
import cv2
import yaml
import time
import math
import copy
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sdk import common, fps
from interfaces.srv import SetStringBool, SetString
from ros_robot_controller_msgs.msg import ArmCoords
from app.utils import calculate_grasp_yaw, position_change_detect, pick_and_place, image_process, distortion_inverse_map, utils


class ObjectSortingNode(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.image_process = image_process.GetObjectSurface()
        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))
        self.lab_data = self.data['/**']['ros__parameters']
        self.camera_type = os.environ.get('CAMERA_TYPE', '').lower()

        self.min_area = 200
        self.max_area = 7000
        self.target_labels = {
            "red": False,
            "green": False,
            "blue": False,
            "yellow": False,
        }
        self.running = True
        self.replaceable = None
        self.status_clock = True

        self._init_parameters()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.missing_target_pub = self.create_publisher(String, '~/missing_target_color', 10)
        self.place_request_pub = self.create_publisher(String, '~/place_request', 10)

        self.timer_cb_group = ReentrantCallbackGroup()
        # self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.enable_sorting_srv = self.create_service(SetBool, '~/enable_sorting', self.enable_sorting_srv_callback)
        self.set_target_srv = self.create_service(SetStringBool, '~/set_grab_color', self.set_target_srv_callback)
        self.set_placed_color_srv = self.create_service(SetString, '~/set_placed_color', self.set_placed_color_srv_callback)
        self.place_status_client = self.create_client(Trigger, '/object_placement/place_status')

        self.kinematics_client = self.create_client(Trigger, 'kinematics/init_finish')
        self.kinematics_client.wait_for_service()
        self.controller_manager_client = self.create_client(Trigger, 'controller_manager/init_finish', callback_group=self.timer_cb_group)
        self.controller_manager_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def query_place_status(self):
        if not self.place_status_client.wait_for_service(timeout_sec=0.2):
            return False, 'service_unavailable'

        future = self.place_status_client.call_async(Trigger.Request())
        while self.running and rclpy.ok() and not future.done():
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            return False, 'service_unavailable'

        response = future.result()
        return response.success, response.message.strip()

    def wait_for_place_status(self, requested_color):
        last_status = ''
        request_seen = False
        while self.running and rclpy.ok() and not self.enter:
            success, status = self.query_place_status()

            if status != last_status:
                last_status = status
                if status.startswith("not_found:"):
                    color = status.split(":", 1)[1]
                    self.get_logger().warn(f'place target not found, waiting: {color}')
                elif status.startswith("found:"):
                    color = status.split(":", 1)[1]
                    self.get_logger().info(f'place target found: {color}')
                elif status.startswith("placed:"):
                    color = status.split(":", 1)[1]
                    self.get_logger().info(f'place target placed: {color}')
                elif status and status != 'idle':
                    self.get_logger().warn(f'place status: {status}')

            if status.startswith("not_found:") or status.startswith("found:"):
                request_seen = True

            if success and request_seen and (status == "finish" or status.startswith("placed:")):
                self.go_home(False)
                self.enter = True
                return
            elif status.startswith("not_found:"):
                # Keep the arm waiting in place until the placement service reports completion.
                self.enter = False
            elif status.startswith("found:"):
                self.enter = False

            time.sleep(0.3)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def get_missing_target_color_callback(self, request, response):
        response.success = bool(self.latest_missing_target_message)
        response.message = self.latest_missing_target_message
        return response

    def _init_parameters(self):
        self.endpoint = None
        self.target_miss_count = 0
        self.latest_missing_target_message = ''
        self.transport_info = None
        self.intrinsic = None
        self.distortion = None
        self.start_transport = False
        self.enable_sorting = False
        self.white_area_center = None
        self.enter = False
        self.roi = []
        self.count_move = 0
        self.count_still = 0
        self.target = None
        self.start_get_roi = False
        self.last_position = None
        self.last_object_info_list = None
        self.image_sub = None
        self.camera_info_sub = None
        self.placed_color = ''
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))
        self.lab_data = self.data['/**']['ros__parameters']
        self.get_logger().info(f"lab_data: {self.lab_data}")

    def init_process(self):
        self.timer.cancel()

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()

        self.prepare_sorting()

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.create_service(Trigger, '~/get_missing_target_color', self.get_missing_target_color_callback)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def prepare_sorting(self):
        self.get_logger().info('\033[1;32m%s\033[0m' % "prepare object sorting")
        for color in self.target_labels:
            self.target_labels[color] = False
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.start_get_roi = True
        self.enable_sorting = False
        self.enter = True
        pick_and_place.interrupt(True)
        self.publish_arm(200, 0, 200, -90, 0, -65, 1000)

    def go_home(self, interrupt=True):
        if interrupt:
            self.publish_arm(200, 0, 200, -90, 0, -65, 500)
            time.sleep(0.5)

        self.publish_arm(200, 0, 200, -90, 0, -65, 1000)
        time.sleep(1)

        self.publish_arm(200, 0, 200, -90, 0, -65, 1000)
        time.sleep(1)

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])

        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat

        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        x_min = min(imgpts, key=lambda p: p[0])[0]
        x_max = max(imgpts, key=lambda p: p[0])[0]
        y_min = min(imgpts, key=lambda p: p[1])[1]
        y_max = max(imgpts, key=lambda p: p[1])[1]
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

        self.roi = roi

    def enable_sorting_srv_callback(self, request, response):
        if request.data:
            if any(self.target_labels.values()):
                self.get_logger().info('\033[1;32m%s\033[0m' % 'start  object sorting')
                # pick_and_place.interrupt(False)
                self.enable_sorting = True
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'wait target color before sorting')
                # pick_and_place.interrupt(True)
                self.enable_sorting = False
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop  object sorting')
            # pick_and_place.interrupt(True)
            self.enable_sorting = False

        response.success = True
        response.message = "start"
        return response

    def set_target_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32mset target %s %s\033[0m' % (str(request.data_str), str(request.data_bool)))
        if request.data_str in self.target_labels:
            self.target_labels[request.data_str] = request.data_bool
            self.latest_missing_target_message = ''
            if any(self.target_labels.values()):
                pick_and_place.interrupt(False)
                self.enable_sorting = True
            else:
                pick_and_place.interrupt(True)
                self.enable_sorting = False

        response.success = True
        response.message = "start"
        return response

    def set_placed_color_srv_callback(self, request, response):
        self.placed_color = request.data.strip()
        response.success = True
        response.message = self.placed_color
        # if self.replaceable:           
        #     self.publish_place_request(self.placed_color)
        #     self.replaceable = False
        self.get_logger().info(f'placed color: {self.placed_color}')
        return response

    def get_detected_colors(self, target_info):
        return sorted({target[0] for target in target_info})

    def get_missing_target_colors(self, detected_colors):
        return [
            color
            for color, enabled in self.target_labels.items()
            if enabled and color not in detected_colors
        ]

    def format_missing_target_message(self, missing_target_colors):
        if not missing_target_colors:
            return ''
        return f"缺少{','.join(missing_target_colors)}色块"

    def publish_missing_target_message(self, missing_target_colors):
        missing_target_message = self.format_missing_target_message(missing_target_colors)
        if missing_target_message == self.latest_missing_target_message:
            return

        self.latest_missing_target_message = missing_target_message
        msg = String()
        msg.data = missing_target_message
        self.missing_target_pub.publish(msg)
        if missing_target_message:
            self.get_logger().info(missing_target_message)

    def publish_place_request(self, target_color):
        msg = String()
        msg.data = target_color
        self.place_request_pub.publish(msg)
        self.get_logger().info(f'place request: {target_color}')

    def get_object_pixel_position(self, bgr_image, roi):
        target_info = []
        draw_image = bgr_image.copy()
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        roi_img = self.image_process.get_top_surface(roi_img)
        image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)

        for i in ['red', 'green', 'blue', 'yellow','orange']:
            index = 0
            mask = cv2.inRange(image_lab,
                               tuple(self.lab_data['color_range_list'][i]['min']),
                               tuple(self.lab_data['color_range_list'][i]['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)
            contours = map(lambda a_c: a_c[1], filter(lambda a: self.min_area <= a[0] <= self.max_area, contours_area))
            for c in contours:
                rect = cv2.minAreaRect(c)
                (center_x, center_y), _ = cv2.minEnclosingCircle(c)
                center_x, center_y = roi[2] + center_x, roi[0] + center_y
                corners = list(map(lambda p: (roi[2] + p[0], roi[0] + p[1]), cv2.boxPoints(rect)))
                cv2.drawContours(draw_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)

                index += 1
                angle = utils.get_long_edge_angle(rect)
                rect_size = rect[1]
                target_info.append([i, index, (int(center_x), int(center_y)), (int(rect_size[0]), int(rect_size[1])), angle])
        return draw_image, target_info

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.03):
        projection_matrix = np.row_stack((np.column_stack((extristric[1], extristric[0])), np.array([[0, 0, 0, 1]])))
        world_pose = np.asarray(common.pixels_to_world([position], intrinsic, projection_matrix)[0], dtype=np.float64)
        world_pose[0] = world_pose[0]
        world_pose[1] = world_pose[1]

        white_area_center = np.array(white_area_center, dtype=np.float64).reshape(4, 4)
        local_point = np.ones(4, dtype=np.float64)
        local_point[:3] = world_pose
        position = np.matmul(white_area_center, local_point)[:3]
        position[2] = height

        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['pixel']['offset'])
        scale = tuple(config_data['pixel']['scale'])
        for i in range(3):
            position[i] = position[i] * scale[i]
            position[i] = position[i] + offset[i]
        return position, projection_matrix

    def calculate_pick_grasp_yaw(self, position, target, target_info, intrinsic, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180

        gripper_size = [
            common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
            common.calculate_pixel_length(0.015, intrinsic, projection_matrix)
        ]
        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info, gripper_size, yaw)

    def calculate_place_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        yaw1 = yaw + angle
        if yaw < 0:
            yaw2 = yaw1 + 90
        else:
            yaw2 = yaw1 - 90

        yaw = yaw2
        if abs(yaw1) < abs(yaw2):
            yaw = yaw1

        return utils.normalize_gripper_roll_deg(yaw)

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                position, yaw, target = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]

                finish = pick_and_place.pick(position, 80, yaw, 540, 0.05, self.arm_pub)
                if finish:
                    self.enter = False
                    if self.placed_color != "left":
                        self.publish_arm(0, -200, 200, -90, 0, 30, 1000)
                    time.sleep(1.2)
                    requested_color = self.placed_color
                    self.publish_place_request(requested_color)
                    # self.replaceable = True
                    finish = False
                    threading.Thread(target=self.wait_for_place_status, args=(requested_color,), daemon=True).start()

                # else:
                #     self.go_home(True)

                self.target = None
                self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while self.running:
            if self.enter:
                try:
                    bgr_image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue

                if self.start_get_roi:
                    self.get_roi()
                    self.start_get_roi = False

                roi = self.roi.copy()
                intrinsic = self.intrinsic

                if len(roi) > 0 and self.enable_sorting and not self.start_transport:
                    display_image, target_info = self.get_object_pixel_position(bgr_image, roi)

                    if target_info:
                        if self.last_object_info_list:
                            target_info = position_change_detect.position_reorder(target_info, self.last_object_info_list, 20)
                        self.last_object_info_list = copy.deepcopy(target_info)

                    detected_colors = self.get_detected_colors(target_info)
                    missing_target_colors = self.get_missing_target_colors(detected_colors)

                    for target in target_info:
                        cv2.putText(
                            display_image,
                            '{}'.format(target[0]),
                            (target[2][0] - 4 * len(target[0] + str(target[1])), target[2][1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                    target_miss = True
                    for target in target_info:
                        if self.target_labels.get(target[0], False):
                            if self.target is not None:
                                if self.target[0] != target[0] or self.target[1] != target[1]:
                                    continue
                                else:
                                    target_miss = False
                                    self.target = target

                            if self.camera_type == 'usb_cam':
                                x, y = distortion_inverse_map.undistorted_to_distorted_pixel(
                                    target[2][0], target[2][1], self.intrinsic, self.distortion
                                )
                                target[2] = (x, y)

                            position, projection_matrix = self.get_object_world_position(
                                target[2], intrinsic, self.extristric, self.white_area_center
                            )
                            # self.get_logger().info(f'target[2]:{target[2]}')
                            # self.get_logger().info(f'position:{position}')

                            result = self.calculate_pick_grasp_yaw(position, target, target_info, intrinsic, projection_matrix)
                            if result is not None and self.target is None:
                                self.target = target
                                break

                            if self.last_position is not None and self.target is not None and result is not None:
                                e_distance = round(
                                    math.sqrt(pow(self.last_position[0] - position[0], 2)) +
                                    math.sqrt(pow(self.last_position[1] - position[1], 2)),
                                    5
                                )
                                if e_distance <= 0.005:
                                    cv2.line(display_image, result[1][0], result[1][1], (255, 255, 0), 2, cv2.LINE_AA)
                                    self.count_move = 0
                                    self.count_still += 1
                                else:
                                    self.count_move += 1
                                    self.count_still = 0

                                if self.count_move > 10:
                                    self.target = None
                                if self.count_still > 10:
                                    self.count_still = 0
                                    self.count_move = 0
                                    self.target = target
                                    yaw = utils.normalize_gripper_roll_deg(result[0])
                                    self.transport_info = [position, yaw, target]
                                    self.start_transport = True

                            self.last_position = position

                    if target_miss:
                        self.target_miss_count += 1
                    if self.target_miss_count > 10:
                        self.target_miss_count = 0
                        self.target = None
                        self.publish_missing_target_message(missing_target_colors)
                    if not missing_target_colors:
                        self.latest_missing_target_message = ''
                else:
                    display_image = bgr_image.copy()

                if bgr_image is not None and self.get_parameter('display').value:
                    cv2.imshow('result_image', display_image)
                    cv2.waitKey(1)
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
            else:
                time.sleep(0.1)

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_rgb_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)


def main():
    rclpy.init()
    node = ObjectSortingNode('object_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()


if __name__ == "__main__":
    main()
