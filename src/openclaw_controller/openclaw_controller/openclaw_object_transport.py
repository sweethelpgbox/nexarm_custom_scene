#!/usr/bin/env python3
# coding: utf8
# Object Sorting 目标分拣 (适配新架构版本)

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sdk import common, fps
from app.common import Heart
from dt_apriltags import Detector
from interfaces.srv import SetStringBool

from ros_robot_controller_msgs.msg import ArmCoords

from app.utils import calculate_grasp_yaw,position_change_detect, pick_and_place, image_process, distortion_inverse_map, utils

class ObjectSortingNode(Node):
    # 放置位置，根据新机械臂坐标系需要微调
    # 新增了 yellow 的固定放置位置
    place_position = {'green': [0.130, -0.21, 0.200],
                      'red':   [0.03, -0.210, 0.200],
                      'blue':  [-0.130, -0.210, 0.200],
                      'yellow':[-0.230, -0.210, 0.200], 
                      'tag1':  [-0.076, 0.16, 0.04],
                      'tag2':  [-0.006, 0.16, 0.04],
                      'tag3':  [0.064, 0.16, 0.04]}

    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        # 模型加载路径
        proto_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/deploy.prototxt'
        model_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/hed_pretrained_bsds.caffemodel'
        self.image_process = image_process.GetObjectSurface(proto_path, model_path)
        
        self.at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=4, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        
        # 文件路径配置
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        
        # 参数初始化
        self.camera_type = os.environ.get('CAMERA_TYPE', 'usb_cam')
        self.tag_size = 0.033 
        self.min_area = 500
        self.max_area = 7000
        # 增加 yellow 目标
        self.target_labels = {"red": False, "green": False, "blue": False, "yellow": False, "tag1": False, "tag2": False, "tag3": False}
        self.running = True

        self._init_parameters()

        # 加载校准数据(移至初始化阶段，避免在循环中重复读取硬盘)
        self.calibration_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))

        # 发布者 
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)

        timer_cb_group = ReentrantCallbackGroup()
        
        # 服务
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        # self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.set_running_srv = self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.set_target_srv = self.create_service(SetStringBool, '~/set_target', self.set_target_srv_callback)
        
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def _init_parameters(self):
        self.heart = None
        self.target_miss_count = 0
        self.transport_info = None
        self.intrinsic = None
        self.distortion = None
        self.start_transport = False
        self.is_running = False
        self.white_area_center = None
        self.enter = False
        self.roi = []
        self.count_move = 0
        self.count_still = 0
        self.target = None
        self.start_get_roi = False
        self.last_position = None
        self.last_object_info_list = None
        self.latest_image = None
        self.image_seq = 0
        self.image_sub = None
        self.camera_info_sub = None
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))  
        self.lab_data = self.data['/**']['ros__parameters']
        # 新增：记录每个目标连续丢失的帧数，用于自动取消完成的任务
        self.label_miss_counts = {k: 0 for k in self.target_labels.keys()}

    def init_process(self):
        self.timer.cancel()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        
        self.enter_srv_callback(Trigger.Request(), Trigger.Response())
        req = SetBool.Request()
        req.data = True 
        
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def get_node_state(self, request, response):
        response.success = True
        return response

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

    def go_home(self, interrupt=True):
        if interrupt:
            self.publish_arm(200, 0, 200, -90, 0, -65, 500)
            time.sleep(0.5)

        self.publish_arm(200, 0, 200, -90, 0, -65, 1000)
        time.sleep(1)

        self.publish_arm(200, 0, 200, -90, 0, -65, 1000)
        time.sleep(1)

    def go_left(self):
        self.publish_arm(0, 200, 200, -90, 0, 30, 1000)
        time.sleep(1.0)

    def go_right(self):
        self.publish_arm(0, -200, 200, -90, 0, 30, 1000)
        time.sleep(1.0)

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        
        while True:
            if self.intrinsic is not None and self.distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.04)
        self.extristric = tvec, rmat
        imgpts, _ = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), self.intrinsic, self.distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        x_min, x_max = min(imgpts[:,0]), max(imgpts[:,0])
        y_min, y_max = min(imgpts[:,1]), max(imgpts[:,1])
        self.roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

    def pixel_to_calibrated_world(self, center_pixel):
        if self.camera_type == 'usb_cam':
            x, y = distortion_inverse_map.undistorted_to_distorted_pixel(
                center_pixel[0], center_pixel[1], self.intrinsic, self.distortion)
            center_pixel = (x, y)

        pos_world, _ = self.get_object_world_position(center_pixel, self.intrinsic, self.extristric, self.white_area_center)
        
        offset = tuple(self.calibration_data['pixel']['offset'])
        scale = tuple(self.calibration_data['pixel']['scale'])
        for i in range(3):
            pos_world[i] = pos_world[i] * scale[i] + offset[i]
            
        return pos_world

    def apply_kinematics_calibration(self, position):
        offset = tuple(self.calibration_data['kinematics']['offset'])
        scale = tuple(self.calibration_data['kinematics']['scale'])
        calibrated_pos = [0, 0, 0]
        for i in range(3):
            calibrated_pos[i] = position[i] * scale[i] + offset[i]
        return calibrated_pos

    def calculate_place_grasp_yaw(self, position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        return yaw

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

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                position, yaw, target = self.transport_info
                target_name = target[0]
                
                pick_position = self.apply_kinematics_calibration(position)
                
                finish = pick_and_place.pick(pick_position, 80, yaw, 540, 0.05, self.arm_pub)
                if finish:
                    if target_name in ('red', 'green', 'blue'):
                        with self.lock:
                            image_seq = self.image_seq
                        self.go_left()
                        time.sleep(2)
                        
                        place_result = self.get_color_place_position(target_name, image_seq)
                        if place_result is None:
                            self.get_logger().error(f'No {target_name} place area detected, cancel place')
                            self.go_home(False)
                            self.target = None
                            self.start_transport = False
                            continue
                            
                        pos_place, place_target = place_result
                        # self.get_logger().info(f'\033[1;34m{"Place Area":<30}: {place_target[0]}, center: {place_target[2]}\033[0m')
                        cv2.circle(self.latest_image, place_target[2], 5, (255, 0, 255), -1)
                        # self.get_logger().info(f'\033[1;34m{"pos_place (Original)":<30}: {pos_place}\033[0m')

                        yaw_place = self.calculate_place_grasp_yaw(pos_place)
                        
                        pos_place = [
                            -pos_place[1],     
                            pos_place[0],    
                            pos_place[2]      
                        ]
                    else:
                        self.go_right()
                        time.sleep(2)
                        image_seq = self.image_seq
                        
                        place_result = self.get_color_place_position(target_name, image_seq)
                        if place_result is None:
                            self.get_logger().error(f'No {target_name} place area detected, cancel place')
                            self.go_home(False)
                            self.target = None
                            self.start_transport = False
                            continue
                            
                        pos_place, place_target = place_result
                        self.get_logger().info(f'\033[1;34m{"Place Area":<30}: {place_target[0]}, center: {place_target[2]}\033[0m')
                        cv2.circle(self.latest_image, place_target[2], 5, (255, 0, 255), -1)
                        self.get_logger().info(f'\033[1;34m{"pos_place (Original)":<30}: {pos_place}\033[0m')

                        yaw_place = self.calculate_place_grasp_yaw(pos_place)
                        
                        pos_place = [
                            pos_place[1],     
                            -pos_place[0],    
                            pos_place[2]      
                        ]
                        
                    final_place_position = self.apply_kinematics_calibration(pos_place)

                    # self.get_logger().info(f'\033[1;34m{"Place Position (Final)":<30}: {final_place_position}, Yaw: {yaw_place}\033[0m')
                    
                    # finish = pick_and_place.place(final_place_position, 80, yaw_place, 200, self.arm_pub)
                    # self.get_logger().info(f'Placing object at position: {final_place_position}, Yaw: {yaw_place}')
                    finish = self.place(final_place_position, yaw_place)
                    self.go_home(not finish)
                else:
                    self.go_home(True)
                
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
                if len(roi) > 0 and self.is_running and not self.start_transport:
                    display_image, target_info = self.get_object_pixel_position(bgr_image, roi)
                    
                    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
                    tags = self.at_detector.detect(gray, True, (intrinsic[0,0], intrinsic[1,1], intrinsic[0,2], intrinsic[1,2]), self.tag_size)
                    for tag in tags:
                        tag_id_str = f'tag{tag.tag_id}'
                        if tag_id_str in self.target_labels:
                            corners = tag.corners.astype(int)
                            cv2.drawContours(display_image, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
                            rect = cv2.minAreaRect(np.array(tag.corners).astype(np.float32))
                            angle = utils.get_long_edge_angle(rect)
                            target_info.append([tag_id_str, 1, (int(rect[0][0]), int(rect[0][1])), (int(rect[1][0]), int(rect[1][1])), angle])

                    if target_info and self.last_object_info_list:
                        target_info = position_change_detect.position_reorder(target_info, self.last_object_info_list, 20)
                    self.last_object_info_list = copy.deepcopy(target_info)
                  
                    for target in target_info:
                        if target[0].startswith('tag'):
                            cv2.putText(display_image, '{}'.format(target[0]), (target[2][0] - 4 * len(target[0] + str(target[1])), target[2][1] + 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # 自动检查并关闭已经搬运完成（画面中未检测到）的目标
                    visible_labels = {t[0] for t in target_info}  # 当前画面中所有看到的颜色/tag集合
                    
                    for label, is_active in self.target_labels.items():
                        if is_active:
                            # 如果该目标开启了，但是当前画面里没有它
                            if label not in visible_labels:
                                self.label_miss_counts[label] += 1
                                # 如果连续 120 帧（约两秒钟）都没看到它，说明已经搬完了或根本没有
                                if self.label_miss_counts[label] > 120: 
                                    self.target_labels[label] = False   # 自动设为 False
                                    self.label_miss_counts[label] = 0   # 计数清零
                                    self.get_logger().info(f'\033[1;33m【任务自动结束】画面中未检测到 {label}，已自动将其搬运状态设置为 False\033[0m')
                            else:
                                # 如果画面里还有它，计数器清零，继续搬运
                                self.label_miss_counts[label] = 0
                        else:
                            # 没开启的目标，计数器保持0
                            self.label_miss_counts[label] = 0

                    target_miss = True 
                    for target in target_info:
                        if self.target_labels.get(target[0], False):                            
                            if self.target is not None:
                                if self.target[0] != target[0] or self.target[1] != target[1]: continue
                                target_miss = False
                                self.target = target

                            pos_world = self.pixel_to_calibrated_world(target[2])
                            
                            # self.get_logger().info('\033[1;32m%s\033[0m' % 'pos_world: {}'.format(pos_world))

                            if self.target is None:
                                self.target = target
                                break
                    
                            if self.last_position is not None and self.target is not None:                               
                                dist = math.sqrt((self.last_position[0]-pos_world[0])**2 + (self.last_position[1]-pos_world[1])**2)
                                if dist <= 0.5:
                                    self.count_still += 1
                                    self.count_move = 0
                                else:
                                    self.count_move += 1
                                    self.count_still = 0

                                if self.count_move > 10: self.target = None
                                self.get_logger().info(f"self.count_still:{self.count_still}")

                                if self.count_still > 15:
                                    _, projection_matrix = self.get_object_world_position(
                                        target[2], intrinsic, self.extristric, self.white_area_center
                                    )
                                    result = self.calculate_pick_grasp_yaw(pos_world, target, target_info, intrinsic, projection_matrix)
                                    if result is None:
                                        continue
                                    self.count_still, self.count_move = 0, 0
                                    yaw_val = utils.normalize_gripper_roll_deg(result[0])
                                    self.transport_info = [pos_world, yaw_val, target]
                                    self.start_transport = True
                            self.last_position = pos_world

                    if target_miss: self.target_miss_count += 1
                    if self.target_miss_count > 10:
                        self.target_miss_count = 0
                        self.target = None
                else:
                    display_image = bgr_image.copy()

                cv2.imshow('result_image', display_image)
                cv2.waitKey(1)
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
            else:
                time.sleep(0.1)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter object sorting")
        self._init_parameters()
        # self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.start_get_roi = True
        self.go_home(False)
        self.enter = True
        response.success = True
        return response
     
    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit object sorting")
            if self.image_sub: self.destroy_subscription(self.image_sub)
            if self.camera_info_sub: self.destroy_subscription(self.camera_info_sub)
            self.heart.destroy()
            self.enter = False
            self.start_transport = False
            pick_and_place.interrupt(True)
        response.success = True
        return response
    
    def set_running_srv_callback(self, request, response):
        if request.data:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start object sorting')
            pick_and_place.interrupt(False)
            self.is_running = True
            response.success = True
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop object sorting')
            pick_and_place.interrupt(True)
            self.is_running = False
            response.success = False
        return response

    def set_target_srv_callback(self, request, response):
        if request.data_str in self.target_labels:
            self.target_labels[request.data_str] = request.data_bool
        response.success = True
        return response

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_rgb_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        with self.lock:
            self.latest_image = bgr_image.copy()
            self.image_seq += 1
        if self.image_queue.full(): self.image_queue.get()
        self.image_queue.put(bgr_image)

    def wait_latest_image(self, min_seq=None, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline and self.running:
            with self.lock:
                if self.latest_image is not None and (min_seq is None or self.image_seq > min_seq):
                    return self.latest_image.copy(), self.image_seq
            time.sleep(0.05)
        return None, None

    def normalize_roi(self, roi, image_shape):
        if roi is None or len(roi) != 4:
            return None
        image_h, image_w = image_shape[:2]
        y_min, y_max, x_min, x_max = [int(v) for v in roi]
        y_min = max(0, min(y_min, image_h))
        y_max = max(0, min(y_max, image_h))
        x_min = max(0, min(x_min, image_w))
        x_max = max(0, min(x_max, image_w))
        if y_min >= y_max or x_min >= x_max: return None
        return np.array([y_min, y_max, x_min, x_max], dtype=int)

    def get_color_area_pixel_position(self, bgr_image, roi, color_name):
        roi = self.normalize_roi(roi, bgr_image.shape)
        if roi is None or color_name not in self.lab_data['color_range_list']:
            return None
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][color_name]['min']), tuple(self.lab_data['color_range_list'][color_name]['max']))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour, area = common.get_area_max_contour(contours, self.min_area)
        if contour is None: return None
        rect = cv2.minAreaRect(contour)
        center_x, center_y = roi[2] + rect[0][0], roi[0] + rect[0][1]
        return [color_name, 1, (int(center_x), int(center_y)), (int(rect[1][0]), int(rect[1][1])), rect[2], area]

    def get_color_place_position(self, color_name, min_image_seq=None, timeout=3.0):
        if self.intrinsic is None or self.distortion is None or self.extristric is None or self.white_area_center is None:
            return None
        deadline = time.time() + timeout
        image_seq = min_image_seq

        while time.time() < deadline and self.running:
            bgr_image, image_seq = self.wait_latest_image(image_seq, max(0.1, min(0.5, deadline - time.time())))
            if bgr_image is None: continue

            roi = None if len(self.roi) == 0 else self.roi.copy()
            place_target = self.get_color_area_pixel_position(bgr_image, roi, color_name)
            if place_target is None: continue

            pos_place = self.pixel_to_calibrated_world(place_target[2])
            pos_place[2] = 0.04 
            
            return pos_place, place_target
        return None

    def get_object_pixel_position(self, bgr_image, roi):
        target_info = []
        draw_image = bgr_image.copy()
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        roi_img = self.image_process.get_top_surface(roi_img)
        image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
        for i in ['red', 'green', 'blue', 'yellow']:
            mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][i]['min']), tuple(self.lab_data['color_range_list'][i]['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            for c in contours:
                area = cv2.contourArea(c)
                if self.min_area <= area <= self.max_area:
                    rect = cv2.minAreaRect(c)
                    center_x, center_y = roi[2] + rect[0][0], roi[0] + rect[0][1]
                    box = np.intp(cv2.boxPoints(rect))
                    box[:, 0] += roi[2]
                    box[:, 1] += roi[0]
                    cv2.drawContours(draw_image, [box], -1, (0, 255, 255), 2)
                    angle = utils.get_long_edge_angle(rect)
                    target_info.append([i, 1, (int(center_x), int(center_y)), (int(rect[1][0]), int(rect[1][1])), angle])
        return draw_image, target_info

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.04):
        projection_matrix = np.row_stack((np.column_stack((extristric[1], extristric[0])), np.array([[0, 0, 0, 1]])))
        world_pose = common.pixels_to_world([position], intrinsic, projection_matrix)[0]
        world_pose[0], world_pose[1] = -world_pose[0], -world_pose[1]
        pos = white_area_center[:3, 3] + world_pose
        pos[2] = height
        return pos, projection_matrix

    def place(self,position,yaw_place):
        if position[0] > 0.05:
            x_mm = position[0] * 1000.0 + 35
        elif position[0] < -0.05:
            x_mm = position[0] * 1000.0 - 35
        else:
            x_mm = position[0] * 1000.0
        y_mm = position[1] * 1000.0
        z_mm = position[2] * 1000.0
        claw_close = 30
        claw_open = -60

        self.publish_arm(x_mm, y_mm, z_mm, -90.0, yaw_place, claw_close, 1000)
        time.sleep(1.2)
        self.publish_arm(x_mm, y_mm, z_mm, -90.0, yaw_place, claw_open, 1000)
        time.sleep(1.2)
        self.publish_arm(x_mm, y_mm, 200.0, -90.0, yaw_place, claw_open, 1500)
        time.sleep(1.8)
        return True


def main():
    node = ObjectSortingNode('openclaw_object_transport')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
