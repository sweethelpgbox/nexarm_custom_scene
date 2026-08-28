#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import math
import yaml
import copy
import time
import torch
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
import message_filters
from sdk import common
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ultralytics.utils.ops import scale_masks
from ultralytics.models.fastsam import FastSAMPredictor
from tf2_ros import Buffer, TransformListener, TransformException

from speech import speech
from app.common import Heart
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetBox
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from app.utils import utils, image_process, calculate_grasp_yaw_by_depth, pick_and_place

device = 'cuda' if torch.cuda.is_available() else 'cpu'
MIN_GRASP_Z = 0.025

def prompt(results, bboxes=None, points=None, labels=None, texts=None, log=None):
    if bboxes is None and points is None and texts is None:
        return results
    prompt_results = []
    if not isinstance(results, list):
        results = [results]
    for result in results:
        if len(result) == 0:
            prompt_results.append(result)
            continue
        masks = result.masks.data
        if masks.shape[1:] != result.orig_shape:
            masks = scale_masks(masks[None], result.orig_shape)[0]
        idx = torch.zeros(len(result), dtype=torch.bool, device=device)
        if bboxes is not None:
            bboxes = torch.as_tensor(bboxes, dtype=torch.int32, device=device)
            bboxes = bboxes[None] if bboxes.ndim == 1 else bboxes
            bbox_areas = (bboxes[:, 3] - bboxes[:, 1]) * (bboxes[:, 2] - bboxes[:, 0])
            mask_areas = torch.stack([masks[:, b[1] : b[3], b[0] : b[2]].sum(dim=(1, 2)) for b in bboxes])
            full_mask_areas = torch.sum(masks, dim=(1, 2))
 
            u = mask_areas / full_mask_areas
            u = torch.nan_to_num(u, nan=0.0)
            indices = (u >= (torch.max(u) - 0.1)).nonzero(as_tuple=True)[1]
            u1 = full_mask_areas / bbox_areas
            max_index = indices[torch.argmax(u1[indices])]
            idx[max_index] = True

        prompt_results.append(result[idx])

    return prompt_results

class ObjectTransport(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.image_queue = queue.Queue(maxsize=2)
        self._init_parameters()
        
        self.record_position = []
        self.lock = threading.RLock()
        self.base_gripper_height = utils.get_gripper_size(500)[1]
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.transport_finished_pub = self.create_publisher(Bool, '~/transport_finished', 1)
        self.current_pose = None
        self.known_pose = dict(DEFAULT_ARM_POSE)
        self.endpoint = get_endpoint_matrix(self.known_pose)
        self.hand2cam_tf_matrix = load_hand2cam_tf_matrix()
        self.depth_offset = (0.0, 0.0, 0.0)
        self.depth_scale = (1.0, 1.0, 1.0)
        self.kinematics_offset = (0.0, 0.0, 0.0)
        self.kinematics_scale = (1.0, 1.0, 1.0)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')
        timer_cb_group = ReentrantCallbackGroup()

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_sorting_srv = self.create_service(SetBool, '~/enable_transport', self.enable_transport_srv_callback)
        self.set_pick_position_srv = self.create_service(SetBox, '~/set_pick_position', self.set_pick_position_srv_callback)
        self.set_place_position_srv = self.create_service(SetBox, '~/set_place_position', self.set_place_position_srv_callback)
        self.record_position_srv = self.create_service(SetBox, '~/record_position', self.record_position_srv_callback)
        
        code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])
        overrides = dict(conf=0.4, task="segment", mode="predict", model=os.path.join(code_path, 'resources/models', "FastSAM-x.pt"), save=False, imgsz=640)
        self.predictor = FastSAMPredictor(overrides=overrides)
        self.predictor(np.zeros((640, 400, 3), dtype=np.uint8))
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.camera_info_file = 'camera_info.yaml'
        self.config_path = APP_CONFIG_PATH + '/'
        self.peripherals_config_path = PERIPHERALS_CONFIG_PATH + '/'
        self.data = common.get_yaml_data("/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml")
        self.lab_data = self.data['/**']['ros__parameters']
        calibration = load_calibration_config()
        self.depth_offset = calibration['depth_offset']
        self.depth_scale = calibration['depth_scale']
        self.kinematics_offset = calibration['kinematics_offset']
        self.kinematics_scale = calibration['kinematics_scale']
        tf_buffer = Buffer()
        self.tf_listener = TransformListener(tf_buffer, self)
        transform = None
        try:
            self.get_logger().info('等待相机TF: depth_cam_color_frame <- rgb_camera_link')
            transform = tf_buffer.lookup_transform(
                'depth_cam_color_frame', 'rgb_camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0) )
            self.static_transform = transform  # 保存变换数据
            self.get_logger().info(f'Static transform: {self.static_transform}')
        except TransformException as e:
            self.get_logger().error(f'Failed to get static transform: {e}')

        if self.hand2cam_tf_matrix is not None and transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.transform_matrix = common.xyz_quat_to_mat([translation.x, translation.y, translation.z], [rotation.w, rotation.x, rotation.y, rotation.z])
            self.hand2cam_tf_matrix = np.matmul(self.transform_matrix, self.hand2cam_tf_matrix)
        elif self.hand2cam_tf_matrix is None:
            self.get_logger().warn('hand2cam_tf_matrix 未加载，将无法进行精确抓取位置计算')
        else:
            self.get_logger().warn('相机TF未获取到，使用已有 hand2cam_tf_matrix，抓取位置可能偏移')

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def arm_state_callback(self, msg):
        self.current_pose = arm_pose_dict(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.claw, msg.yaw)

    def request_real_pose_snapshot(self, timeout_sec=0.6):
        return request_real_pose_snapshot(self, self.arm_state_client, timeout_sec)

    def get_pose_snapshot(self, timeout_sec=0.6):
        return get_pose_snapshot(self, timeout_sec)

    def get_endpoint_matrix(self, pose=None):
        return get_endpoint_matrix(pose if pose is not None else self.get_pose_snapshot())

    def apply_depth_calibration(self, position):
        return apply_axis_calibration(position, self.depth_offset, self.depth_scale)

    def apply_kinematics_calibration(self, position):
        return apply_axis_calibration(position, self.kinematics_offset, self.kinematics_scale)

    def _init_parameters(self):
        self.heart = None
        self.enter = False
        self.start_transport = False
        self.enable_transport = False
        self.sync = None
        self.start_get_roi = False
        self.rgb_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.depth_info_sub = None
        self.white_area_center = None
        self.roi = None
        self.plane = None
        self.extristric = None
        self.corners = None
        self.intrinsic = None
        self.distortion = None
        self.action_list = []
        self.target = []
        self.start_stamp = time.time()

    def init_process(self):
        self.timer.cancel()
        wait_for_arm_runtime_ready(
            self,
            self.arm_pub,
            self.controller_init_client,
            self.kinematics_init_client,
        )

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.go_home(interrupt=False, back=False)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)
        update_known_pose(self, x, y, z, pitch, roll, claw)

    def go_home(self, interrupt=True, back=True):
        init = load_scene_home_pose()
        init_x, init_y, init_z = init['x'], init['y'], init['z']
        init_pitch, init_roll = init['pitch'], init['roll']
        open_claw = init['claw']
        if interrupt:
            self.publish_arm(init_x, init_y, init_z, init_pitch, init_roll, open_claw, 500)
            time.sleep(0.5)

        self.publish_arm(init_x, init_y, init_z, init_pitch, init_roll, open_claw, 1000)
        time.sleep(1.0)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        self.endpoint = self.get_endpoint_matrix(pose_snapshot)

        if back:
            time.sleep(1.5)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter object transport")
        with self.lock:
            self._init_parameters()
            self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
            if self.sync is None:
                self.rgb_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
                self.depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')
                self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/depth/camera_info')
                self.info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/rgb/camera_info')
                # 同步时间戳, 时间允许有误差在0.03s
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    [self.rgb_sub, self.depth_sub, self.info_sub, self.depth_info_sub], 3, 0.2)
                self.sync.registerCallback(self.multi_callback)

            self.enter = True
            self.start_get_roi = True
        
        self.go_home()
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit  object transport")
        with self.lock:
            self._init_parameters()
            if self.sync is not None:
                self.sync.disconnect(self.multi_callback)
                self.sync = None
            pick_and_place.interrupt()
        response.success = True
        response.message = "start"
        return response

    def enable_transport_srv_callback(self, request, response):
        with self.lock:
            if request.data:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'enable  object transport')
                self.enable_transport = True
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'exit  object transport')
                pick_and_place.interrupt()
                self.enable_transport = False
        response.success = True
        response.message = "start"
        return response

    def set_pick_position_srv_callback(self, request, response):
        with self.lock:
            self.get_logger().info(f'[set_pick_position] label={request.label} box={list(request.box)} offset={list(request.offset)}')
            self.action_list.append(['pick', request.box])
        response.success = True
        response.message = "start"
        return response

    def record_position_srv_callback(self, request, response):
        with self.lock:
            self.get_logger().info(f'[record_position] label={request.label} box={list(request.box)} offset={list(request.offset)}')
            self.action_list.append(['record', request.label, request.box])
        response.success = True
        response.message = "start"
        return response

    def set_place_position_srv_callback(self, request, response):
        with self.lock:
            self.get_logger().info(f'[set_place_position] label={request.label} box={list(request.box)} offset={list(request.offset)}')
            if request.label and not request.box:
                self.action_list.append(['restore', request.label])
            else:
                self.action_list.append(['place', request.offset, request.box])
        response.success = True
        response.message = "start"
        return response

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组
            corners = np.array(config['corners']).reshape(-1, 3)
            with self.lock:
                self.extristric = np.array(config['extristric'])
                self.white_area_center = np.array(config['white_area_pose_world'])
                self.plane = config['plane']
                self.corners = np.array(config['corners'])

        while self.intrinsic is None or self.distortion is None:
            self.get_logger().info("waiting for camera info")
            time.sleep(0.1)

        with self.lock:
            tvec = self.extristric[:1]  # 取第一行
            rmat = self.extristric[1:]  # 取后面三行

            tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
            # self.get_logger().info(f'corners: {corners}')
            imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)

            # 裁切出ROI区域(crop RIO region)
            x_min = min(imgpts, key=lambda p: p[0])[0] # x轴最小值(the minimum value of X-axis)
            x_max = max(imgpts, key=lambda p: p[0])[0] # x轴最大值(the maximum value of X-axis)
            y_min = min(imgpts, key=lambda p: p[1])[1] # y轴最小值(the minimum value of Y-axis)
            y_max = max(imgpts, key=lambda p: p[1])[1] # y轴最大值(the maximum value of Y-axis)
            roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            self.roi = roi

    def cal_grap_point(self, mask, x, y, box, edge_index):
        # 计算边的方向向量
        edge_vector = box[(edge_index + 1) % 4] - box[edge_index]
        edge_vector = edge_vector / np.linalg.norm(edge_vector)

        # 计算垂直于边的方向向量
        perpendicular_vector = np.array([-edge_vector[1], edge_vector[0]])

        # 定义一条从质心出发的长线
        line_length = 1000  # 线的长度
        line_start = (x - int(perpendicular_vector[0] * line_length), y - int(perpendicular_vector[1] * line_length))
        line_end = (x + int(perpendicular_vector[0] * line_length), y + int(perpendicular_vector[1] * line_length))

        # 在ROI图像上绘制这条线
        line_image = np.zeros_like(mask)
        cv2.line(line_image, line_start, line_end, 255, 1, cv2.LINE_AA)

        # 找到线与ROI的交点
        intersection_image = cv2.bitwise_and(mask, line_image)
        intersection_contours, _ = cv2.findContours(intersection_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areaMaxContour, area_max = common.get_area_max_contour(intersection_contours)
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)  # 获取最小外接矩形(obtain the minimum bounding rectangle)
            center = [rect[0][0], rect[0][1]]
            object_width = max(rect[1])
            return [center, object_width]
        else:
            return False

    def get_object_pixel_position(self, image, roi):
        # roi[x, y, x, y]
        everything_results = self.predictor(image)
        # Prompt inference
        results = prompt(everything_results, bboxes=[roi], log=self.get_logger())
        mask = results[0].masks

        mask = mask.data  # 通常是 torch.Tensor
        if not isinstance(mask, np.ndarray):
            mask = mask.cpu().numpy()
        # self.get_logger().info(f'results: {results[0].boxes}')
        if mask.ndim == 3 and mask.shape[0] == 1:  # 可能是 (1, H, W) 需要去掉第一维
            mask = mask[0]

        mask = (mask * 255).astype(np.uint8)
        x1, y1, x2, y2 = [int(v) for v in roi]
        x1 = max(0, min(mask.shape[1] - 1, x1))
        x2 = max(0, min(mask.shape[1] - 1, x2))
        y1 = max(0, min(mask.shape[0] - 1, y1))
        y2 = max(0, min(mask.shape[0] - 1, y2))
        if x2 <= x1 or y2 <= y1:
            self.get_logger().info(f'[get_object_pixel_position] invalid roi={roi}')
            return False
        roi_mask = np.zeros_like(mask, dtype=np.uint8)
        roi_mask[y1:y2, x1:x2] = 255
        mask = cv2.bitwise_and(mask, roi_mask)
        fallback_center = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]
        fallback_width = max(8.0, min(x2 - x1, y2 - y1))
        
        # cv2.imshow('mask', mask)
        M = cv2.moments(mask)
        annotated_frame = results[0].plot()
        # cv2.imshow("YOLO Inference", annotated_frame)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            grasp_point = (cx, cy)
            cv2.circle(image, grasp_point, 5, (255, 0, 0), -1)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = common.get_area_max_contour(contours)
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)  # 获取最小外接矩形(obtain the minimum bounding rectangle)
                #4.5版本定义为，x轴顺时针旋转最先重合的边为w，angle为x轴顺时针旋转的角度，angle取值为(0,90]
                _, (width, height), angle = rect

                bbox = np.intp(cv2.boxPoints(rect))
                cv2.drawContours(image, [bbox], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓(draw rectangle contour)
                
                # 计算矩形的长边和短边
                edge_lengths = [np.linalg.norm(bbox[i] - bbox[(i + 1) % 4]) for i in range(4)]
                long_edge_index = np.argmax(edge_lengths)
                short_edge_index = np.argmin(edge_lengths)
                
                grasp_point1 = self.cal_grap_point(mask, cx, cy, bbox, long_edge_index)
                grasp_point2 = self.cal_grap_point(mask, cx, cy, bbox, short_edge_index)

                if not grasp_point1 and not grasp_point2:
                    # cv2.imshow('img', image)
                    return False
                if grasp_point1:
                    grasp_point1.append(angle)
                    cv2.circle(image, (int(grasp_point1[0][0]), int(grasp_point1[0][1])), 5, (0, 0, 255), -1)
                if grasp_point2: 
                    grasp_point2.append(angle - 90)
                    cv2.circle(image, (int(grasp_point2[0][0]), int(grasp_point2[0][1])), 5, (255, 0, 0), -1)
                cv2.rectangle(image, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 0), 2, 1)
                # cv2.imshow('img', image)
                return [grasp_point1, grasp_point2]
        self.get_logger().info(f'[get_object_pixel_position] fallback to roi center={fallback_center} roi={roi}')
        return [[fallback_center, fallback_width, 0.0], [fallback_center, fallback_width, -90.0]]

    def get_grap_angle(self, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix):
        image_height, image_width = bgr_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        self.get_logger().info(f'[get_grap_angle] rgb=({image_width}x{image_height}), depth=({depth_w}x{depth_h})')
        x, y, object_width = object_info[0][0], object_info[0][1], object_info[1]
        w = 50
        if x + w > image_width:
            w = image_width - x
        elif x - w < 0:
            w = x
        h = 50
        if y + h > image_height:
            h = image_height - y
        elif y - h < 0:
            h = y

        cx, cy = int(round(x)), int(round(y))
        angle = 0
        box = np.intp(cv2.boxPoints(((cx, cy), (w, h), angle)))
        # 创建掩码
        mask = np.zeros(depth_image.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [box], 255)
        # 应用掩码到深度图像
        masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=mask)
        min_dist = utils.find_depth_range(masked_depth, max_dist)
        self.get_logger().info(f'[get_grap_angle] grasp_pixel=({cx},{cy}), min_dist={min_dist}')
        if min_dist <= 0:
            self.get_logger().info(f'invalid depth for grasp point ({x},{y}); skip target')
            return None, min_dist
        fx, fy = depth_intrinsic_matrix[0], depth_intrinsic_matrix[4]

        object_width = object_width / (fx / (min_dist / 1000.0))

        gripper_angle = utils.set_gripper_size(object_width)

        return gripper_angle, min_dist

    def get_position(self, object_info, min_dist, angle, gripper_angle, depth_intrinsic_matrix):
        gripper_info = utils.get_gripper_size(gripper_angle)
        d = gripper_info[1] - self.base_gripper_height

        x, y = object_info[0][0], object_info[0][1]
        cx, cy = int(round(x)), int(round(y))
        self.get_logger().info(f'[get_position] pixel=({cx},{cy}), min_dist={min_dist}, angle={angle}')
        position = utils.calculate_world_position(cx, cy, min_dist, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_intrinsic_matrix)
        if position is None:
            return None
        self.get_logger().info(f'[get_position] world_pos={[round(p,4) for p in position]}, d={d:.4f}')
        position[-1] += d
        position = [float(v) for v in position]
        position[2] = max(float(position[2]), MIN_GRASP_Z)
        roll_deg = normalize_gripper_roll_deg(angle)

        object_info = [position, [x, y], roll_deg]
        return object_info

    def get_object_world_position(self, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values):
        object_info_ = []
        # self.get_logger().info(f'object_info {object_info}')
        if object_info[0]:
            gripper_angle, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[0], max_dist, depth_intrinsic_matrix)
            if min_dist > 0:
                gripper_angle = 540
                object_info_ = self.get_position(object_info[0], min_dist, object_info[0][-1], gripper_angle, depth_intrinsic_matrix)

        if object_info[1]:
            gripper_angle, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[1], max_dist, depth_intrinsic_matrix)
            if min_dist > 0:
                gripper_angle = 540
                info = self.get_position(object_info[1], min_dist, object_info[1][-1], gripper_angle, depth_intrinsic_matrix)
                if info is not None and object_info_:
                    if abs(object_info_[-1]) > abs(info[-1]):
                        object_info_ = info
                elif info is not None:
                    object_info_ = info
        if object_info_:
            object_info_.append(gripper_angle)
        return object_info_

    def main(self):
        while True:
            if self.enter:
                try:
                    bgr_image, depth_image, camera_info, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue
                with self.lock:
                    self.intrinsic = np.matrix(camera_info.k).reshape(1, -1, 3)
                    self.distortion = np.array(camera_info.d)
                    if self.start_get_roi:
                        self.get_roi()
                        self.start_get_roi = False
                max_dist = 350
                depth_image = utils.create_roi_mask(depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.08)
                sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                if self.enable_transport:
                    with self.lock:
                        if self.action_list:
                            box_info = self.action_list[0]
                            self.get_logger().info(f'[transport] action_head={box_info}')
                            if box_info[0] == 'pick':
                                if not self.target:
                                    depth_intrinsic_matrix = depth_camera_info.k
                                    plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                    object_info = self.get_object_pixel_position(bgr_image, box_info[1]) # xyxy
                                    self.get_logger().info(f'[transport] pick box={box_info[1]} object_info={object_info}')
                                    if object_info:
                                        self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                        self.target.extend([box_info[0]])
                                        self.get_logger().info(f'[transport] pick target={self.target}')
                                        self.start_stamp = time.time()
                                else:
                                    if time.time() - self.start_stamp > 2:
                                        self.start_transport = True        
                                        self.enable_transport = False
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                cv2.rectangle(bgr_image, (box_info[-1][0], box_info[-1][1]), (box_info[-1][2], box_info[-1][3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'place':
                                if not self.target:
                                    depth_intrinsic_matrix = depth_camera_info.k
                                    plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                    object_info = self.get_object_pixel_position(bgr_image, box_info[-1]) # xyxy
                                    self.get_logger().info(f'[transport] place box={box_info[-1]} offset={list(box_info[-2])} object_info={object_info}')
                                    if object_info:
                                        self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                        self.target.extend([box_info[0]])
                                        self.target[0][0] += box_info[-2][0]
                                        self.target[0][1] += box_info[-2][1]
                                        if box_info[-2][0] != 0 or box_info[-2][1] != 0:
                                            self.target[0][2] = 0.015
                                        else:
                                            self.target[0][2] += 0.015
                                        self.get_logger().info(f'[transport] place target={self.target}')
                                        self.start_stamp = time.time()
                                else:
                                    if time.time() - self.start_stamp > 2:
                                        self.start_transport = True        
                                        self.enable_transport = False
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                cv2.rectangle(bgr_image, (box_info[-1][0], box_info[-1][1]), (box_info[-1][2], box_info[-1][3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'record':
                                depth_intrinsic_matrix = depth_camera_info.k
                                plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                object_info = self.get_object_pixel_position(bgr_image, box_info[-1]) # xyxy
                                if object_info:
                                    self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                    self.target.extend(['restore'])
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                    self.record_position.append([box_info[1], self.target])
                                    del self.action_list[0]
                                    if not self.action_list:
                                        speech.play_audio(record_finish_audio_path)
                                        self.enable_transport = False
                                        self.target = []
                                cv2.rectangle(bgr_image, (box_info[-1][0], box_info[-1][1]), (box_info[-1][2], box_info[-1][3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'restore':
                               for i in self.record_position:
                                   if box_info[1].lower() in i[0].lower():
                                       self.target = i[1]
                                       self.start_transport = True
                                       self.enable_transport = False
                                       break
                        else:
                            msg = Bool()
                            msg.data = True
                            self.transport_finished_pub.publish(msg)
                self.fps.update()
                self.fps.show_fps(bgr_image)
                result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                cv2.imshow('result_image', result_image)
                cv2.waitKey(1)
            else:
                time.sleep(0.1)
        cv2.destroyAllWindows()

    def transport_thread(self):
        while True:
            if self.start_transport:
                p = copy.deepcopy(self.target)
                finish = False
                if p and len(p) >= 4:
                    position = [float(v) for v in p[0]]
                    position[2] = max(float(position[2]), MIN_GRASP_Z)
                    roll_deg = float(p[2])
                    action = self.target[-1]
                    self.get_logger().info(f'[transport] start action={action} raw_target={p} command_position={position} roll={roll_deg:.1f}')
                    if action == 'pick':
                        finish = pick_and_place.pick(position, 85, roll_deg, 540, 0.02, self.arm_pub, None)
                        self.get_logger().info(f'[transport] pick result finish={finish}')
                    elif action == 'place':
                        finish = pick_and_place.place(position, 85, roll_deg, 200, self.arm_pub, None)
                        self.get_logger().info(f'[transport] place result finish={finish}')
                    elif action == 'restore':
                        finish = pick_and_place.place(position, 85, roll_deg, 200, self.arm_pub, None)
                        self.get_logger().info(f'[transport] restore/place result finish={finish}')
                if not finish:
                    self.go_home(False)
                with self.lock:
                    self.target = []
                    del self.action_list[0]
                self.enable_transport = True
                self.start_transport = False
            else:
                time.sleep(0.1)

    def multi_callback(self, ros_rgb_image, ros_depth_image, camera_info, depth_camera_info):
        _, bgr_image = decode_color_image(ros_rgb_image)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16,
                                 buffer=ros_depth_image.data)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((bgr_image, depth_image, camera_info, depth_camera_info))

def main():
    node = ObjectTransport('object_transport')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
