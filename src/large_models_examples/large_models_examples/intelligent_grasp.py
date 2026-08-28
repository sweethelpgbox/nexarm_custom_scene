#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import math
import yaml
import time
import copy
import torch
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
import message_filters
from sdk import common
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger, SetBool, Empty
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ultralytics.models.fastsam import FastSAMPredictor
from tf2_ros import Buffer, TransformListener, TransformException
from ultralytics.utils.ops import scale_masks


from speech import speech
from app.common import Heart
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetBox
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from app.utils import utils, image_process, calculate_grasp_yaw_by_depth, pick_and_place

device = 'cuda' if torch.cuda.is_available() else 'cpu'
MIN_GRASP_Z = 0.025

def prompt(results, bboxes=None, points=None, labels=None, texts=None):
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

class IntelligentGrasp(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS() # frame rate counter 帧率统计器
        self.image_queue = queue.Queue(maxsize=2)
        self._init_parameters()
        
        self.set_above = False
        self.lock = threading.RLock()
        self.base_gripper_height = utils.get_gripper_size(500)[1]
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
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
        self.set_target_srv = self.create_service(SetBox, '~/set_target', self.set_target_srv_callback)
        
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
        self.box_info = []
        self.target = []
        self.start_stamp = time.time()
        self.last_debug_stamp = 0.0

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
        self.go_home()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def debug_throttle(self, message, interval=1.0):
        now = time.time()
        if now - self.last_debug_stamp >= interval:
            self.last_debug_stamp = now
            self.get_logger().info(message)

    def normalize_box(self, box, image_width, image_height):
        try:
            box_values = list(box)
        except TypeError:
            return None
        if len(box_values) != 4:
            return None
        try:
            x1, y1, x2, y2 = [int(round(float(i))) for i in box_values]
        except (TypeError, ValueError):
            return None
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        x1 = max(0, min(image_width - 1, x1))
        x2 = max(0, min(image_width - 1, x2))
        y1 = max(0, min(image_height - 1, y1))
        y2 = max(0, min(image_height - 1, y2))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        return [x1, y1, x2, y2]

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
            time.sleep(1.0)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter object transport")
        with self.lock:
            self._init_parameters()
            self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # Heartbeat package 心跳包
            if self.sync is None:
                self.rgb_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
                self.depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')
                self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/depth/camera_info')
                self.info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/rgb/camera_info')
                # Synchronize the timestamp. The time is allowed to have an error of 0.03 seconds. 同步时间戳, 时间允许有误差在0.03s
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    [self.rgb_sub, self.depth_sub, self.info_sub, self.depth_info_sub], 3, 0.2)
                self.sync.registerCallback(self.multi_callback)
           
            self.enter = True
            self.start_get_roi = True
        
        self.go_home(interrupt=False, back=False)

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
                self.get_logger().info('\033[1;32m%s\033[0m' % f'enable object transport box_info={self.box_info}')
                self.enable_transport = True
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'exit  object transport')
                pick_and_place.interrupt()
                self.enable_transport = False
        response.success = True
        response.message = "start"
        return response

    def set_target_srv_callback(self, request, response):
        with self.lock:
            self.get_logger().info('\033[1;32m%s\033[0m' % f'set target label={request.label} box={list(request.box)}')
            self.box_info = [request.label, request.box] 
            self.target = []
            self.start_transport = False
        response.success = True
        response.message = "start"
        return response

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # Convert to numpy array. 转换为 numpy 数组
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
            tvec = self.extristric[:1]  # Take the first row. 取第一行
            rmat = self.extristric[1:]  # Take the last three rows. 取后面三行

            tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
            # self.get_logger().info(f'corners: {corners}')
            imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)

            # Crop RIO region 裁切出ROI区域
            x_min = min(imgpts, key=lambda p: p[0])[0] # The minimum value of the X-axis. x轴最小值
            x_max = max(imgpts, key=lambda p: p[0])[0] # The maximum value of the X-axis. x轴最大值
            y_min = min(imgpts, key=lambda p: p[1])[1] # The minimum value of the Y-axis. y轴最小值
            y_max = max(imgpts, key=lambda p: p[1])[1] # The maximum value of the Y-axis. y轴最大值
            roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            self.roi = roi

    def cal_grap_point(self, mask, x, y, box, edge_index, name):
        # Calculate the direction vector of the edge. 计算边的方向向量
        edge_vector = box[(edge_index + 1) % 4] - box[edge_index]
        edge_vector = edge_vector / np.linalg.norm(edge_vector)

        # Calculate direction vector perpendicular to the edge. 计算垂直于边的方向向量
        perpendicular_vector = np.array([-edge_vector[1], edge_vector[0]])

        # Define a long line starting from the centroid. 定义一条从质心出发的长线
        line_length = 1000  # Length of the line. 线的长度
        line_start = (x - int(perpendicular_vector[0] * line_length), y - int(perpendicular_vector[1] * line_length))
        line_end = (x + int(perpendicular_vector[0] * line_length), y + int(perpendicular_vector[1] * line_length))

        # Draw the line on the ROI image. 在ROI图像上绘制这条线
        line_image = np.zeros_like(mask)
        cv2.line(line_image, line_start, line_end, 255, 5, cv2.LINE_AA)
        # Find the intersection between the line and the ROI. 找到线与ROI的交点
        intersection_image = cv2.bitwise_and(mask, line_image)
        # cv2.imshow(name, intersection_image)
        intersection_contours, _ = cv2.findContours(intersection_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areaMaxContour, area_max = common.get_area_max_contour(intersection_contours)
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)  # Obtain the minimum bounding rectangle. 获取最小外接矩形
            center = [rect[0][0], rect[0][1]]
            object_width = max(rect[1])
            return [center, object_width]
        else:
            return False

    def get_object_pixel_position(self, image, roi):
        # roi[x, y, x, y]
        try:
            everything_results = self.predictor(image)
            # Prompt inference
            results = prompt(everything_results, bboxes=[roi])
            if not results or results[0].masks is None:
                self.debug_throttle(f'FastSAM found no mask for roi={roi}')
                return False
            mask = results[0].masks
        except Exception as e:
            self.debug_throttle(f'FastSAM failed for roi={roi}: {e}')
            return False

        mask = mask.data  # Usually a torch.Tensor. 通常是 torch.Tensor
        if not isinstance(mask, np.ndarray):
            mask = mask.cpu().numpy()
        # self.get_logger().info(f'results: {results[0].boxes}')
        if mask.ndim == 3 and mask.shape[0] == 1:  # Might be (1, H, W), need to remove the first dimension. 可能是 (1, H, W) 需要去掉第一维
            mask = mask[0]

        mask = (mask * 255).astype(np.uint8)
        x1, y1, x2, y2 = [int(v) for v in roi]
        x1 = max(0, min(mask.shape[1] - 1, x1))
        x2 = max(0, min(mask.shape[1] - 1, x2))
        y1 = max(0, min(mask.shape[0] - 1, y1))
        y2 = max(0, min(mask.shape[0] - 1, y2))
        if x2 <= x1 or y2 <= y1:
            self.debug_throttle(f'invalid roi={roi}')
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
                rect = cv2.minAreaRect(areaMaxContour)  # Obtain the minimum bounding rectangle 获取最小外接矩形
                # In version 4.5, w is defined as the edge that first aligns with the x-axis under clockwise rotation; angle is the clockwise rotation from the x-axis, ranging in (0, 90].
                # 4.5版本定义为，x轴顺时针旋转最先重合的边为w，angle为x轴顺时针旋转的角度，angle取值为(0,90]
                _, (width, height), angle = rect
                # self.get_logger().info(f'angle: {angle} {rect}')
                bbox = np.intp(cv2.boxPoints(rect))
                cv2.drawContours(image, [bbox], -1, (0, 255, 255), 2, cv2.LINE_AA)  # Draw rectangle contour. 绘制矩形轮廓
                
                # Calculate the long and short sides of the rectangle. 计算矩形的长边和短边
                edge_lengths = [np.linalg.norm(bbox[i] - bbox[(i + 1) % 4]) for i in range(4)]
                long_edge_index = np.argmax(edge_lengths)
                short_edge_index = np.argmin(edge_lengths)
                
                grasp_point1 = self.cal_grap_point(mask, cx, cy, bbox, long_edge_index, 'long')
                grasp_point2 = self.cal_grap_point(mask, cx, cy, bbox, short_edge_index, 'short')

                if not grasp_point1 and not grasp_point2:
                    # cv2.imshow('img', image)
                    return False
                if grasp_point1:
                    if width > height:
                        grasp_point1.append(angle - 90 )
                    else:
                        grasp_point1.append(angle)
                    cv2.circle(image, (int(grasp_point1[0][0]), int(grasp_point1[0][1])), 5, (0, 0, 255), -1)
                    # self.get_logger().info(f'grasp_point1: {grasp_point1}')
                if grasp_point2:
                    if width > height:
                        grasp_point2.append(angle)
                    else:
                        grasp_point2.append(angle - 90)
                    cv2.circle(image, (int(grasp_point2[0][0]), int(grasp_point2[0][1])), 5, (255, 0, 0), -1)
                    # self.get_logger().info(f'grasp_point2: {grasp_point2}')
                # cv2.imshow('img', image)
                return [grasp_point1, grasp_point2]
        self.debug_throttle(f'fallback to roi center={fallback_center} roi={roi}')
        return [[fallback_center, fallback_width, 0.0], [fallback_center, fallback_width, -90.0]]

    def get_grap_angle(self, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix):
        image_height, image_width = bgr_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        self.get_logger().info(f'[get_grap_angle] rgb=({image_width}x{image_height}) depth=({depth_w}x{depth_h}) object_info={object_info}')
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
        # Create mask. 创建掩码
        mask = np.zeros(depth_image.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [box], 255)
        # Apply mask to depth image. 应用掩码到深度图像
        masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=mask)
        min_dist = utils.find_depth_range(masked_depth, max_dist)
        fx, fy = depth_intrinsic_matrix[0], depth_intrinsic_matrix[4]
        self.get_logger().info(f'[get_grap_angle] grasp_pixel=({cx},{cy}) min_dist={min_dist} fx={fx} fy={fy}')
        if min_dist <= 0:
            self.get_logger().info(f'invalid depth for grasp point ({x},{y}); skip target')
            return None, min_dist

        object_width = object_width / (fx / (min_dist / 1000.0))

        gripper_angle = utils.set_gripper_size(object_width)
        self.get_logger().info(f'[get_grap_angle] object_width_m={object_width:.4f} gripper_angle={gripper_angle}')

        return gripper_angle, min_dist

    def get_position(self, object_info, min_dist, angle, gripper_angle, depth_intrinsic_matrix):
        gripper_info = utils.get_gripper_size(gripper_angle)
        d = gripper_info[1] - self.base_gripper_height

        x, y = object_info[0][0], object_info[0][1]
        cx, cy = int(round(x)), int(round(y))
        self.get_logger().info(f'[get_position] pixel=({cx},{cy}) min_dist={min_dist} angle={angle} gripper={gripper_angle}')
        position = utils.calculate_world_position(cx, cy, min_dist, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_intrinsic_matrix)
        if position is None:
            self.get_logger().info('[get_position] world position calculation failed')
            return None
        self.get_logger().info(f'[get_position] world_raw={[round(p, 4) for p in position]} gripper_z_delta={d:.4f}')
        position[-1] += d
        position = [float(v) for v in position]
        position[2] = max(float(position[2]), MIN_GRASP_Z)
        roll_deg = normalize_gripper_roll_deg(angle)
        self.get_logger().info(f'[get_position] world_position={[round(p, 4) for p in position]} roll={roll_deg:.1f}')

        object_info = [position, [x, y], roll_deg]
        return object_info

    def is_grasp_width_supported(self, gripper_angle):
        if gripper_angle is None:
            return False
        return -60.0 <= float(gripper_angle) <= 30.0

    def get_object_world_position(self, object_name, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values):
        object_info_ = []
        gripper_angle = []
        command_gripper_angle = 540
        # self.get_logger().info(f'object_info {object_info}')
        if object_info[0]:
            gripper_angle, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[0], max_dist, depth_intrinsic_matrix)
            if min_dist > 0 and self.is_grasp_width_supported(gripper_angle):
                object_info_ = self.get_position(object_info[0], min_dist, object_info[0][-1], command_gripper_angle, depth_intrinsic_matrix)
        if object_info[1]:
            gripper_angle_, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[1], max_dist, depth_intrinsic_matrix)
            if min_dist > 0 and self.is_grasp_width_supported(gripper_angle_):
                info = self.get_position(object_info[1], min_dist, object_info[1][-1], command_gripper_angle, depth_intrinsic_matrix)
                if info is not None and object_info_:
                    if abs(object_info_[-1]) > abs(info[-1]):
                        object_info_ = info
                        gripper_angle = command_gripper_angle
                elif info is not None:
                    object_info_ = info
                    gripper_angle = command_gripper_angle
        if object_info_:
            gripper_angle = command_gripper_angle
            object_info_.append(gripper_angle)
            object_info_.append(object_name)
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
                        if not self.box_info or len(self.box_info) < 2:
                            self.debug_throttle('enable_transport is true but no target box has been set')
                        else:
                            image_height, image_width = bgr_image.shape[:2]
                            target_box = self.normalize_box(self.box_info[1], image_width, image_height)
                            if target_box is None:
                                self.debug_throttle(f'invalid target box for current image: raw={self.box_info[1]} image={image_width}x{image_height}')
                            elif not self.target:
                                depth_intrinsic_matrix = depth_camera_info.k
                                plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                object_info = self.get_object_pixel_position(bgr_image, target_box) # xyxy
                                self.debug_throttle(f'grasp debug label={self.box_info[0]} box={target_box} object_info={object_info}')
                                if object_info:
                                    self.target = self.get_object_world_position(self.box_info[0], bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                    self.start_stamp = time.time()
                                    if self.target:
                                        self.get_logger().info(f'grasp target={self.target}')
                                    else:
                                        self.get_logger().info('grasp target failed: object too big or depth invalid')
                                else:
                                    self.debug_throttle(f'no segmentation mask found in target box={target_box}')
                            else:
                                if time.time() - self.start_stamp > 2:
                                    self.get_logger().info(f'start transport target={self.target}')
                                    self.start_transport = True        
                                cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                            if target_box is not None:
                                cv2.rectangle(bgr_image, (target_box[0], target_box[1]), (target_box[2], target_box[3]), (0, 255, 0), 2, 1)
                self.fps.update()
                self.fps.show_fps(bgr_image)
                #result_image = np.concatenate([bgr_image[40:440, ], depth_color_map], axis=1)
                result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                cv2.imshow('image', result_image)
                cv2.waitKey(1)
                if not self.set_above:
                    cv2.moveWindow('image', 1920 - 640*2, 0)
                    os.system("wmctrl -r image -b add,above")
                    self.set_above = True
            else:
                time.sleep(0.1)
        cv2.destroyAllWindows()

    def transport_thread(self):
        while True:
            if self.start_transport:
                self.enable_transport = False
                self.start_transport = False
                p = copy.deepcopy(self.target)
                if p and len(p) >= 4:
                    position = [float(v) for v in p[0]]
                    position[2] = max(float(position[2]), MIN_GRASP_Z)
                    roll_deg = float(p[2])
                    gripper_angle = float(p[3])
                    self.get_logger().info(f'pick command position={position} roll={roll_deg:.1f} gripper={gripper_angle:.1f}')
                    finish = pick_and_place.pick(position, 85, roll_deg, gripper_angle, 0.02, self.arm_pub, None)
                    self.get_logger().info(f'pick result finish={finish}')
                    if finish:
                        place_position = [0.25, 0.20, 0.18]
                        self.get_logger().info(f'place command position={place_position}')
                        pick_and_place.place(place_position, 85, 0.0, 200, self.arm_pub, None)
                    if not finish:
                        self.go_home(False, False)
                else:
                    self.get_logger().info('object too big')
                with self.lock:
                    self.target = []
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
    node = IntelligentGrasp('intelligent_grasp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
