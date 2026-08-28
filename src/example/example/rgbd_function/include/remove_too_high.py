#!/usr/bin/python3
# coding=utf8
"""
等高保持
找出识别区域高度最高的物体
如果物体高度超过阈值则将最高的物体移除
"""
import os
import cv2
import math
import time
import rclpy
import queue
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from app.utils import utils
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords, BuzzerState, ArmFullState
from example.scene_pose import load_scene_home_pose
from ros_robot_controller_msgs.srv import GetArmFullState
from rclpy.callback_groups import ReentrantCallbackGroup
from example.rgbd_function.include.grasp_height import side_grasp_z_from_top
from app import calibrated_pose


def decode_color_image(ros_image):
    height = int(ros_image.height)
    width = int(ros_image.width)
    encoding = str(getattr(ros_image, 'encoding', '') or '').lower()
    step = int(getattr(ros_image, 'step', 0) or 0)
    if step > 0 and width > 0:
        channels = max(1, step // width)
    else:
        channels = 4 if ('rgba' in encoding or 'bgra' in encoding) else 3

    shape = (height, width) if channels == 1 else (height, width, channels)
    raw_image = np.ndarray(shape=shape, dtype=np.uint8, buffer=ros_image.data)
    raw_image = np.copy(raw_image)

    if channels == 1:
        rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_GRAY2RGB)
    elif channels == 4:
        if encoding == 'rgba8':
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_RGBA2RGB)
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGRA2RGB)
    else:
        if encoding == 'rgb8':
            rgb_image = raw_image
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)

    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    return rgb_image, bgr_image


def fold_gripper_roll_deg(angle_deg):
    roll = utils.normalize_gripper_roll_deg(angle_deg)
    if roll > 45.0:
        roll -= 90.0
    elif roll < -45.0:
        roll += 90.0
    return utils.normalize_gripper_roll_deg(roll, limit_deg=60.0)


def image_angle_to_arm_roll_deg(position, image_angle):
    base_yaw = 0.0
    if position is not None and (abs(position[0]) > 1e-6 or abs(position[1]) > 1e-6):
        base_yaw = math.degrees(math.atan2(position[1], position[0]))
    return fold_gripper_roll_deg(base_yaw + float(image_angle))


def upper_side_grasp_z_from_top(top_z_m, object_height_m=None, min_z_m=0.008):
    if object_height_m is None:
        return side_grasp_z_from_top(top_z_m, object_height_m, min_z_m)
    top_z = max(float(top_z_m), float(min_z_m))
    height = max(float(object_height_m), 0.0)
    return max(float(min_z_m), round(top_z - height * 0.35, 6))


class RemoveTooHighObjectNode(Node):
    pick_offset = [-0.02, 0.05, 0.0, 0.0, -0.02]
    INIT_HOME = load_scene_home_pose()
    INIT_X = INIT_HOME['x']
    INIT_Y = INIT_HOME['y']
    INIT_Z = INIT_HOME['z']
    INIT_PITCH = INIT_HOME['pitch']
    INIT_ROLL = INIT_HOME['roll']
    INIT_CLAW = INIT_HOME['claw']
    GRAB_CLAW = -25.0
    OPEN_CLAW = -82.5
    MIN_OBJECT_HEIGHT_M = 0.018
    MAX_OBJECT_HEIGHT_M = 0.12
    MAX_CONTOUR_AREA_PX = 35000

    PLACE_X = 100.0
    PLACE_Y = -150.0
    PLACE_Z = 80.0
    PLACE_LIFT_Z = 150.0
    PLACE_PITCH = -60.0

    def __init__(self, name='remove_too_high'):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.endpoint = None
        self.fps = fps.FPS()
        self.moving = False
        self.running = True
        self.stamp = time.time()
        self.detect_debug_stamp = 0.0
        self.last_position = (0.0, 0.0, 0.0)
        self.start_process = self.get_bool_param('start', False)
        self.display = self.get_bool_param('display', True)
        self.current_pose = None
        self.known_pose = {
            'x': self.INIT_X,
            'y': self.INIT_Y,
            'z': self.INIT_Z,
            'pitch': self.INIT_PITCH,
            'roll': self.INIT_ROLL,
            'claw': self.INIT_CLAW,
            'yaw': 0.0,
        }
        self.config_path = '/home/ubuntu/ros2_ws/src/app/config/'
        self.calibration_file = 'calibration.yaml'
        self.transform_file = 'transform.yaml'
        self.camera_info_path = '/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml'
        self.hand2cam_tf_matrix = None
        self.plane = None
        self.corners = None
        self.extristric = None
        self.depth_offset = (0.0, 0.0, 0.0)
        self.depth_scale = (1.0, 1.0, 1.0)
        self.kinematics_offset = (0.0, 0.0, 0.0)
        self.kinematics_scale = (1.0, 1.0, 1.0)

        self.image_queue = queue.Queue(maxsize=2)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.08)
        sync.registerCallback(self.multi_callback)

        timer_cb_group2 = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group2)

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return bool(value)
        except Exception:
            return default

    def arm_state_callback(self, msg):
        self.current_pose = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'pitch': float(msg.pitch),
            'roll': float(msg.roll),
            'claw': float(msg.claw),
            'yaw': float(msg.yaw),
        }

    def request_real_pose_snapshot(self, timeout_sec=0.3):
        if not self.arm_state_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
            return None
        future = self.arm_state_client.call_async(GetArmFullState.Request())
        end_time = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end_time:
            if future.done():
                try:
                    result = future.result()
                except Exception:
                    return None
                if result is None or not result.success:
                    return None
                return {
                    'x': float(result.x),
                    'y': float(result.y),
                    'z': float(result.z),
                    'pitch': float(result.pitch),
                    'roll': float(result.roll),
                    'claw': float(result.claw),
                    'yaw': float(result.yaw),
                }
            time.sleep(0.01)
        return None

    def get_pose_snapshot(self):
        return self.current_pose or self.request_real_pose_snapshot() or self.known_pose

    def wait_for_motion_ready(self):
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.2)

    def load_calibration_parameters(self):
        try:
            camera_info = common.get_yaml_data(self.camera_info_path)
            matrix = camera_info.get('hand2cam_tf_matrix')
            if matrix is not None:
                self.hand2cam_tf_matrix = np.array(matrix, dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f'加载 hand2cam_tf_matrix 失败: {exc}')

        try:
            transform = common.get_yaml_data(os.path.join(self.config_path, self.transform_file))
            plane = transform.get('plane')
            if plane is not None:
                self.plane = tuple(float(v) for v in plane)
            corners = transform.get('corners')
            if corners is not None:
                self.corners = np.array(corners, dtype=np.float64)
            extristric = transform.get('extristric')
            if extristric is not None:
                self.extristric = np.array(extristric, dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f'加载 transform.yaml 失败: {exc}')

        try:
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
            self.depth_offset = tuple(float(v) for v in calibration['depth']['offset'])
            self.depth_scale = tuple(float(v) for v in calibration['depth']['scale'])
            self.kinematics_offset = tuple(float(v) for v in calibration['kinematics']['offset'])
            self.kinematics_scale = tuple(float(v) for v in calibration['kinematics']['scale'])
        except Exception as exc:
            self.get_logger().warn(f'加载 calibration.yaml 失败: {exc}')

    def apply_depth_calibration(self, position):
        calibration = {
            'depth': {
                'offset': self.depth_offset,
                'scale': self.depth_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'depth').tolist()

    def apply_kinematics_calibration(self, position):
        calibration = {
            'kinematics': {
                'offset': self.kinematics_offset,
                'scale': self.kinematics_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'kinematics').tolist()

    def init_process(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.wait_for_motion_ready()
        self.load_calibration_parameters()
        self.go_home(wait_time=1.5)
        threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info('remove_too_high ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum=None, frame=None):
        self.running = False

    def start_srv_callback(self, request, response):
        self.start_process = True
        response.success = True
        response.message = 'start'
        return response

    def stop_srv_callback(self, request, response):
        self.start_process = False
        self.moving = False
        self.last_position = (0.0, 0.0, 0.0)
        self.go_home(wait_time=1.0)
        response.success = True
        response.message = 'stop'
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
        self.get_logger().info(
            f'arm cmd x={msg.x:.1f} y={msg.y:.1f} z={msg.z:.1f} '
            f'pitch={msg.pitch:.1f} roll={msg.roll:.1f} claw={msg.claw:.1f} time_ms={msg.time_ms}'
        )
        self.arm_pub.publish(msg)
        self.known_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'pitch': float(pitch),
            'roll': float(roll),
            'claw': float(claw),
            'yaw': float(self.known_pose.get('yaw', 0.0)),
        }

    def go_home(self, wait_time=1.0):
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.OPEN_CLAW, 1500)
        time.sleep(wait_time)
        self.get_endpoint_matrix()

    def get_endpoint_matrix(self):
        p = self.get_pose_snapshot()
        x_m = float(p['x']) / 1000.0
        y_m = float(p['y']) / 1000.0
        z_m = float(p['z']) / 1000.0
        yaw_deg = float(p.get('yaw', math.degrees(math.atan2(y_m, x_m if abs(x_m) > 1e-6 else 1e-6))))
        pitch_deg = float(p['pitch'])
        roll_deg = float(p['roll'])

        yaw = math.radians(yaw_deg)
        pitch_rad = math.radians(-pitch_deg)
        roll_rad = math.radians(roll_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        rz_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        ry_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
        rx_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
        endpoint = np.eye(4, dtype=np.float64)
        endpoint[:3, :3] = rz_yaw @ ry_pitch @ rx_roll
        endpoint[:3, 3] = [x_m, y_m, z_m]
        self.endpoint = endpoint
        return endpoint

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except Exception:
                pass
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        return self.get_endpoint_matrix()

    def buzz(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def pick(self, position, image_angle, object_height=None):
        position = list(position)
        if position[0] > 0.22:
            position[2] += 0.01
        position[2] = upper_side_grasp_z_from_top(position[2], object_height)
        position = self.apply_kinematics_calibration(position)

        x_mm = position[0] * 1000.0
        y_mm = position[1] * 1000.0
        z_mm = position[2] * 1000.0
        pitch = self.INIT_PITCH
        grasp_roll = image_angle_to_arm_roll_deg(position, image_angle)
        pre_grasp_z_mm = z_mm + 18.0
        approach_z_mm = max(pre_grasp_z_mm + 50.0, 150.0)
        self.get_logger().info(
            f'pick target x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} '
            f'height={0.0 if object_height is None else object_height * 1000.0:.1f}mm '
            f'image_angle={float(image_angle):.1f} roll={grasp_roll:.1f}'
        )

        def abort_if_stopped():
            if not self.start_process:
                self.moving = False
                self.go_home(wait_time=1.0)
                return True
            return False

        try:
            self.buzz()
            time.sleep(0.6)
            if abort_if_stopped():
                return

            self.publish_arm(x_mm, y_mm, approach_z_mm, pitch, 0.0, self.OPEN_CLAW, 1800)
            time.sleep(1.8)
            if abort_if_stopped():
                return

            self.publish_arm(x_mm, y_mm, pre_grasp_z_mm, pitch, grasp_roll, self.OPEN_CLAW, 1500)
            time.sleep(1.5)
            if abort_if_stopped():
                return

            self.publish_arm(x_mm, y_mm, z_mm, pitch, grasp_roll, self.OPEN_CLAW, 1200)
            time.sleep(1.2)
            if abort_if_stopped():
                return

            time.sleep(1.0)
            self.publish_arm(x_mm, y_mm, z_mm, pitch, grasp_roll, self.GRAB_CLAW, 700)
            time.sleep(0.8)
            if abort_if_stopped():
                return

            self.publish_arm(x_mm, y_mm, z_mm + 40.0, pitch, grasp_roll, self.GRAB_CLAW, 1800)
            time.sleep(1.8)
            if abort_if_stopped():
                return

            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_LIFT_Z, self.PLACE_PITCH, 0.0, self.GRAB_CLAW, 1500)
            time.sleep(1.5)
            if abort_if_stopped():
                return
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_Z, self.PLACE_PITCH, 0.0, self.GRAB_CLAW, 1200)
            time.sleep(1.2)
            if abort_if_stopped():
                return
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_Z, self.PLACE_PITCH, 0.0, self.OPEN_CLAW, 600)
            time.sleep(0.8)
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_LIFT_Z, self.PLACE_PITCH, 0.0, self.OPEN_CLAW, 1500)
            time.sleep(1.5)
            self.go_home(wait_time=1.5)
        except Exception as e:
            self.get_logger().info('抓取过程中发生异常: ' + str(e))
        finally:
            self.stamp = time.time()
            self.moving = False

    def extract_top_candidate(self, depth_image, bgr_image, depth_camera_info):
        if self.hand2cam_tf_matrix is None or self.plane is None or self.corners is None or self.extristric is None:
            return None, None

        max_dist = 1000
        try:
            roi_depth = utils.create_roi_mask(
                depth_image.copy(),
                bgr_image,
                self.corners,
                depth_camera_info,
                self.extristric,
                max_dist,
                0.08,
            )
        except Exception:
            roi_depth = depth_image.copy()

        ih, iw = roi_depth.shape[:2]
        if ih > 400:
            roi_depth[380:400, :] = max_dist

        plane_values = utils.get_plane_values(roi_depth, self.plane, depth_camera_info.k)
        valid_depth_mask = np.logical_and(roi_depth > 0, roi_depth < max_dist)
        height_bool = np.logical_and.reduce((
            valid_depth_mask,
            plane_values > self.MIN_OBJECT_HEIGHT_M,
            plane_values < self.MAX_OBJECT_HEIGHT_M,
        ))
        height_mask = np.where(height_bool, 255, 0).astype(np.uint8)
        kernel = np.ones((5, 5), np.uint8)
        cleaned_mask = cv2.morphologyEx(height_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_height = 0.0
        debug_mask = cv2.cvtColor(cleaned_mask, cv2.COLOR_GRAY2BGR)
        rejected_area = 0
        rejected_no_height = 0
        rejected_low_height = 0
        rejected_no_depth = 0

        for obj in contours:
            area = cv2.contourArea(obj)
            if area < 500 or self.moving:
                rejected_area += 1
                continue
            if area > self.MAX_CONTOUR_AREA_PX:
                rejected_area += 1
                continue

            mask = np.zeros((ih, iw), dtype=np.uint8)
            cv2.drawContours(mask, [obj], -1, 255, cv2.FILLED)

            top_surface_heights = plane_values[mask == 255]
            top_surface_heights = top_surface_heights[
                np.logical_and(top_surface_heights > 0.005, top_surface_heights < self.MAX_OBJECT_HEIGHT_M)
            ]
            if len(top_surface_heights) == 0:
                rejected_no_height += 1
                continue

            object_height = float(np.percentile(top_surface_heights, 90))
            if object_height < self.MIN_OBJECT_HEIGHT_M:
                rejected_low_height += 1
                continue

            top_mask = np.logical_and(mask == 255, plane_values > max(object_height - 0.010, 0.005))
            ys, xs = np.where(top_mask)
            if len(xs) == 0:
                moments = cv2.moments(obj)
                if abs(moments['m00']) < 1e-6:
                    continue
                cx = moments['m10'] / moments['m00']
                cy = moments['m01'] / moments['m00']
            else:
                cx = float(np.mean(xs))
                cy = float(np.mean(ys))

            cx_i = int(np.clip(round(cx), 0, iw - 1))
            cy_i = int(np.clip(round(cy), 0, ih - 1))
            contour_depths = roi_depth[top_mask]
            contour_depths = contour_depths[np.logical_and(contour_depths > 0, contour_depths < max_dist)]
            if contour_depths.size == 0:
                contour_depths = roi_depth[mask == 255]
                contour_depths = contour_depths[np.logical_and(contour_depths > 0, contour_depths < max_dist)]
                if contour_depths.size == 0:
                    rejected_no_depth += 1
                    continue
            depth_value = float(np.median(contour_depths))

            endpoint = self.get_endpoint()
            pose_t = utils.calculate_world_position(
                cx_i,
                cy_i,
                depth_value,
                self.plane,
                endpoint,
                self.hand2cam_tf_matrix,
                depth_camera_info.k,
            )
            pose_t = self.apply_depth_calibration(list(pose_t))
            rect = cv2.minAreaRect(obj)
            image_angle = utils.get_long_edge_angle(rect)

            if object_height > best_height:
                best_height = object_height
                best = {
                    'contour': obj,
                    'pose_t': pose_t,
                    'pixel': (cx_i, cy_i),
                    'depth_value': depth_value,
                    'height': object_height,
                    'rect': rect,
                    'image_angle': image_angle,
                }

            cv2.drawContours(debug_mask, [obj], -1, (0, 255, 255), 2)
            cv2.circle(debug_mask, (cx_i, cy_i), 4, (0, 0, 255), -1)

        now = time.time()
        if now - self.detect_debug_stamp > 1.0:
            self.detect_debug_stamp = now
            top_h = max(1, ih // 3)
            valid_count = int(np.count_nonzero(valid_depth_mask))
            top_valid_count = int(np.count_nonzero(valid_depth_mask[:top_h, :]))
            height_count = int(np.count_nonzero(height_bool))
            top_height_count = int(np.count_nonzero(height_bool[:top_h, :]))
            raw_valid = depth_image[np.logical_and(depth_image > 0, depth_image < max_dist)]
            raw_top_valid = depth_image[:top_h, :][np.logical_and(depth_image[:top_h, :] > 0, depth_image[:top_h, :] < max_dist)]
            raw_range = (
                f'{int(np.min(raw_valid))}-{int(np.max(raw_valid))}'
                if raw_valid.size else 'none'
            )
            raw_top_range = (
                f'{int(np.min(raw_top_valid))}-{int(np.max(raw_top_valid))}'
                if raw_top_valid.size else 'none'
            )
            if best is None:
                best_text = 'none'
            else:
                best_text = (
                    f"pixel={best['pixel']} height={best['height'] * 1000.0:.1f}mm "
                    f"depth={best['depth_value']:.1f} pos=({best['pose_t'][0]:.3f},{best['pose_t'][1]:.3f},{best['pose_t'][2]:.3f})"
                )
            self.get_logger().info(
                f'detect debug contours={len(contours)} valid={valid_count} top_valid={top_valid_count} '
                f'height_mask_px={height_count} top_height_mask_px={top_height_count} raw_depth={raw_range} '
                f'raw_top_depth={raw_top_range} rejected(area={rejected_area}, no_height={rejected_no_height}, '
                f'low_height={rejected_low_height}, no_depth={rejected_no_depth}) best={best_text}'
            )

        return best, debug_mask

    def main(self):
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                _, bgr_image = decode_color_image(ros_rgb_image)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                depth_image = np.copy(depth_image)

                sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350.0 * 255.0
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                best, debug_mask = self.extract_top_candidate(depth_image, bgr_image, depth_camera_info)

                if debug_mask is not None:
                    debug_mask_resized = cv2.resize(debug_mask, (depth_color_map.shape[1], depth_color_map.shape[0]))
                    depth_color_map = cv2.addWeighted(depth_color_map, 0.75, debug_mask_resized, 0.25, 0)

                if best is not None:
                    obj = best['contour']
                    pose_t = best['pose_t']
                    px, py = best['pixel']
                    depth_value = best['depth_value']
                    object_height = best['height']
                    rect = best['rect']
                    image_angle = best['image_angle']

                    box = np.int32(cv2.boxPoints(rect))
                    cv2.drawContours(depth_color_map, [box], -1, (255, 255, 255), 2)
                    cv2.circle(depth_color_map, (px, py), 8, (32, 32, 32), -1)
                    cv2.circle(depth_color_map, (px, py), 6, (255, 255, 255), -1)
                    cv2.circle(bgr_image, (px, py), 8, (32, 32, 32), -1)
                    cv2.circle(bgr_image, (px, py), 6, (255, 255, 255), -1)

                    txt = 'Dist: {}mm Height:{:.1f}mm'.format(int(depth_value), object_height * 1000.0)
                    position_text = f'x:{pose_t[0]:.3f}m y:{pose_t[1]:.3f}m z:{pose_t[2]:.3f}m angle:{image_angle:.1f}'
                    cv2.putText(depth_color_map, txt, (11, depth_color_map.shape[0] - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                    cv2.putText(depth_color_map, txt, (10, depth_color_map.shape[0] - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)
                    cv2.putText(bgr_image, position_text, (10, bgr_image.shape[0] - 50), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                    cv2.putText(bgr_image, position_text, (10, bgr_image.shape[0] - 50), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                    dist = math.sqrt(
                        (self.last_position[0] - pose_t[0]) ** 2 +
                        (self.last_position[1] - pose_t[1]) ** 2 +
                        (self.last_position[2] - pose_t[2]) ** 2
                    )
                    self.last_position = pose_t

                    if dist < 0.002 and object_height > 0.01 and self.start_process:
                        if time.time() - self.stamp > 0.5:
                            self.stamp = time.time()
                            self.moving = True
                            self.get_logger().info(
                                f'stable target pixel=({px},{py}) pos=({pose_t[0]:.3f},{pose_t[1]:.3f},{pose_t[2]:.3f}) '
                                f'height={object_height * 1000.0:.1f}mm image_angle={image_angle:.1f}'
                            )
                            threading.Thread(target=self.pick, args=(list(pose_t), image_angle, object_height), daemon=True).start()
                    else:
                        self.stamp = time.time()
                else:
                    self.last_position = (0.0, 0.0, 0.0)
                    self.stamp = time.time()

                self.fps.update()
                result_image = np.concatenate([self.fps.show_fps(bgr_image), depth_color_map], axis=1)
                if self.display:
                    cv2.imshow('depth', result_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        self.running = False
                    elif key == ord('s'):
                        self.start_process = True
                    elif key == ord('a'):
                        self.start_process = False
                        self.moving = False
                        self.go_home(wait_time=1.0)
            except Exception as e:
                self.get_logger().info('处理异常: ' + str(e))

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RemoveTooHighObjectNode('remove_too_high')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
