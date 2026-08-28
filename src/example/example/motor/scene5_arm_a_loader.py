#!/usr/bin/env python3
# coding: utf-8

import copy
import math
import os
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from app import calibrated_pose
from app.utils import image_process
from app.utils import calculate_grasp_yaw, distortion_inverse_map, pick_and_place, utils
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ArmCoords
from sdk import common
from sdk.scene_context import load_scene_environment
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int8
from std_srvs.srv import Trigger


CONFIG_PATH = '/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml'
PLAY_CONFIG_PATH = '/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml'
APP_CONFIG_DIR = '/home/ubuntu/ros2_ws/src/app/config'
LAB_CONFIG_PATH = os.path.join(APP_CONFIG_DIR, 'lab_config.yaml')
TRANSFORM_PATH = os.path.join(APP_CONFIG_DIR, 'transform.yaml')
CALIBRATION_FILE = 'calibration.yaml'
COLOR_KEYS = ('yellow', 'red', 'green', 'blue')
COLOR_CLAW_GRAB_ANGLE = -17.0
GARBAGE_CLAW_GRAB_ANGLE = -45.0
COLOR_OBJECT_HEIGHT_M = 0.03
GARBAGE_OBJECT_HEIGHT_M = 0.04
COLOR_PICK_Z_OFFSET_M = 0.01
GARBAGE_PICK_Z_OFFSET_M = 0.01
GARBAGE_CLASSES = {
    'BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks',
    'Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate',
    'StorageBattery', 'Toothbrush', 'Umbrella',
}


class Scene5ArmALoader(Node):
    def __init__(self):
        super().__init__(
            'scene5_arm_a_loader',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.config_path = self.string_param('config_path', CONFIG_PATH)
        self.play_config_path = self.string_param('play_config_path', PLAY_CONFIG_PATH)
        self.scene_env = load_scene_environment()
        self.config = self.load_config(self.config_path)
        play_config = self.load_config(self.play_config_path)
        dual = play_config.get(
            'scene5_dual_arm',
            self.config.get('scenes', {}).get('scene_5', {}).get('scene5_dual_arm', {}),
        )
        arm_a = dual.get('arm_a', {})
        vision = dual.get('arm_a_vision', {})
        conveyor = dual.get('conveyor', {})
        conveyor_follow = dual.get('arm_a_conveyor_follow', {})
        self.points = dual.get('arm_a_points', {})
        default_prefix = arm_a.get('controller_prefix', '/arm_a/ros_robot_controller')
        self.controller_prefix = self.string_param('arm_a_prefix', default_prefix).rstrip('/')
        if not self.controller_prefix.startswith('/'):
            self.controller_prefix = '/' + self.controller_prefix

        default_image_topic = (
            self.scene_env.camera_topic('rgb/image_raw')
            if self.scene_env.is_scene5 else '/depth_cam/rgb/image_raw'
        )
        default_camera_info_topic = (
            self.scene_env.camera_topic('rgb/camera_info')
            if self.scene_env.is_scene5 else '/depth_cam/rgb/camera_info'
        )
        default_object_topic = (
            self.scene_env.topic('scene5_arm_a_yolo/yolo/object_detect')
            if self.scene_env.is_scene5 else '/scene5_arm_a_yolo/yolo/object_detect'
        )
        default_yolo_start_service = (
            self.scene_env.topic('scene5_arm_a_yolo/start')
            if self.scene_env.is_scene5 else '/scene5_arm_a_yolo/start'
        )
        default_yolo_stop_service = (
            self.scene_env.topic('scene5_arm_a_yolo/stop')
            if self.scene_env.is_scene5 else '/scene5_arm_a_yolo/stop'
        )
        self.image_topic = self.string_param('image_topic', default_image_topic)
        self.camera_info_topic = self.string_param('camera_info_topic', default_camera_info_topic)
        self.object_topic = self.string_param('object_topic', default_object_topic)
        self.yolo_start_service = self.string_param('yolo_start_service', default_yolo_start_service)
        self.yolo_stop_service = self.string_param('yolo_stop_service', default_yolo_stop_service)
        self.yolo_box_format = self.string_param(
            'yolo_box_format',
            str(vision.get('yolo_box_format', 'center_wh')),
        ).strip().lower()
        default_service_prefix = self.scene_env.role_namespace if self.scene_env.is_scene5 else '/arm_a'
        self.service_prefix = self.string_param('service_prefix', default_service_prefix).rstrip('/')

        self.target_center_x = float(vision.get('target_center_x', 300.0))
        self.stable_required = int(vision.get('stable_count', 10))
        self.world_stable_distance_m = float(vision.get('world_stable_distance_m', 0.005))
        self.min_area = float(vision.get('color_min_area', 500.0))
        self.max_area = float(vision.get('color_max_area', 7000.0))
        self.color_object_height_m = float(vision.get('color_object_height_m', 0.04))
        self.garbage_object_height_m = float(vision.get('garbage_object_height_m', 0.04))
        self.yolo_max_age_sec = float(vision.get('yolo_max_age_sec', 0.6))
        self.conveyor_follow_enabled = bool(conveyor_follow.get('enabled', False))
        self.conveyor_follow_axis = str(conveyor_follow.get('axis', 'x')).strip().lower()
        if self.conveyor_follow_axis not in ('x', 'y', 'z'):
            self.conveyor_follow_axis = 'x'
        self.conveyor_speed_profiles = self.load_conveyor_speed_profiles(conveyor_follow)
        self.conveyor_max_offset_mm = float(conveyor_follow.get('max_offset_mm', 60.0))
        self.conveyor_min_axis_mm = float(conveyor_follow.get('min_axis_mm', 0.0))
        default_conveyor_topic = conveyor_follow.get(
            'topic',
            conveyor.get('topic', '/arm_b/ros_robot_controller/conveyor/set'),
        )
        self.conveyor_topic = self.string_param('conveyor_topic', default_conveyor_topic)
        self.current_conveyor_cmd = 0

        self.bridge = CvBridge()
        self.image_process = image_process.GetObjectSurface()
        self.image_queue = queue.Queue(maxsize=2)
        self.yolo_objects = []
        self.yolo_stamp = 0.0
        self.yolo_lock = threading.RLock()
        self.lock = threading.RLock()
        self.running = False
        self.moving = False
        self.single_cycle = False
        self.single_done = threading.Event()
        self.last_target = None
        self.stable_count = 0
        self.last_object_info_list = None
        self.lab_data = {}
        self.load_lab_config()

        self.camera_type = os.environ.get('CAMERA_TYPE', '').lower()
        self.intrinsic = None
        self.distortion = None
        self.extristric = None
        self.white_area_center = None
        self.roi = []
        self.roi_ready = False
        self.missing_calibration_warned = False
        self.service_cb_group = ReentrantCallbackGroup()

        self.arm_pub = self.create_publisher(ArmCoords, self.ctl_topic('arm/set_coords'), 5)
        self.result_image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)
        self.create_subscription(ObjectsInfo, self.object_topic, self.object_callback, 1)
        self.create_subscription(Int8, self.conveyor_topic, self.conveyor_callback, 1)
        self.scene_runtime_prepare_client = self.create_client(
            Trigger,
            self.ctl_topic('scene_runtime/prepare'),
            callback_group=self.service_cb_group,
        )
        self.yolo_start_client = self.create_client(Trigger, self.yolo_start_service, callback_group=self.service_cb_group)
        self.yolo_stop_client = self.create_client(Trigger, self.yolo_stop_service, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/arm_a/home'), self.on_home, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/arm_a/start'), self.on_start, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/startA'), self.on_start, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/arm_a/load_once'), self.on_load_once, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/arm_a/stop'), self.on_stop, callback_group=self.service_cb_group)
        self.create_service(Trigger, self.service_name('scene5/stopA'), self.on_stop, callback_group=self.service_cb_group)
        threading.Thread(target=self.vision_loop, daemon=True).start()
        self.get_logger().info(
            f'scene5 arm A app-style loader ready: controller={self.controller_prefix}, '
            f'image={self.image_topic}, camera_info={self.camera_info_topic}, '
            f'objects={self.object_topic}, conveyor={self.conveyor_topic}'
        )

    def ctl_topic(self, suffix):
        return f'{self.controller_prefix}/{suffix.lstrip("/")}'

    def service_name(self, suffix):
        prefix = self.service_prefix.strip('/')
        suffix = str(suffix).lstrip('/')
        return f'/{prefix}/{suffix}' if prefix else suffix

    def string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

    def load_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def load_conveyor_speed_profiles(self, conveyor_follow):
        profiles = conveyor_follow.get('speed_profiles', {})
        if not isinstance(profiles, dict):
            return {}
        loaded = {}
        for cmd, raw_profile in profiles.items():
            if not isinstance(raw_profile, dict):
                continue
            try:
                cmd_value = int(cmd)
                mmps = float(raw_profile.get('mmps', 0.0))
                if mmps <= 0:
                    continue
                loaded[cmd_value] = {
                    'mmps': mmps,
                    'lead_ms': int(raw_profile.get('lead_ms', 300)),
                    'release_ms': int(raw_profile.get('release_ms', 450)),
                    'tail_ms': int(raw_profile.get('tail_ms', 250)),
                }
            except (TypeError, ValueError):
                continue
        return loaded

    def load_lab_config(self):
        try:
            data = self.load_config(LAB_CONFIG_PATH)
            self.lab_data = data['/**']['ros__parameters']
        except Exception as ex:
            self.get_logger().warn(f'load LAB config failed: {ex}')
            self.lab_data = {}

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)
        if not self.roi_ready:
            self.load_app_roi()

    def load_app_roi(self):
        if self.intrinsic is None or self.distortion is None:
            return False
        try:
            config = self.load_config(TRANSFORM_PATH)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
            tvec = extristric[:1]
            rmat = extristric[1:]
            tvec, rmat = common.extristric_plane_shift(
                np.array(tvec).reshape((3, 1)),
                np.array(rmat),
                0.03,
            )
            self.extristric = tvec, rmat
            imgpts, _ = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)
            x_min = min(imgpts, key=lambda p: p[0])[0]
            x_max = max(imgpts, key=lambda p: p[0])[0]
            y_min = min(imgpts, key=lambda p: p[1])[1]
            y_max = max(imgpts, key=lambda p: p[1])[1]
            self.roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            self.roi_ready = True
            self.get_logger().info(f'arm A app ROI ready: roi={list(self.roi)}')
            return True
        except Exception as ex:
            if not self.missing_calibration_warned:
                self.get_logger().warn(f'arm A app calibration load failed: {ex}')
                self.missing_calibration_warned = True
            return False

    def conveyor_callback(self, msg):
        with self.lock:
            self.current_conveyor_cmd = int(msg.data)

    def image_callback(self, ros_image):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except Exception as ex:
            self.get_logger().warn(f'arm A BGR image decode failed: {ex}')
            return
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        self.image_queue.put(np.array(bgr_image, dtype=np.uint8))

    def object_callback(self, msg):
        objects = []
        for obj in msg.objects:
            class_name = str(obj.class_name)
            if class_name not in GARBAGE_CLASSES:
                continue
            if len(obj.box) < 4:
                continue
            objects.append({
                'class_name': class_name,
                'score': float(obj.score),
                'box': list(obj.box),
                'width': float(obj.width),
                'height': float(obj.height),
                'angle': float(obj.angle),
            })
        with self.yolo_lock:
            self.yolo_objects = objects
            self.yolo_stamp = time.time()

    def publish_pose(self, pose, claw_override=None):
        msg = ArmCoords()
        msg.x = float(pose.get('x', 220.0))
        msg.y = float(pose.get('y', 0.0))
        msg.z = float(pose.get('z', 230.0))
        msg.pitch = float(pose.get('pitch', -90.0))
        msg.roll = float(pose.get('roll', 0.0))
        msg.claw = float(claw_override) if claw_override is not None else float(pose.get('claw', -75.0))
        msg.time_ms = int(pose.get('time_ms', 1000))
        self.arm_pub.publish(msg)
        time.sleep(max(0.1, msg.time_ms / 1000.0))

    def move_named(self, name, claw_override=None):
        pose = self.points.get(name)
        if not isinstance(pose, dict):
            raise RuntimeError(f'missing arm_a point: {name}')
        self.publish_pose(pose, claw_override=claw_override)

    def move_release_lift(self, claw_override=None):
        pose = self.points.get('release_lift')
        if isinstance(pose, dict):
            self.publish_pose(pose, claw_override=claw_override)
            return
        release = self.points.get('release')
        if not isinstance(release, dict):
            raise RuntimeError('missing arm_a point: release')
        lift = dict(release)
        lift['z'] = float(release.get('z', 100.0)) + 100.0
        lift['time_ms'] = int(float(release.get('time_ms', 600))) + 200
        self.publish_pose(lift, claw_override=claw_override)

    def current_conveyor_profile(self):
        with self.lock:
            cmd = int(self.current_conveyor_cmd)
        if not self.conveyor_follow_enabled:
            return cmd, None
        return cmd, self.conveyor_speed_profiles.get(cmd)

    def clamped_follow_offset(self, profile, duration_key):
        duration_sec = max(0.0, float(profile.get(duration_key, 0)) / 1000.0)
        offset = float(profile['mmps']) * duration_sec
        limit = max(0.0, self.conveyor_max_offset_mm)
        return min(offset, limit) if limit > 0.0 else offset

    def with_follow_offset(self, pose, offset_mm, time_ms=None):
        adjusted = dict(pose)
        adjusted[self.conveyor_follow_axis] = float(adjusted.get(self.conveyor_follow_axis, 0.0)) + float(offset_mm)
        if time_ms is not None:
            adjusted['time_ms'] = int(time_ms)
        return adjusted

    def move_fixed_place_to_conveyor(self, hold_claw):
        self.move_named('place_approach', claw_override=hold_claw)
        self.move_named('place', claw_override=hold_claw)
        self.move_named('release')
        self.move_release_lift()

    def move_place_with_conveyor_follow(self, hold_claw):
        cmd, profile = self.current_conveyor_profile()
        if profile is None:
            self.move_fixed_place_to_conveyor(hold_claw)
            return

        approach = self.points.get('place_approach')
        place = self.points.get('place')
        release = self.points.get('release')
        release_lift = self.points.get('release_lift')
        if not isinstance(approach, dict) or not isinstance(place, dict) or not isinstance(release, dict):
            raise RuntimeError('missing arm_a conveyor place points')

        release_offset = self.clamped_follow_offset(profile, 'release_ms')
        tail_offset = self.clamped_follow_offset(profile, 'tail_ms')
        start_offset = -release_offset
        drop_offset = 0.0
        end_offset = tail_offset
        base_axis = float(place.get(self.conveyor_follow_axis, 0.0))
        if base_axis + start_offset < self.conveyor_min_axis_mm:
            axis_shift = self.conveyor_min_axis_mm - (base_axis + start_offset)
            start_offset += axis_shift
            drop_offset += axis_shift
            end_offset += axis_shift
        release_ms = profile.get('release_ms', place.get('time_ms', 600))
        tail_ms = profile.get('tail_ms', release.get('time_ms', 500))
        self.get_logger().info(
            f'arm A conveyor follow: cmd={cmd}, axis={self.conveyor_follow_axis}, '
            f'mmps={profile["mmps"]:.1f}, start_offset={start_offset:.1f}, '
            f'drop_offset={drop_offset:.1f}, end_offset={end_offset:.1f}'
        )

        self.publish_pose(self.with_follow_offset(approach, start_offset), claw_override=hold_claw)
        self.publish_pose(self.with_follow_offset(place, start_offset), claw_override=hold_claw)
        self.publish_pose(self.with_follow_offset(place, drop_offset, release_ms), claw_override=hold_claw)
        self.publish_pose(self.with_follow_offset(release, end_offset, tail_ms))
        if isinstance(release_lift, dict):
            self.publish_pose(self.with_follow_offset(release_lift, end_offset))
        else:
            lift = dict(release)
            lift['z'] = float(release.get('z', 100.0)) + 100.0
            lift['time_ms'] = int(float(release.get('time_ms', 600))) + 200
            self.publish_pose(self.with_follow_offset(lift, end_offset))

    def detect_color_targets(self, bgr_image, result_image):
        color_ranges = self.lab_data.get('color_range_list', {})
        if not self.roi_ready or not color_ranges:
            return []
        img_h, img_w = bgr_image.shape[:2]
        y0, y1, x0, x1 = [int(v) for v in self.roi]
        y0, y1 = max(0, y0), min(img_h, y1)
        x0, x1 = max(0, x0), min(img_w, x1)
        if y1 <= y0 or x1 <= x0:
            return []

        targets = []
        roi_img = bgr_image[y0:y1, x0:x1]
        roi_img = self.image_process.get_top_surface(roi_img)
        image_lab = cv2.cvtColor(cv2.GaussianBlur(roi_img, (3, 3), 3), cv2.COLOR_BGR2LAB)
        for color_key in COLOR_KEYS:
            if color_key not in color_ranges:
                continue
            index = 0
            color_range = color_ranges[color_key]
            mask = cv2.inRange(image_lab, tuple(color_range['min']), tuple(color_range['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)
            contours = map(lambda a_c: a_c[1], filter(lambda a: self.min_area <= a[0] <= self.max_area, contours_area))
            for contour in contours:
                area = math.fabs(cv2.contourArea(contour))
                rect = cv2.minAreaRect(contour)
                (center_x, center_y), _radius = cv2.minEnclosingCircle(contour)
                center_x, center_y = x0 + center_x, y0 + center_y
                rect_size = rect[1]
                corners = list(map(lambda p: (x0 + p[0], y0 + p[1]), cv2.boxPoints(rect)))
                cv2.drawContours(result_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                cv2.putText(result_image, color_key, (int(center_x) + 8, int(center_y)),
                            cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
                index += 1
                target_info = [
                    color_key,
                    index,
                    (int(center_x), int(center_y)),
                    (int(rect_size[0]), int(rect_size[1])),
                    int(round(rect[2])),
                ]
                targets.append({
                    'kind': 'color',
                    'label': color_key,
                    'center': target_info[2],
                    'target_info': target_info,
                    'score': area,
                })
        return targets

    def yolo_box_center(self, obj):
        box = obj['box']
        if len(box) < 4:
            return None
        x0, y0, x1_or_w, y1_or_h = [float(v) for v in box[:4]]
        if self.yolo_box_format in ('xyxy', 'corner_xyxy'):
            return (x0 + x1_or_w) / 2.0, (y0 + y1_or_h) / 2.0, max(1.0, x1_or_w - x0), max(1.0, y1_or_h - y0)
        return x0, y0, max(1.0, x1_or_w), max(1.0, y1_or_h)

    def detect_yolo_targets(self, result_image):
        with self.yolo_lock:
            if time.time() - self.yolo_stamp > self.yolo_max_age_sec:
                return []
            objects = list(self.yolo_objects)
        targets = []
        index = 0
        for obj in objects:
            parsed = self.yolo_box_center(obj)
            if parsed is None:
                continue
            center_x, center_y, box_w, box_h = parsed
            x1 = int(center_x - box_w / 2.0)
            y1 = int(center_y - box_h / 2.0)
            x2 = int(center_x + box_w / 2.0)
            y2 = int(center_y + box_h / 2.0)
            cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.circle(result_image, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            cv2.putText(result_image, 'garbage', (x1, max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 255, 0), 2)
            index += 1
            target_info = [
                obj['class_name'],
                index,
                (int(center_x), int(center_y)),
                (int(box_w), int(box_h)),
                int(round(obj['angle'])),
            ]
            targets.append({
                'kind': 'garbage',
                'label': obj['class_name'],
                'center': target_info[2],
                'target_info': target_info,
                'score': float(obj['score']),
            })
        return targets

    def object_height_for_target(self, target):
        if target.get('kind') == 'color':
            return self.color_object_height_m
        return self.garbage_object_height_m

    def get_object_world_position(self, pixel, height=None):
        if self.intrinsic is None or self.extristric is None or self.white_area_center is None:
            return None, None
        config_data = calibrated_pose.load_axis_calibration(APP_CONFIG_DIR, CALIBRATION_FILE)
        pick_height = self.garbage_object_height_m if height is None else float(height)
        return calibrated_pose.pixel_to_calibrated_world(
            pixel,
            self.intrinsic,
            self.extristric,
            self.white_area_center,
            config_data,
            height=pick_height,
        )

    def calculate_pick_grasp_yaw(self, position, target_info, all_target_info, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        gripper_size = [
            common.calculate_pixel_length(0.09, self.intrinsic, projection_matrix),
            common.calculate_pixel_length(0.015, self.intrinsic, projection_matrix),
        ]
        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target_info, all_target_info, gripper_size, yaw)

    def resolve_app_targets(self, targets):
        if not targets:
            self.last_object_info_list = None
            return []
        all_target_info = [copy.deepcopy(target['target_info']) for target in targets]
        if self.last_object_info_list:
            all_target_info = position_reorder_compatible(all_target_info, self.last_object_info_list, 20)
        self.last_object_info_list = copy.deepcopy(all_target_info)
        target_by_identity = {
            (target['target_info'][0], target['target_info'][1]): target
            for target in targets
        }
        resolved = []
        for target_info in all_target_info:
            target = target_by_identity.get((target_info[0], target_info[1]))
            if target is None:
                continue
            if self.camera_type == 'usb_cam':
                x, y = distortion_inverse_map.undistorted_to_distorted_pixel(
                    target_info[2][0],
                    target_info[2][1],
                    self.intrinsic,
                    self.distortion,
                )
                target_info[2] = (x, y)
            height = self.object_height_for_target(target)
            position, projection_matrix = self.get_object_world_position(target_info[2], height=height)
            if position is None or projection_matrix is None:
                continue
            yaw_result = self.calculate_pick_grasp_yaw(position, target_info, all_target_info, projection_matrix)
            if yaw_result is None:
                continue
            target = dict(target)
            target['target_info'] = target_info
            target['center'] = target_info[2]
            target['world_position'] = position
            target['yaw'] = utils.normalize_gripper_roll_deg(yaw_result[0])
            target['grasp_line'] = yaw_result[1]
            cv2_line = target.get('draw_line')
            if cv2_line is not None:
                pass
            resolved.append(target)
        return resolved

    def select_scene5_target(self, targets):
        if not targets:
            return None
        return min(targets, key=lambda target: abs(float(target['center'][0]) - self.target_center_x))

    def target_is_stable(self, target):
        if target is None:
            self.stable_count = 0
            self.last_target = None
            return False
        identity = f"{target['kind']}:{target['label']}:{target['target_info'][1]}"
        position = target.get('world_position')
        if position is None:
            self.stable_count = 0
            self.last_target = None
            return False
        if self.last_target is not None:
            last_position = self.last_target['position']
            same_target = self.last_target['identity'] == identity
            e_distance = round(
                math.sqrt(pow(float(last_position[0]) - float(position[0]), 2)) +
                math.sqrt(pow(float(last_position[1]) - float(position[1]), 2)),
                5,
            )
            self.stable_count = self.stable_count + 1 if same_target and e_distance <= self.world_stable_distance_m else 0
        else:
            self.stable_count = 0
        self.last_target = {'identity': identity, 'position': list(position)}
        return self.stable_count >= self.stable_required

    def pick_and_place_to_conveyor(self, target):
        try:
            pick_and_place.interrupt(False)
            position = list(target['world_position'])
            if position[0] > 0.22:
                position[2] += 0.01
            is_color = target.get('kind') == 'color'
            pick_z_offset_m = COLOR_PICK_Z_OFFSET_M if is_color else GARBAGE_PICK_Z_OFFSET_M
            position[2] -= pick_z_offset_m
            config_data = calibrated_pose.load_axis_calibration(APP_CONFIG_DIR, CALIBRATION_FILE)
            position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics').tolist()
            target_claw = COLOR_CLAW_GRAB_ANGLE if is_color else GARBAGE_CLAW_GRAB_ANGLE
            pick_kwargs = {'claw_grab_angle': target_claw}
            hold_claw = target_claw
            finish = pick_and_place.pick(position, 90, target['yaw'], 540, 0.02, self.arm_pub, **pick_kwargs)
            if finish:
                self.move_place_with_conveyor_follow(hold_claw)
                self.move_named('home')
            else:
                self.move_named('home')
        except Exception as ex:
            self.get_logger().error(str(ex))
        finally:
            with self.lock:
                self.moving = False
                if self.single_cycle:
                    self.running = False
                    self.single_cycle = False
                    self.single_done.set()

    def publish_result_image(self, result_image):
        try:
            msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.result_image_pub.publish(msg)
        except Exception as ex:
            self.get_logger().warn(f'publish arm A result image failed: {ex}')

    def vision_loop(self):
        while rclpy.ok():
            try:
                bgr_image = self.image_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            result_image = np.copy(bgr_image)
            color_targets = self.detect_color_targets(bgr_image, result_image)
            garbage_targets = self.detect_yolo_targets(result_image)
            raw_targets = color_targets + garbage_targets
            resolved_targets = self.resolve_app_targets(raw_targets)
            target = self.select_scene5_target(resolved_targets)
            if target is not None and target.get('grasp_line') is not None:
                cv2.line(result_image, target['grasp_line'][0], target['grasp_line'][1], (255, 255, 0), 2, cv2.LINE_AA)
            with self.lock:
                can_move = self.running and not self.moving
            if can_move and self.target_is_stable(target):
                with self.lock:
                    if not self.running or self.moving:
                        self.publish_result_image(result_image)
                        continue
                    self.moving = True
                threading.Thread(target=self.pick_and_place_to_conveyor, args=(target,), daemon=True).start()
            elif target is None:
                self.target_is_stable(None)
            self.publish_result_image(result_image)

    def call_yolo(self, client, label):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{label} service unavailable: {client.srv_name}')
            return
        client.call_async(Trigger.Request())

    def prepare_scene_runtime(self, timeout_sec=40.0):
        label = self.scene_runtime_prepare_client.srv_name
        if not self.scene_runtime_prepare_client.wait_for_service(timeout_sec=2.0):
            return False, f'{label} service unavailable'
        future = self.scene_runtime_prepare_client.call_async(Trigger.Request())
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                return False, f'{label} timed out'
            time.sleep(0.02)
        try:
            result = future.result()
        except Exception as exc:
            return False, f'{label} failed: {exc}'
        if result is None or not getattr(result, 'success', False):
            msg = getattr(result, 'message', 'no response') if result is not None else 'no response'
            return False, f'{label} failed: {msg}'
        return True, getattr(result, 'message', 'prepared')

    def prepare_and_home(self):
        ok, msg = self.prepare_scene_runtime()
        if not ok:
            return False, msg
        self.move_named('home')
        return True, msg

    def on_home(self, request, response):
        try:
            ok, msg = self.prepare_and_home()
            if not ok:
                response.success = False
                response.message = msg
                return response
            response.success = True
            response.message = 'arm_a home done'
        except Exception as ex:
            response.success = False
            response.message = str(ex)
        return response

    def on_load_once(self, request, response):
        with self.lock:
            if self.running:
                response.success = False
                response.message = 'arm_a already running'
                return response
        try:
            ok, msg = self.prepare_and_home()
            if not ok:
                response.success = False
                response.message = msg
                return response
        except Exception as ex:
            response.success = False
            response.message = str(ex)
            return response
        with self.lock:
            self.running = True
            self.single_cycle = True
            self.single_done.clear()
        self.call_yolo(self.yolo_start_client, 'arm_a yolo start')
        pick_and_place.interrupt(False)
        if self.single_done.wait(timeout=60.0):
            response.success = True
            response.message = 'arm_a app-style pick and conveyor place once done'
        else:
            with self.lock:
                self.running = False
                self.single_cycle = False
                self.moving = False
            pick_and_place.interrupt(True)
            response.success = False
            response.message = 'arm_a app-style target timeout'
        return response

    def on_start(self, request, response):
        with self.lock:
            if self.running:
                response.success = True
                response.message = 'arm_a already running'
                return response
        try:
            ok, msg = self.prepare_and_home()
            if not ok:
                response.success = False
                response.message = msg
                return response
        except Exception as ex:
            response.success = False
            response.message = str(ex)
            return response
        with self.lock:
            self.running = True
            self.single_cycle = False
        self.call_yolo(self.yolo_start_client, 'arm_a yolo start')
        pick_and_place.interrupt(False)
        response.success = True
        response.message = 'arm_a app-style pipeline started'
        return response

    def on_stop(self, request, response):
        with self.lock:
            self.running = False
            self.single_cycle = False
        pick_and_place.interrupt(True)
        self.call_yolo(self.yolo_stop_client, 'arm_a yolo stop')
        response.success = True
        response.message = 'arm_a stop acknowledged'
        return response


def position_reorder_compatible(current_points, last_points, distance):
    try:
        from app.utils import position_change_detect
        return position_change_detect.position_reorder(current_points, last_points, distance)
    except Exception:
        return current_points


def main():
    rclpy.init()
    node = Scene5ArmALoader()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
