#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import math
import time
import yaml
import queue
import threading
import numpy as np

import rclpy
import message_filters
from rclpy.node import Node
from cv_bridge import CvBridge
import json
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener, TransformException

import sdk.fps as fps
from sdk import common
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState

INIT_X = 200.0
INIT_Y = 0.0
INIT_Z = 200.0
INIT_PITCH = -90.0
INIT_ROLL = 0.0
INIT_CLAW = 0.0
GRAB_CLAW = 30.0     # 完全闭合 (30°, 0mm)
OPEN_CLAW = -60.0    # 完全张开 (-60°, 51mm)
DEFAULT_MAP_LENGTH_M = 0.13
DEFAULT_MAP_WIDTH_M = 0.167
DEFAULT_HOME_POSE = {
    'x': INIT_X,
    'y': INIT_Y,
    'z': INIT_Z,
    'pitch': INIT_PITCH,
    'roll': INIT_ROLL,
    'claw': INIT_CLAW,
    'time_ms': 2000,
}
SCENE5_ID = 'scene_5'
DEFAULT_CONTROLLER_PREFIX = '/ros_robot_controller'
SCENE5_ARM_PREFIXES = {
    'A': '/arm_a/ros_robot_controller',
    'B': '/arm_b/ros_robot_controller',
}


def normalize_scene5_arm_role(role):
    role = str(role or 'A').strip().upper()
    return role if role in SCENE5_ARM_PREFIXES else 'A'


def scene5_controller_prefix(scene_id=None):
    if scene_id != SCENE5_ID:
        return DEFAULT_CONTROLLER_PREFIX
    return SCENE5_ARM_PREFIXES[normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE'))]


def scene5_calibration_controller_prefix(scene_id=None):
    if scene_id != SCENE5_ID:
        return DEFAULT_CONTROLLER_PREFIX
    return SCENE5_ARM_PREFIXES['A']


def scene5_calibration_allowed(scene_id=None):
    return scene_id != SCENE5_ID or normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE')) == 'A'


class ColorPicker:
    def __init__(self, point, repeat):
        self.point = point
        self.count = 0
        self.color = []
        self.rgb = []
        self.repeat = repeat

    def set_point(self, point):
        self.point = point

    def reset(self):
        self.count = 0
        self.color = []
        self.rgb = []

    def __call__(self, image, result_image):
        h, w = image.shape[:2]
        x, y = int(self.point[0]), int(self.point[1])
        if y == 0: y = 1
        if y == h: y = h - 1
        if x == 0: x = 1
        if x == w: x = w - 1
        image_lab = cv2.cvtColor(image[y - 1:y + 1, x - 1:x + 1], cv2.COLOR_BGR2LAB)
        self.color.extend(image_lab.tolist())
        self.rgb.extend(image[y - 1:y + 1, x - 1:x + 1].tolist())
        self.count += 1
        l, a, b = 0, 0, 0
        r, g, b_ = 0, 0, 0
        for c in self.color:
            l, a, b = l + c[0][0] + c[1][0], a + c[0][1] + c[1][1], b + c[0][2] + c[1][2]
        for c in self.rgb:
            r, g, b_ = r + c[0][0] + c[1][0], g + c[0][1] + c[1][1], b_ + c[0][2] + c[1][2]
        l = int(l / (2 * len(self.color)))
        a = int(a / (2 * len(self.color)))
        b = int(b / (2 * len(self.color)))
        r = int(r / (2 * len(self.rgb)))
        g = int(g / (2 * len(self.rgb)))
        b_ = int(b_ / (2 * len(self.rgb)))
        if 0 <= x < w and 0 <= y < h:
            result_image = cv2.circle(result_image, (x, y), self.count, (r, g, b_), 2 * self.count)
            result_image = cv2.circle(result_image, (x, y), self.count, (255, 255, 0), 5)
        if len(self.color) / 2 > self.repeat:
            self.color.remove(self.color[0])
            self.color.remove(self.color[0])
        if len(self.rgb) / 2 > self.repeat:
            self.rgb.remove(self.rgb[0])
            self.rgb.remove(self.rgb[0])
        if self.count > self.repeat:
            self.count = self.repeat
        if self.count >= self.repeat:
            return ((l, a, b), (r, g, b_)), result_image
        else:
            return None, result_image


class ColorPick(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        self.fps = fps.FPS()
        self.rgb_image_queue = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_type = os.environ.get('CAMERA_TYPE', 'usb_cam')
        self.verbose_logs = os.environ.get('CALIB_VERBOSE_LOGS', '0') == '1'
        self.pick_debug = os.environ.get('CALIB_PICK_DEBUG', '0') == '1'
        self.depth_debug = os.environ.get('CALIB_DEPTH_DEBUG', '0') == '1'
        self.depth_verbose = os.environ.get('CALIB_DEPTH_VERBOSE', '0') == '1'
        self.pixel_debug = os.environ.get('CALIB_PIXEL_DEBUG', '0') == '1'
        self._last_pick_debug_ts = 0.0
        self.running = True
        self.center = []
        self.target_color = None
        self.set_callback = False
        self.color_picker = None
        self.intrinsic = None
        self.distortion = None
        self.transport_info = None
        self.count_move = 0
        self.count_still = 0
        self.mode = 'color'
        self.calibration = False
        self.depth_enable = False
        self.last_position = None
        self.start_transport = False
        self.grab_calib_mode = False
        self.last_depth_target_px = None
        self.last_depth_query_px = None
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.scene_config_file = 'calibration_scene.yaml'
        self.lab_file = '/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml'
        self.camera_info_file = '/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml'
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        if self.chassis_type == 'Slide_Rails':
            self.config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/"
        else:
            self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.controller_prefix = self._controller_prefix_from_scene()
        self.arm_pub = self.create_publisher(ArmCoords, self.ctl_topic('arm/set_coords'), 5)
        _disp_ns = self._camera_namespace()
        _disp_topic = f'{_disp_ns}/calibration/display_image' if _disp_ns else '/calibration/display_image'
        _depth_topic = f'{_disp_ns}/calibration/depth_image' if _disp_ns else '/calibration/depth_image'
        self.display_pub = self.create_publisher(Image, _disp_topic, 1)
        self.depth_display_pub = self.create_publisher(Image, _depth_topic, 1)
        _click_topic = f'{_disp_ns}/calibration/ui_click' if _disp_ns else '/calibration/ui_click'
        self.create_subscription(String, _click_topic, self._on_ui_click, 1)

        self.create_service(SetBool, 'calibration/start_calibration', self.start_calibration_srv_callback)
        self.create_service(Trigger, 'calibration/grab_calibration', self.grab_calibration_srv_callback)
        self.create_service(Trigger, 'calibration/clear_grab_calibration', self.clear_grab_calibration_srv_callback)

        self.home_pose = self._load_home_pose_from_scene()

        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            self.plane = config['plane']
            self.corners = np.array(config['corners'])
            self.extristric = np.array(config['extristric'])
            self.white_area_center = self._normalize_white_area_pose_world(config['white_area_pose_world'])
        self.white_area_length_m, self.white_area_width_m = self._load_scene_dimensions()

        self.hand2cam_tf_matrix_color = np.eye(4)
        self.hand2cam_tf_matrix_depth = np.eye(4)
        self.hand2cam_file_mtime = None
        self.lab_file_mtime = None
        self.lab_color_ranges = {}

        cam_ns = self._camera_namespace()
        self.rgb_sub = message_filters.Subscriber(self, Image, f'{cam_ns}/depth_cam/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, f'{cam_ns}/depth_cam/rgb/camera_info')
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.info_sub], 3, 0.2)
        self.sync.registerCallback(self.rgb_callback)

        if self.camera_type == 'aurora':
            self.depth_sub = message_filters.Subscriber(self, Image, f'{cam_ns}/depth_cam/depth/image_raw')
            self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, f'{cam_ns}/depth_cam/depth/camera_info')
            self.sync_depth = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.depth_info_sub], 3, 0.2)
            self.sync_depth.registerCallback(self.depth_callback)

        self.create_subscription(Image, '/calibration/image_result', self.image_callback, 1)
        self.create_subscription(Bool, '/calibration/finish', self.finish_calibration_callback, 1)
        self.create_subscription(ArmFullState, self.ctl_topic('arm/full_state'), self.arm_state_callback, 5)
        self.current_pose = None
        self.current_pose_ts = 0.0
        self.known_pose = dict(self.home_pose)
        self.known_pose['yaw'] = 0.0

        tf_buffer = Buffer()
        self.color_to_depth_transform_matrix = None
        if self.camera_type == 'aurora':
            self.tf_listener = TransformListener(tf_buffer, self)
            try:
                tf_future = tf_buffer.wait_for_transform_async(
                    target_frame='depth_cam_depth_optical_frame',
                    source_frame='depth_cam_color_frame',
                    time=rclpy.time.Time()
                )
                rclpy.spin_until_future_complete(self, tf_future, timeout_sec=5.0)
                transform = tf_buffer.lookup_transform(
                    'depth_cam_depth_optical_frame', 'depth_cam_color_frame',
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.color_to_depth_transform_matrix = common.xyz_quat_to_mat(
                    [translation.x, translation.y, translation.z],
                    [rotation.w, rotation.x, rotation.y, rotation.z])
            except Exception as e:
                self.get_logger().warn(f'获取深度相机静态变换失败(可忽略): {e}')
        else:
            self.tf_listener = TransformListener(tf_buffer, self)
            try:
                tf_future = tf_buffer.wait_for_transform_async(
                    target_frame='depth_camera_link',
                    source_frame='depth_cam_color_frame',
                    time=rclpy.time.Time()
                )
                rclpy.spin_until_future_complete(self, tf_future, timeout_sec=5.0)
                transform = tf_buffer.lookup_transform(
                    'depth_camera_link', 'depth_cam_color_frame',
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.color_to_depth_transform_matrix = common.xyz_quat_to_mat(
                    [translation.x, translation.y, translation.z],
                    [rotation.w, rotation.x, rotation.y, rotation.z])
            except Exception as e:
                self.get_logger().warn(f'获取相机静态变换失败(可忽略): {e}')

        self._load_hand2cam_from_file(force=True)
        # self.get_logger().info('[PATCH] endpoint_matrix uses roll,-pitch,yaw from ArmFullState')

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def arm_state_callback(self, msg):
        self.current_pose = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'pitch': float(msg.pitch),
            'roll': float(msg.roll),
            'claw': float(msg.claw),
            'yaw': float(msg.yaw),
            'joint_angles': [float(v) for v in msg.joint_angles],
        }
        self.current_pose_ts = time.time()

    def _active_scene_id(self):
        env_scene = (
            os.environ.get('CALIBRATION_CURRENT_SCENE')
            or os.environ.get('CALIBRATION_DEFAULT_SCENE')
            or os.environ.get('SCENE')
        )
        if env_scene:
            return env_scene
        scene_path = os.path.join(self.config_path, self.scene_config_file)
        try:
            with open(scene_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            return str(cfg.get('current_scene', 'scene_0'))
        except Exception:
            return 'scene_0'

    def _controller_prefix_from_scene(self):
        return scene5_calibration_controller_prefix(self._active_scene_id())

    def _camera_namespace(self):
        if self._active_scene_id() == SCENE5_ID:
            role = normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE'))
            return f'/arm_{role.lower()}'
        return ''

    def ctl_topic(self, suffix):
        return f'{self.controller_prefix.rstrip("/")}/{suffix.lstrip("/")}'

    def _normalize_white_area_pose_world(self, pose_matrix):
        return np.array(pose_matrix, dtype=np.float64).reshape(4, 4)

    def _load_scene_dimensions(self):
        scene_path = os.path.join(self.config_path, self.scene_config_file)
        try:
            with open(scene_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            scenes = cfg.get('scenes', {}) if isinstance(cfg, dict) else {}
            if isinstance(scenes, dict):
                scenes.setdefault('scene_0', {
                    'name': 'Scene 0',
                    'length_m': 0.13,
                    'width_m': 0.167,
                    'calibration_tag': {
                        'id': 1,
                        'size_m': 0.04,
                        'effective_size_m': 0.033,
                        'yaw_deg': 0.0,
                        'center_in_map_m': {'x': -0.045, 'y': 0.0635, 'z': 0.0},
                    },
                    'home_pose': dict(DEFAULT_HOME_POSE),
                })
            scene_id = (
                os.environ.get('CALIBRATION_CURRENT_SCENE')
                or os.environ.get('CALIBRATION_DEFAULT_SCENE')
                or os.environ.get('SCENE')
                or (cfg.get('current_scene', 'scene_0') if isinstance(cfg, dict) else 'scene_0')
            )
            if isinstance(scenes, dict) and scene_id not in scenes and scenes:
                scene_id = next(iter(scenes.keys()))
            scene = scenes.get(scene_id, {}) if isinstance(scenes, dict) else {}
            calibration_scene_id = scene.get('use_calibration_scene', scene_id) if isinstance(scene, dict) else scene_id
            if isinstance(scenes, dict) and calibration_scene_id in scenes:
                scene = scenes.get(calibration_scene_id, {})
            length_m = float(scene.get('length_m', DEFAULT_MAP_LENGTH_M))
            width_m = float(scene.get('width_m', DEFAULT_MAP_WIDTH_M))
            return length_m, width_m
        except Exception as exc:
            self.get_logger().warn(f'读取场景尺寸失败，使用默认地图尺寸: {exc}')
            return DEFAULT_MAP_LENGTH_M, DEFAULT_MAP_WIDTH_M

    def _load_home_pose_from_scene(self):
        home_pose = dict(DEFAULT_HOME_POSE)
        scene_path = os.path.join(self.config_path, self.scene_config_file)
        try:
            with open(scene_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            scenes = cfg.get('scenes', {}) if isinstance(cfg, dict) else {}
            if isinstance(scenes, dict):
                scenes.setdefault('scene_0', {
                    'name': 'Scene 0',
                    'length_m': 0.13,
                    'width_m': 0.167,
                    'calibration_tag': {
                        'id': 1,
                        'size_m': 0.04,
                        'effective_size_m': 0.033,
                        'yaw_deg': 0.0,
                        'center_in_map_m': {'x': -0.045, 'y': 0.0635, 'z': 0.0},
                    },
                    'home_pose': dict(DEFAULT_HOME_POSE),
                })
            scene_id = (
                os.environ.get('CALIBRATION_CURRENT_SCENE')
                or os.environ.get('CALIBRATION_DEFAULT_SCENE')
                or os.environ.get('SCENE')
                or (cfg.get('current_scene', 'scene_0') if isinstance(cfg, dict) else 'scene_0')
            )
            if isinstance(scenes, dict) and scene_id not in scenes and scenes:
                scene_id = next(iter(scenes.keys()))
            scene = scenes.get(scene_id, {}) if isinstance(scenes, dict) else {}
            if scene_id == SCENE5_ID and isinstance(scene.get('calibration_pose'), dict):
                home = scene.get('calibration_pose', {})
            else:
                home = scene.get('home_pose', {}) if isinstance(scene.get('home_pose'), dict) else {}
            for key, default_value in DEFAULT_HOME_POSE.items():
                if key == 'time_ms':
                    home_pose[key] = int(float(home.get(key, default_value)))
                else:
                    home_pose[key] = float(home.get(key, default_value))
        except Exception as exc:
            self.get_logger().warn(f'读取场景默认位姿失败，使用默认位姿: {exc}')
        return home_pose

    def _apply_loaded_hand2cam(self, hand2cam_matrix):
        self.hand2cam_tf_matrix_color = np.array(hand2cam_matrix, dtype=np.float64).reshape(4, 4)
        if self.color_to_depth_transform_matrix is not None:
            # Match /home/ubuntu/factory_utils/calibration depth path:
            # transform_matrix @ hand2cam_tf_matrix.
            self.hand2cam_tf_matrix_depth = np.matmul(self.color_to_depth_transform_matrix, self.hand2cam_tf_matrix_color)
        else:
            self.hand2cam_tf_matrix_depth = np.array(self.hand2cam_tf_matrix_color)
        # if self.verbose_logs:
        #     self.get_logger().info(f'hand2cam_tf_matrix_color: {self.hand2cam_tf_matrix_color}')
        #     self.get_logger().info(f'hand2cam_tf_matrix_depth: {self.hand2cam_tf_matrix_depth}')

    def _load_hand2cam_from_file(self, force=False):
        try:
            mtime = os.path.getmtime(self.camera_info_file)
        except OSError as exc:
            if force:
                self.get_logger().error(f'读取 hand2cam 配置失败: {exc}')
            return False

        if (not force and self.hand2cam_file_mtime is not None and mtime <= self.hand2cam_file_mtime):
            return False

        try:
            with open(self.camera_info_file, 'r', encoding='utf-8') as f:
                cam_config = yaml.safe_load(f) or {}
            matrix = cam_config.get('hand2cam_tf_matrix')
            if matrix is None:
                raise KeyError('camera_info.yaml 中缺少 hand2cam_tf_matrix')
            self._apply_loaded_hand2cam(matrix)
            self.hand2cam_file_mtime = mtime
            return True
        except Exception as exc:
            self.get_logger().error(f'加载 hand2cam_tf_matrix 失败: {exc}')
            return False

    def _load_lab_color_ranges(self, force=False):
        try:
            mtime = os.path.getmtime(self.lab_file)
        except OSError:
            self.lab_color_ranges = {}
            self.lab_file_mtime = None
            return {}
        if not force and self.lab_file_mtime is not None and mtime <= self.lab_file_mtime:
            return self.lab_color_ranges
        try:
            with open(self.lab_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            params = data.get('/**', {}).get('ros__parameters', {})
            ranges = params.get('color_range_list', {})
            self.lab_color_ranges = ranges if isinstance(ranges, dict) else {}
            self.lab_file_mtime = mtime
        except Exception:
            self.lab_color_ranges = {}
            self.lab_file_mtime = None
        return self.lab_color_ranges

    def _select_lab_color_range(self, lab_value):
        ranges = self._load_lab_color_ranges()
        if not ranges:
            return None, None, None
        lab = np.asarray(lab_value, dtype=np.float64)
        candidates = []
        for name, cfg in ranges.items():
            try:
                min_color = np.asarray(cfg['min'], dtype=np.int32)
                max_color = np.asarray(cfg['max'], dtype=np.int32)
            except Exception:
                continue
            if min_color.shape != (3,) or max_color.shape != (3,):
                continue
            if np.all(lab >= min_color) and np.all(lab <= max_color):
                center = (min_color.astype(np.float64) + max_color.astype(np.float64)) * 0.5
                candidates.append((float(np.linalg.norm(lab - center)), name, min_color.tolist(), max_color.tolist()))
        if not candidates:
            return None, None, None
        _, name, min_color, max_color = min(candidates, key=lambda item: item[0])
        return name, min_color, max_color

    def _vec_text(self, values, precision=4):
        return '(' + ', '.join(f'{float(v):.{precision}f}' for v in values) + ')'

    def _format_pose(self, pose):
        if pose is None:
            return 'None'
        joint_angles = pose.get('joint_angles')
        joint_text = ''
        if joint_angles:
            joint_text = f", joints={[round(float(v), 2) for v in joint_angles[:6]]}"
        return (
            f"x={float(pose.get('x', 0.0)):.2f}, y={float(pose.get('y', 0.0)):.2f}, z={float(pose.get('z', 0.0)):.2f}, "
            f"pitch={float(pose.get('pitch', 0.0)):.2f}, roll={float(pose.get('roll', 0.0)):.2f}, "
            f"claw={float(pose.get('claw', 0.0)):.2f}, yaw={float(pose.get('yaw', 0.0)):.2f}{joint_text}"
        )

    def _matrix_summary(self, matrix):
        try:
            matrix_np = np.array(matrix, dtype=np.float64)
            translation, euler = common.mat_to_xyz_euler(matrix_np, degrees=True)
            return (
                f"t={self._vec_text(translation)}, rpy={self._vec_text(euler, precision=2)}, "
                f"row0={np.array2string(matrix_np[0], precision=4)}, "
                f"row1={np.array2string(matrix_np[1], precision=4)}, "
                f"row2={np.array2string(matrix_np[2], precision=4)}"
            )
        except Exception as exc:
            return f"matrix_summary_failed: {exc}, raw={np.array2string(np.array(matrix), precision=4)}"

    def _extristric_to_matrix(self, extristric):
        matrix = np.eye(4)
        extristric_np = np.array(extristric, dtype=np.float64)
        if extristric_np.shape == (4, 3):
            matrix[:3, 3] = extristric_np[0]
            matrix[:3, :3] = extristric_np[1:]
        return matrix

    def _plane_depth_at_pixel(self, pixel_x, pixel_y, plane, intrinsic_matrix):
        a, b, c, d = [float(v) for v in plane]
        fx, fy = float(intrinsic_matrix[0]), float(intrinsic_matrix[4])
        cx, cy = float(intrinsic_matrix[2]), float(intrinsic_matrix[5])
        denom = a * (float(pixel_x) - cx) / fx + b * (float(pixel_y) - cy) / fy + c
        if abs(denom) < 1e-9:
            return float('nan')
        return -d / denom

    def _pose_delta_summary(self, current_pose, known_pose):
        if current_pose is None or known_pose is None:
            return 'current_pose_or_known_pose_missing'
        keys = ['x', 'y', 'z', 'pitch', 'roll', 'claw', 'yaw']
        return ', '.join(
            f"{key}={float(current_pose.get(key, 0.0)) - float(known_pose.get(key, 0.0)):.2f}"
            for key in keys
        )

    def _log_pick_debug(self, msg, min_interval=0.3, key='pick'):
        if not self.pick_debug:
            return
        if key.startswith('depth'):
            if not self.depth_debug:
                return
        elif key.startswith('pixel'):
            if not self.pixel_debug:
                return
        now = time.time()
        attr = f'_last_{key}_debug_ts'
        last = getattr(self, attr, 0.0)
        if now - last < min_interval:
            return
        setattr(self, attr, now)
        self.get_logger().warn(msg)

    def _should_log_debug(self, name, min_interval=0.8):
        now = time.time()
        attr_name = f'_last_debug_{name}_ts'
        last_time = getattr(self, attr_name, 0.0)
        if now - last_time < min_interval:
            return False
        setattr(self, attr_name, now)
        return True

    def calculate_pick_grasp_roll(self, position, angle=0.0):
        base_yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            base_yaw += 180.0
        elif position[0] < 0 and position[1] > 0:
            base_yaw -= 180.0

        yaw1 = base_yaw + float(angle)
        if yaw1 < 0:
            yaw2 = yaw1 + 90.0
        else:
            yaw2 = yaw1 - 90.0
        grasp_yaw = yaw1 if abs(yaw1) < abs(yaw2) else yaw2

        roll_deg = float(np.clip(grasp_yaw, -120.0, 120.0))
        return roll_deg, grasp_yaw, base_yaw

    def get_endpoint_matrix(self):
        """使用与手眼求解一致的末端姿态约定构建 4x4 齐次变换矩阵。"""
        p = self.current_pose if self.current_pose is not None else self.known_pose
        x_m = p['x'] / 1000.0
        y_m = p['y'] / 1000.0
        z_m = p['z'] / 1000.0
        yaw_deg = p.get('yaw')
        if yaw_deg is None:
            yaw_deg = math.degrees(math.atan2(p['y'], p['x'])) if p['x'] or p['y'] else 0.0
        endpoint = common.xyz_euler_to_mat(
            [x_m, y_m, z_m],
            [float(p.get('roll', 0.0)), -float(p.get('pitch', 0.0)), float(yaw_deg)],
            degrees=True,
        )
        return endpoint

    def _get_fresh_endpoint_matrix(self, max_age_sec=0.6):
        if self.current_pose is None:
            return None, None, 'missing_current_pose'
        age = time.time() - float(self.current_pose_ts or 0.0)
        if age > max_age_sec:
            return None, None, f'stale_current_pose(age={age:.3f}s)'
        endpoint = self.get_endpoint_matrix()
        return endpoint, age, None

    def init_process(self):
        self.timer.cancel()
        self.go_home()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.pick_and_place_thread, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % '初始化完成')

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
        yaw_deg = math.degrees(math.atan2(float(y), float(x))) if (float(x) != 0.0 or float(y) != 0.0) else 0.0
        self.known_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'pitch': float(pitch),
            'roll': float(roll),
            'claw': float(claw),
            'yaw': yaw_deg,
        }
        self.get_logger().info(
            f'[ARM_CMD] send={self._format_pose(self.known_pose)}, time_ms={int(time_ms)}, '
            f'latest_real={self._format_pose(self.current_pose)}, real-minus-cmd=({self._pose_delta_summary(self.current_pose, self.known_pose)})'
        )

    def go_home(self, claw=None):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        target_claw = hp['claw'] if claw is None else float(claw)
        wait_start = time.time()
        while self.arm_pub.get_subscription_count() == 0:
            if time.time() - wait_start > 5.0:
                self.get_logger().error('等待 ros_robot_controller 订阅超时，请确认底层控制节点已启动')
                return False
            self.get_logger().info('等待 ros_robot_controller 订阅...')
            time.sleep(0.5)
        time_ms = int(hp.get('time_ms', 1500))
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], target_claw, time_ms)
        time.sleep(max(0.0, time_ms / 1000.0) + 0.3)
        return True

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                raw_y = y + 40 if self.camera_type == 'usb_cam' else y
                if x > 640:
                    self.mode = 'depth'
                    self.center = [x - 640, raw_y]
                else:
                    self.mode = 'color'
                    self.color_picker = ColorPicker([x, raw_y], 20)
                    self.center = [x, raw_y]
                    self.target_color = None
                self._log_pick_debug(
                    f'[CLICK_DEBUG] display_px=({x},{y}), raw_px=({self.center[0]},{self.center[1]}), '
                    f'mode={self.mode}, camera_type={self.camera_type}, crop_y_offset={40 if self.camera_type == "usb_cam" else 0}',
                    min_interval=0.0,
                    key='click',
                )

    def _on_ui_click(self, msg):
        try:
            data = json.loads(msg.data)
            mode = str(data.get('mode', 'color'))
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            with self.lock:
                raw_y = y + 40 if self.camera_type == 'usb_cam' else y
                if mode == 'depth':
                    self.mode = 'depth'
                    self.center = [x, raw_y]
                else:
                    self.mode = 'color'
                    self.color_picker = ColorPicker([x, raw_y], 20)
                    self.center = [x, raw_y]
                    self.target_color = None
        except Exception:
            pass

    def start_calibration_srv_callback(self, request, response):
        if not scene5_calibration_allowed(self._active_scene_id()):
            response.success = False
            response.message = "scene_5 calibration is only allowed on arm A"
            return response
        with self.lock:
            self.calibration = request.data
        response.success = True
        response.message = "开始"
        return response

    def grab_calibration_srv_callback(self, request, response):
        with self.lock:
            self.grab_calib_mode = True
            self.start_transport = False
        response.success = True
        response.message = "grab_calibration"
        return response

    def clear_grab_calibration_srv_callback(self, request, response):
        with self.lock:
            self.grab_calib_mode = False
        response.success = True
        response.message = "clear_grab_calibration"
        return response


    def get_object_world_position(self, position, intrinsic, height_m=0.03):
        tvec = self.extristric[:1]
        rmat = self.extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), height_m)
        projection_matrix = np.row_stack((np.column_stack((rmat, tvec)), np.array([[0, 0, 0, 1]])))
        world_pose = np.asarray(common.pixels_to_world([position], intrinsic, projection_matrix)[0], dtype=np.float64)
        local_point = np.ones(4, dtype=np.float64)
        local_point[:3] = world_pose
        position_w = np.matmul(self.white_area_center, local_point)[:3]
        position_w[2] = height_m

        raw_position_w = np.array(position_w, dtype=np.float64)

        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['pixel']['offset'])
        scale = tuple(config_data['pixel']['scale'])
        for i in range(3):
            position_w[i] = position_w[i] * scale[i]
            position_w[i] = position_w[i] + offset[i]
        self._log_pick_debug(
            f'[PIXEL_WORLD_DEBUG] pixel=({float(position[0]):.1f},{float(position[1]):.1f}), '
            f'mapping=pixels_to_world_full_pose, local_map={self._vec_text(world_pose)}, raw_world={self._vec_text(raw_position_w)}, '
            f'scaled_world={self._vec_text(position_w)}, pixel_scale={scale}, pixel_offset={offset}, '
            f'map_size=({self.white_area_length_m:.4f},{self.white_area_width_m:.4f}), '
            f'white_area={self._matrix_summary(self.white_area_center)}',
            min_interval=0.0,
            key='pixel_world',
        )
        return position_w, projection_matrix

    def get_depth_world_position_from_calibration(self, position, intrinsic, height_m):
        tvec = self.extristric[:1]
        rmat = self.extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), height_m)
        projection_matrix = np.row_stack((np.column_stack((rmat, tvec)), np.array([[0, 0, 0, 1]])))
        local_map = np.asarray(common.pixels_to_world([position], intrinsic, projection_matrix)[0], dtype=np.float64)
        local_point = np.ones(4, dtype=np.float64)
        local_point[:3] = local_map
        position_w = np.matmul(self.white_area_center, local_point)[:3]
        position_w[2] = height_m
        return position_w, local_map, projection_matrix

    def get_pixel_position(self, bgr_image, display_image, point):
        from app.utils import image_process, distortion_inverse_map
        if not hasattr(self, '_image_process'):
            self._image_process = image_process.GetObjectSurface(100, 200)
        if self.color_picker is not None and self.target_color is None:
            self.target_color, display_image = self.color_picker(bgr_image, display_image)
            if self.target_color is not None:
                self._log_pick_debug(
                    f'[PIXEL_PICK_DEBUG] selected_color_lab={self.target_color[0]}, selected_color_bgr={self.target_color[1]}, '
                    f'click_px=({int(point[0])},{int(point[1])})',
                    min_interval=0.0,
                    key='pixel_color',
                )
        elif self.target_color is not None and point:
            roi_img = self._image_process.get_top_surface(bgr_image)
            image_lab = cv2.cvtColor(cv2.GaussianBlur(roi_img, (3, 3), 3), cv2.COLOR_BGR2LAB)
            lab_name, min_color, max_color = self._select_lab_color_range(self.target_color[0])
            if min_color is None or max_color is None:
                threshold = 0.3
                min_color = [int(self.target_color[0][0] - 50 * threshold * 2),
                             int(self.target_color[0][1] - 50 * threshold),
                             int(self.target_color[0][2] - 50 * threshold)]
                max_color = [int(self.target_color[0][0] + 50 * threshold * 2),
                             int(self.target_color[0][1] + 50 * threshold),
                             int(self.target_color[0][2] + 50 * threshold)]
                lab_name = 'click_dynamic'
            mask = cv2.inRange(image_lab, tuple(min_color), tuple(max_color))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour_areas_all = [math.fabs(cv2.contourArea(c)) for c in contours]
            contours_area = list(zip(contour_areas_all, contours))
            contours = list(map(lambda a_c: a_c[1], filter(lambda a: 500 <= a[0] <= 7000, contours_area)))
            self._log_pick_debug(
                f'[PIXEL_MASK_DEBUG] click_px=({int(point[0])},{int(point[1])}), lab_source={lab_name}, '
                f'lab_min={min_color}, lab_max={max_color}, '
                f'contours_all={len(contour_areas_all)}, contours_filtered={len(contours)}, '
                f'areas_top={sorted([round(a, 1) for a in contour_areas_all], reverse=True)[:8]}',
                min_interval=0.8,
                key='pixel_mask',
            )
            min_d = float('inf')
            target = None
            for c in contours:
                rect = cv2.minAreaRect(c)
                d = math.sqrt((rect[0][0] - point[0]) ** 2 + (rect[0][1] - point[1]) ** 2)
                if d < min_d:
                    min_d = d
                    target = rect
            self.color_picker = None
            if target is not None:
                cv2.circle(display_image, (int(target[0][0]), int(target[0][1])), 8, (0, 255, 255), -1)
                box = np.intp(cv2.boxPoints(target))
                cv2.drawContours(display_image, [box], -1, (0, 255, 255), 2, cv2.LINE_AA)
                undistorted_target = target
                if self.camera_type == 'usb_cam':
                    x, y = distortion_inverse_map.undistorted_to_distorted_pixel(
                        target[0][0], target[0][1], self.intrinsic, self.distortion)
                    target = ((x, y), target[1], target[-1])
                self._log_pick_debug(
                    f'[PIXEL_TARGET_DEBUG] click_px=({int(point[0])},{int(point[1])}), '
                    f'undistorted_center=({float(undistorted_target[0][0]):.1f},{float(undistorted_target[0][1]):.1f}), '
                    f'world_query_px=({float(target[0][0]):.1f},{float(target[0][1]):.1f}), '
                    f'size=({float(target[1][0]):.1f},{float(target[1][1]):.1f}), angle={float(target[-1]):.1f}, '
                    f'distance_to_click_px={float(min_d):.1f}',
                    min_interval=0.0,
                    key='pixel_target',
                )
                position_m, _ = self.get_object_world_position(target[0], self.intrinsic)
                if self.pixel_debug:
                    self.get_logger().info(f'[像素定位] 像素点: ({target[0][0]:.1f}, {target[0][1]:.1f}), 世界坐标(m): x={position_m[0]:.4f}, y={position_m[1]:.4f}, z={position_m[2]:.4f}, 角度: {target[-1]:.1f}')
                return [position_m, target[-1]]
            else:
                self._log_pick_debug(
                    f'[PIXEL_TARGET_DEBUG] no target found near click_px=({int(point[0])},{int(point[1])})',
                    min_interval=0.8,
                    key='pixel_none',
                )
        return None

    def get_object_position(self, depth_image, depth_color_map, bgr_image, camera_info, depth_camera_info, point):
        from app.utils import utils
        if point:
            h, w = depth_image.shape[:2]
            roi_h, roi_w = 5, 5
            w_1 = max(0, int(point[0] - roi_w))
            w_2 = min(w, int(point[0] + roi_w))
            h_1 = max(0, int(point[1] - roi_h))
            h_2 = min(h, int(point[1] + roi_h))
            cv2.rectangle(depth_color_map, (w_1, h_1), (w_2, h_2), (0, 255, 0), 2)
            roi = depth_image[h_1:h_2, w_1:w_2]
            distances = roi[np.logical_and(roi > 0, roi < 40000)]
            if self.depth_verbose:
                self._log_pick_debug(
                    f'[DEPTH_CLICK_DEBUG] query_px=({int(point[0])},{int(point[1])}), roi=({w_1},{h_1})-({w_2},{h_2}), '
                    f'valid_depth_count={len(distances)}, depth_min={float(np.min(distances)) if len(distances) > 0 else -1:.1f}, '
                    f'depth_max={float(np.max(distances)) if len(distances) > 0 else -1:.1f}, image_size=({w},{h})',
                    min_interval=0.5,
                    key='depth_click',
                )
            if len(distances) > 0:
                distance = int(np.mean(distances))
                plane_values = utils.get_plane_values(depth_image, self.plane, depth_camera_info.k)
                plane_roi = plane_values[h_1:h_2, w_1:w_2]
                finite_plane_roi = plane_roi[np.isfinite(plane_roi)]
                point_x = int(np.clip(point[0], 0, w - 1))
                point_y = int(np.clip(point[1], 0, h - 1))
                if finite_plane_roi.size > 0:
                    roi_plane_min = float(np.min(finite_plane_roi))
                    roi_plane_max = float(np.max(finite_plane_roi))
                    roi_plane_mean = float(np.mean(finite_plane_roi))
                    roi_plane_over = int(np.count_nonzero(finite_plane_roi > 0.015))
                else:
                    roi_plane_min = -1.0
                    roi_plane_max = -1.0
                    roi_plane_mean = -1.0
                    roi_plane_over = 0
                plane_depth_m = self._plane_depth_at_pixel(point_x, point_y, self.plane, depth_camera_info.k)
                click_depth_m = float(distance) / 1000.0
                self._log_pick_debug(
                    f'[DEPTH_PLANE_ROI_DEBUG] query_px=({int(point[0])},{int(point[1])}), '
                    f'point_plane_value={float(plane_values[point_y, point_x]):.5f}, '
                    f'roi_plane_min={roi_plane_min:.5f}, roi_plane_max={roi_plane_max:.5f}, '
                    f'roi_plane_mean={roi_plane_mean:.5f}, roi_over_0.015={roi_plane_over}/{int(finite_plane_roi.size)}, '
                    f'click_depth={click_depth_m:.4f}, plane_depth={plane_depth_m:.4f}, '
                    f'depth_minus_plane={(click_depth_m - plane_depth_m):.4f}, '
                    f'plane={self._vec_text(self.plane)}',
                    min_interval=0.5,
                    key='depth_plane_roi',
                )
                contours_list = utils.extract_contours(plane_values, 0.015)
                contour_source = 'factory_global_plane'
                min_distance = float('inf')
                center = None
                contour_debug = []
                for cnt in contours_list:
                    (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                    dist = np.sqrt((cx - point[0]) ** 2 + (cy - point[1]) ** 2)
                    cx_i = int(np.clip(round(cx), 0, w - 1))
                    cy_i = int(np.clip(round(cy), 0, h - 1))
                    center_depth = float(depth_image[cy_i, cx_i])
                    center_plane_value = float(plane_values[cy_i, cx_i])
                    area = float(cv2.contourArea(cnt))
                    contour_debug.append((
                        float(dist),
                        f'c=({cx_i},{cy_i}),d={float(dist):.1f},area={area:.1f},'
                        f'depth={center_depth:.1f},pv={center_plane_value:.5f}',
                    ))
                    if dist < min_distance:
                        min_distance = dist
                        center = [cnt, int(cx), int(cy), center_depth]
                contour_debug = [item[1] for item in sorted(contour_debug, key=lambda item: item[0])[:12]]
                self._log_pick_debug(
                    f'[DEPTH_CONTOUR_DEBUG] query_px=({int(point[0])},{int(point[1])}), contours={len(contours_list)}, '
                    f'selected_center={None if center is None else (center[1], center[2])}, '
                    f'selected_distance_px={float(min_distance) if center is not None else -1:.2f}, '
                    f'selected_depth={None if center is None else center[3]}, source={contour_source}, '
                    f'click_depth_mean={distance}, contour_nearest={contour_debug}',
                    min_interval=0.5,
                    key='depth_contour',
                )
                if center is not None:
                    [x, y], (width_r, height_r), angle = cv2.minAreaRect(center[0])
                    x, y = int(x), int(y)
                    depth = depth_image[y, x]
                    self.last_depth_target_px = (x, y)
                    self.last_depth_query_px = (int(point[0]), int(point[1]))
                    depth_height_m = float(plane_values[y, x])
                    position_m, local_map, projection_matrix = self.get_depth_world_position_from_calibration(
                        (x, y), np.matrix(camera_info.k).reshape(1, -1, 3), depth_height_m)
                    raw_depth_position = np.array(position_m, dtype=np.float64)
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['depth']['offset'])
                    scale = tuple(config_data['depth']['scale'])
                    for i in range(3):
                        position_m[i] = position_m[i] * scale[i]
                        position_m[i] = position_m[i] + offset[i]
                    cv2.circle(depth_color_map, (x, y), 5, (0, 0, 255), -1)
                    cv2.drawMarker(depth_color_map, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
                    cv2.drawContours(depth_color_map,
                                     [np.intp(cv2.boxPoints(((x, y), (width_r, height_r), angle)))],
                                     -1, (0, 0, 255), 2, cv2.LINE_AA)
                    if self.depth_debug:
                        self.get_logger().warn(
                            f'[DEPTH_TARGET] query_px=({int(point[0])},{int(point[1])}), target_px=({x},{y}), '
                            f'distance_px={float(min_distance):.2f}, depth={float(depth):.1f}'
                        )
                    self._log_pick_debug(
                        f'[DEPTH_WORLD_DEBUG] target_px=({x},{y}), query_px=({int(point[0])},{int(point[1])}), '
                        f'depth={float(depth):.1f}, plane={self._vec_text(self.plane)}, '
                        f'raw_world={self._vec_text(raw_depth_position)}, scaled_world={self._vec_text(position_m)}, '
                        f'local_map={self._vec_text(local_map)}, height_m={depth_height_m:.4f}, '
                        f'depth_scale={scale}, depth_offset={offset}, angle={float(angle):.2f}, '
                        f'mapping=calibrated_map_projection, projection={self._matrix_summary(projection_matrix)}',
                        min_interval=0.0,
                        key='depth_world',
                    )
                    return [position_m, angle]
                else:
                    self._log_pick_debug(
                        f'[DEPTH_TARGET_DEBUG] no contour target near query_px=({int(point[0])},{int(point[1])})',
                        min_interval=0.8,
                        key='depth_none',
                    )
            else:
                self._log_pick_debug(
                    f'[DEPTH_TARGET_DEBUG] no valid depth around query_px=({int(point[0])},{int(point[1])})',
                    min_interval=0.8,
                    key='depth_no_value',
                )
        return None

    def pick_and_place_thread(self):
        while self.running:
            if self.start_transport:
                position_m, angle = self.transport_info
                position_m = np.array(position_m, dtype=np.float64).copy()
                endpoint = self.get_endpoint_matrix()
                if self.verbose_logs:
                    self.get_logger().info(f'[抓取] 当前末端矩阵:\n{endpoint}')
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                grasp_roll_deg, grasp_yaw_deg, base_yaw_deg = self.calculate_pick_grasp_roll(position_m, angle)
                far_reach_compensation = 0.0
                if self.verbose_logs:
                    self.get_logger().warn(f'[抓取] 原始世界坐标(m): x={position_m[0]:.4f}, y={position_m[1]:.4f}, z={position_m[2]:.4f}')
                for i in range(3):
                    position_m[i] = position_m[i] * scale[i]
                    position_m[i] = position_m[i] + offset[i]
                if self.verbose_logs:
                    self.get_logger().warn(f'[抓取] 校准后坐标(m): x={position_m[0]:.4f}, y={position_m[1]:.4f}, z={position_m[2]:.4f}, scale={scale}, offset={offset}')
                    self.get_logger().warn(
                        f'[抓取姿态] 物体角度={float(angle):.2f}, 基座朝向={base_yaw_deg:.2f}, 抓取旋转={grasp_yaw_deg:.2f}, '
                        f'roll={grasp_roll_deg:.2f}, z补偿={far_reach_compensation:.3f}'
                    )
                x_mm = position_m[0] * 1000.0
                y_mm = position_m[1] * 1000.0
                z_mm = (position_m[2] + 0.03) * 1000.0
                if self.verbose_logs:
                    self.get_logger().warn(f'[抓取] 发送坐标(mm): x={x_mm:.1f}, y={y_mm:.1f}, z_上方={z_mm:.1f}')
                self.home_pose = self._load_home_pose_from_scene()
                pick_pitch = self.home_pose['pitch']
                self.publish_arm(x_mm, y_mm, z_mm, pick_pitch, grasp_roll_deg, OPEN_CLAW, 1500)
                time.sleep(1.8)
                if self.grab_calib_mode:
                    # 运动学校准：只悬停不夹取，保持 grab_calib_mode 以便下次点击继续干运行
                    with self.lock:
                        self.center = []
                        self.start_transport = False
                    continue
                z_mm_down = (position_m[2] - 0.015) * 1000.0
                if self.verbose_logs:
                    self.get_logger().warn(f'[抓取] 下降坐标(mm): x={x_mm:.1f}, y={y_mm:.1f}, z_下降={z_mm_down:.1f}')
                self.publish_arm(x_mm, y_mm, z_mm_down, pick_pitch, grasp_roll_deg, OPEN_CLAW, 800)
                time.sleep(1.0)
                self.publish_arm(x_mm, y_mm, z_mm_down, pick_pitch, grasp_roll_deg, GRAB_CLAW, 500)
                time.sleep(0.6)
                self.publish_arm(x_mm, y_mm, z_mm, pick_pitch, grasp_roll_deg, GRAB_CLAW, 800)
                time.sleep(1.0)
                self.publish_arm(x_mm, y_mm, z_mm, pick_pitch, grasp_roll_deg, OPEN_CLAW, 500)
                time.sleep(0.6)
                self.go_home(claw=OPEN_CLAW)
                with self.lock:
                    self.center = []
                    self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while self.running:
            try:
                if self.calibration:
                    result_image = self.image_queue.get(block=True, timeout=10)
                else:
                    bgr_image, camera_info = self.rgb_image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            if not self.calibration:
                with self.lock:
                    self.intrinsic = np.matrix(camera_info.k).reshape(1, -1, 3)
                    self.distortion = np.array(camera_info.d)
                result_image = np.copy(bgr_image)
            depth_color_map = None
            if not self.start_transport:
                with self.lock:
                    position = None
                    if self.center and self.mode == 'color' and not self.calibration:
                        result = self.get_pixel_position(bgr_image, result_image, self.center)
                        if result is not None:
                            position, angle = result
                    if self.depth_enable:
                        from app.utils import utils
                        try:
                            depth_image, depth_camera_info = self.depth_image_queue.get(block=True, timeout=1)
                        except queue.Empty:
                            if self.depth_verbose and self.center and self.mode == 'depth':
                                self._log_pick_debug(
                                    f'[DEPTH_PIPELINE_DEBUG] query_px=({int(self.center[0])},{int(self.center[1])}), '
                                    f'depth_queue_empty=1, depth_enable={self.depth_enable}',
                                    min_interval=0.8,
                                    key='depth_pipeline_empty',
                                )
                            continue
                        max_dist = 350
                        depth_image = utils.create_roi_mask(
                            depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.05)
                        if self.depth_verbose and self.center and self.mode == 'depth':
                            valid_after_mask = int(np.count_nonzero(np.logical_and(depth_image > 0, depth_image < max_dist)))
                            query_x = int(np.clip(self.center[0], 0, depth_image.shape[1] - 1))
                            query_y = int(np.clip(self.center[1], 0, depth_image.shape[0] - 1))
                            self._log_pick_debug(
                                f'[DEPTH_PIPELINE_DEBUG] query_px=({int(self.center[0])},{int(self.center[1])}), '
                                f'clamped_px=({query_x},{query_y}), depth_at_query_after_roi={float(depth_image[query_y, query_x]):.1f}, '
                                f'valid_after_roi={valid_after_mask}, corners_count={len(np.array(self.corners).reshape((-1, 3)))}, '
                                f'plane={self._vec_text(self.plane)}',
                                min_interval=0.5,
                                key='depth_pipeline',
                            )
                        sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                        depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                        if self.center and self.mode == 'depth':
                            result = self.get_object_position(
                                depth_image, depth_color_map, bgr_image, camera_info, depth_camera_info, self.center)
                            if result is not None:
                                position, angle = result
                    if self.last_position is not None and position is not None:
                        e_distance = round(
                            math.sqrt(pow(self.last_position[0] - position[0], 2)) +
                            math.sqrt(pow(self.last_position[1] - position[1], 2)), 5)
                        if e_distance <= 0.005:
                            self.count_move = 0
                            self.count_still += 1
                        else:
                            self.count_move += 1
                            self.count_still = 0
                        if self.count_move > 10:
                            self.count_move = 0
                        if self.count_still > 5:
                            self.count_still = 0
                            self.count_move = 0
                            self.transport_info = [position, angle]
                            self.start_transport = True
                        if self.depth_verbose and self.center:
                            self._log_pick_debug(
                                f'[PICK_STABLE_DEBUG] mode={self.mode}, e_distance={e_distance:.5f}, '
                                f'count_still={self.count_still}, count_move={self.count_move}, '
                                f'start_transport={self.start_transport}, position={self._vec_text(position)}, '
                                f'last_position={self._vec_text(self.last_position)}',
                                min_interval=0.3,
                                key='depth_stable' if self.mode == 'depth' else 'pixel_stable',
                            )
                    elif self.depth_verbose and self.center and self.mode == 'depth':
                        self._log_pick_debug(
                            f'[PICK_STABLE_DEBUG] mode=depth, waiting_for_two_valid_positions=1, '
                            f'position={None if position is None else self._vec_text(position)}, '
                            f'last_position={None if self.last_position is None else self._vec_text(self.last_position)}, '
                            f'count_still={self.count_still}, count_move={self.count_move}',
                            min_interval=0.5,
                            key='depth_stable_wait',
                        )
                    self.last_position = position
            self.fps.update()
            self.fps.show_fps(result_image if not self.calibration else result_image)
            # 发布纯颜色处理图（不含深度，供上位机颜色标定框显示）
            _color_pub = result_image[40:440, ] if self.camera_type == 'usb_cam' else result_image
            try:
                self.display_pub.publish(self.bridge.cv2_to_imgmsg(_color_pub, encoding='bgr8'))
            except Exception:
                pass
            # 发布深度图（供上位机深度标定框显示）
            if self.depth_enable and depth_color_map is not None:
                _depth_pub = depth_color_map[40:440, ] if self.camera_type == 'usb_cam' else depth_color_map
                try:
                    self.depth_display_pub.publish(
                        self.bridge.cv2_to_imgmsg(_depth_pub, encoding='bgr8'))
                except Exception:
                    pass
                result_image = np.concatenate([result_image, depth_color_map], axis=1)
            if self.camera_type == 'usb_cam':
                result_image = result_image[40:440, ]
            cv2.imshow('result_image', result_image)
            if not self.set_callback:
                self.set_callback = True
                cv2.setMouseCallback("result_image", self.mouse_callback)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def rgb_callback(self, ros_rgb_image, camera_info):
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.rgb_image_queue.full():
            self.rgb_image_queue.get()
        self.rgb_image_queue.put((bgr_image, camera_info))

    def depth_callback(self, ros_depth_image, depth_camera_info):
        self.depth_enable = True
        depth_image = np.ndarray(
            shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        if self.depth_image_queue.full():
            self.depth_image_queue.get()
        self.depth_image_queue.put((depth_image, depth_camera_info))

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(image)

    def finish_calibration_callback(self, msg):
        with self.lock:
            self._load_hand2cam_from_file(force=True)
            with open(self.config_path + self.config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.plane = config['plane']
                self.corners = np.array(config['corners'])
                self.extristric = np.array(config['extristric'])
                self.white_area_center = self._normalize_white_area_pose_world(config['white_area_pose_world'])
                self.white_area_length_m, self.white_area_width_m = self._load_scene_dimensions()
                self.home_pose = self._load_home_pose_from_scene()
                if self.verbose_logs:
                    self.get_logger().info(
                        f'[CALIB_RELOAD] plane={self._vec_text(self.plane)}, extristric={self._matrix_summary(self._extristric_to_matrix(self.extristric))}, '
                        f'white_area_center={self._matrix_summary(self.white_area_center)}'
                    )


def main():
    node = ColorPick('color_pick')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
