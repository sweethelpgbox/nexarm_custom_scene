#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import time
import yaml
import math
import rclpy
import queue
import threading
import numpy as np
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from app import scene4_runtime
from app.utils import search_plane
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from dt_apriltags import Detector
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from tf2_ros import Buffer, TransformListener, TransformException

APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"

SCENE0_ID = 'scene_0'
DEFAULT_SCENE_ID = 'scene_1'
SCENE5_ID = 'scene_5'
DEFAULT_CONTROLLER_PREFIX = '/ros_robot_controller'
SCENE5_ARM_PREFIXES = {
    'A': '/arm_a/ros_robot_controller',
    'B': '/arm_b/ros_robot_controller',
}
DEFAULT_CURRENT_SCENE = SCENE0_ID
DEFAULT_SCENE_CONFIG = {
    'current_scene': SCENE0_ID,
    'scenes': {
        SCENE0_ID: {
            'name': 'Scene 0',
            'mode': 'arm_body',
            'length_m': 0.13,
            'width_m': 0.167,
            'calibration_tag': {
                'id': 100,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'yaw_deg': 0.0,
                'center_in_map_m': {
                    'x': -0.045,
                    'y': -0.0635,
                    'z': 0.0,
                },
            },
            'home_pose': {
                'x': 200.0,
                'y': 0.0,
                'z': 200.0,
                'pitch': -90.0,
                'roll': 0.0,
                'claw': 0.0,
                'time_ms': 2000,
            },
        },
        DEFAULT_SCENE_ID: {
            'name': 'Scene 1',
            'length_m': 0.158,
            'width_m': 0.175,
            'home_pose': {
                'x': 110.0,
                'y': 0.0,
                'z': 220.0,
                'pitch': -90.0,
                'roll': 0.0,
                'claw': 0.0,
            },
        },
    },
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


class CalibrationNode(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        self.running = True
        self.imgpts = None
        self.imgpts1 = None
        self.plane = []
        self._init_parameters()
        # AprilTag 位姿解算边长（白黑边界之间的有效边长，单位：m）
        self.tag_size = 0.033
        # AprilTag 纸面外框边长（用于“左上角标签 -> 场景中心”偏移换算，单位：m）
        self.tag_outer_size = 0.040
        self.tag_id = [1, 2, 3]
        self.tag_id_2 = 100

        self.camera_type = os.environ.get('CAMERA_TYPE', 'usb_cam')
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        self.verbose_logs = os.environ.get('CALIB_VERBOSE_LOGS', '0') == '1'
        self.geometry_debug = os.environ.get('CALIB_GEOM_DEBUG', '0') != '0'
        self.plane_roi_depth_frame = os.environ.get('CALIB_PLANE_ROI_DEPTH_FRAME', '0') == '1'
        self._last_geometry_debug_ts = 0.0

        self.config_file = 'transform.yaml'
        self.camera_info_file = '/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml'

        self.config_path = os.path.dirname(scene4_runtime.scene_config_path()) + "/"
        self.scene_config_file = 'calibration_scene.yaml'
        self.scene_name = DEFAULT_CURRENT_SCENE
        self.calibration_scene_name = DEFAULT_CURRENT_SCENE
        self.white_area_width = 0.167
        self.white_area_height = 0.13
        self.calibration_tag_id = 1
        self.scene_tag_center_in_map = None
        self.scene_tag_yaw_deg = 0.0
        self.home_pose = dict(DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_CURRENT_SCENE]['home_pose'])
        self.calibration_pose = dict(self.home_pose, time_ms=int(self.home_pose.get('time_ms', 1500)))
        self.scene4_calibration_rail_ready = False
        self._load_scene_config(force=True)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        self.controller_prefix = scene5_calibration_controller_prefix(self.active_scene_name)

        # --- 新底层: ArmCoords 发布 + ArmFullState 订阅 ---
        self.arm_pub = self.create_publisher(ArmCoords, self.ctl_topic('arm/set_coords'), 5)
        self.current_pose = None
        self.known_pose = {
            'x': float(self.home_pose['x']),
            'y': float(self.home_pose['y']),
            'z': float(self.home_pose['z']),
            'pitch': float(self.home_pose['pitch']),
            'roll': float(self.home_pose['roll']),
            'claw': float(self.home_pose['claw']),
            'yaw': 0.0,
        }
        self.create_subscription(ArmFullState, self.ctl_topic('arm/full_state'), self.arm_state_callback, 5)
        self.client_cb_group = ReentrantCallbackGroup()
        self.arm_state_client = self.create_client(
            GetArmFullState,
            self.ctl_topic('arm/get_full_state'),
            callback_group=self.client_cb_group,
        )
        self.scene4_stepper_client = scene4_runtime.create_stepper_position_client(
            self,
            callback_group=self.client_cb_group,
        )

        self.result_image_pub = self.create_publisher(Image, '~/image_result', 10)
        self.finish_pub = self.create_publisher(Bool, '~/finish', 1)

        # 服务
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/start', self.start_calibration_srv_callback)
        self.create_service(SetBool, '~/start_calibration', self.enable_calibration_srv_callback)

        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        self.hand2cam_tf_matrix_color = np.eye(4)
        self.hand2cam_tf_matrix_depth = np.eye(4)
        self.hand2cam_file_mtime = None
        self.color_to_depth_transform_matrix = None

        tf_buffer = Buffer()
        self.tf_listener = TransformListener(tf_buffer, self)
        try:
            _cam_pfx = self._tf_camera_prefix()
            depth_frame = f'{_cam_pfx}depth_cam_depth_optical_frame' if self.camera_type == 'aurora' else f'{_cam_pfx}depth_camera_link'
            color_frame = f'{_cam_pfx}depth_cam_color_frame'
            tf_future = tf_buffer.wait_for_transform_async(
                target_frame=depth_frame,
                source_frame=color_frame,
                time=rclpy.time.Time()
            )
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=10.0)
            transform = tf_buffer.lookup_transform(
                depth_frame, color_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0))
            self.static_transform = transform

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.color_to_depth_transform_matrix = common.xyz_quat_to_mat(
                [translation.x, translation.y, translation.z],
                [rotation.w, rotation.x, rotation.y, rotation.z])
        except Exception as e:
            self.get_logger().warn(f'获取静态变换失败(可忽略): {e}')

        self._load_hand2cam_from_file(force=True)
        # self.get_logger().info('[PATCH] calibration endpoint uses real ArmFullState orientation')

        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            self.extristric = np.array(config['extristric'])
            self.white_area_pose_cam = np.array(config['white_area_pose_cam'])
            self.white_area_pose_world = self._normalize_white_area_pose_world(config['white_area_pose_world'])

        threading.Thread(target=self.image_processing, daemon=True).start()

    def _init_parameters(self):
        self.thread = None
        self.err_msg = None
        self.heart = None
        self.calibration_step = 0
        self.tags = []
        self.pose = []
        self.tag_count = 0
        self.K = None
        self.D = None
        self.enter = False

        self.image_sub = None
        self.camera_info_sub = None
        self.depth_image_sub = None
        self.depth_cam_info = None
        self.depth_camera_info_sub = None
        self.last_calibration_endpoint = None

    def ctl_topic(self, suffix):
        return f'{self.controller_prefix.rstrip("/")}/{suffix.lstrip("/")}'

    def _camera_topic(self, suffix):
        """根据当前场景返回正确的相机话题路径（scene5 A 加 /arm_a 前缀）"""
        if self.active_scene_name == SCENE5_ID:
            role = normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE'))
            return f'/arm_{role.lower()}/depth_cam/{suffix.lstrip("/")}'
        return f'/depth_cam/{suffix.lstrip("/")}'

    def _tf_camera_prefix(self):
        """返回 TF 帧名前缀（scene5 A/B 时为 'arm_a/' 或 'arm_b/'，其余为 ''）"""
        if self.active_scene_name == SCENE5_ID:
            role = normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE'))
            return f'arm_{role.lower()}/'
        return ''

    def _selected_scene_from_env(self, scenes):
        for key in ('CALIBRATION_CURRENT_SCENE', 'CALIBRATION_DEFAULT_SCENE', 'SCENE'):
            scene_name = os.environ.get(key)
            if scene_name in scenes:
                return scene_name
        return None

    def _load_scene_config(self, force=False):
        scene_path = os.path.join(self.config_path, self.scene_config_file)
        config_data = {}
        try:
            with open(scene_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f) or {}
        except Exception as exc:
            if force:
                self.get_logger().warn(f'读取场景配置失败，使用默认场景: {exc}')
            config_data = {}

        scenes = config_data.get('scenes') if isinstance(config_data, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            scenes = dict(DEFAULT_SCENE_CONFIG['scenes'])
        else:
            scenes = dict(scenes)
            for scene_id, scene_cfg in DEFAULT_SCENE_CONFIG['scenes'].items():
                scenes.setdefault(scene_id, scene_cfg)

        # 用 app config 补全当前 YAML 缺少的场景（场景3/5 等在 stepper config 可能没有）
        _fallback_path = APP_SCENE_CONFIG if os.path.abspath(scene_path) != os.path.abspath(APP_SCENE_CONFIG) else STEPPER_SCENE_CONFIG
        try:
            with open(_fallback_path, 'r', encoding='utf-8') as _f:
                _fb = yaml.safe_load(_f) or {}
            for _sid, _scfg in (_fb.get('scenes') or {}).items():
                scenes.setdefault(_sid, _scfg)
        except Exception:
            pass

        active_scene_name = self._selected_scene_from_env(scenes)
        if active_scene_name is None:
            active_scene_name = config_data.get('current_scene', DEFAULT_CURRENT_SCENE)
        if active_scene_name not in scenes:
            active_scene_name = DEFAULT_CURRENT_SCENE if DEFAULT_CURRENT_SCENE in scenes else next(iter(scenes.keys()))
        active_scene_cfg = scenes.get(active_scene_name, {})
        if not isinstance(active_scene_cfg, dict):
            active_scene_cfg = {}
        calibration_scene_name = active_scene_cfg.get('use_calibration_scene', active_scene_name)
        if calibration_scene_name not in scenes:
            calibration_scene_name = active_scene_name
        calibration_scene_cfg = scenes.get(calibration_scene_name, {})
        if not isinstance(calibration_scene_cfg, dict):
            calibration_scene_cfg = {}
        scene_cfg = active_scene_cfg if isinstance(active_scene_cfg, dict) else {}

        length_m = scene_cfg.get('length_m')
        width_m = scene_cfg.get('width_m')
        white_area_cfg = scene_cfg.get('white_area', {}) if isinstance(scene_cfg.get('white_area'), dict) else {}
        if length_m is None:
            fallback_white_area_cfg = calibration_scene_cfg.get('white_area', {}) if isinstance(calibration_scene_cfg.get('white_area'), dict) else {}
            length_m = white_area_cfg.get('length_m', calibration_scene_cfg.get('length_m', fallback_white_area_cfg.get('length_m', self.white_area_height)))
        if width_m is None:
            fallback_white_area_cfg = calibration_scene_cfg.get('white_area', {}) if isinstance(calibration_scene_cfg.get('white_area'), dict) else {}
            width_m = white_area_cfg.get('width_m', calibration_scene_cfg.get('width_m', fallback_white_area_cfg.get('width_m', self.white_area_width)))

        try:
            self.white_area_height = float(length_m)
            self.white_area_width = float(width_m)
        except Exception:
            self.white_area_height = 0.167
            self.white_area_width = 0.13

        default_home = DEFAULT_SCENE_CONFIG['scenes'].get(
            str(active_scene_name),
            DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_CURRENT_SCENE],
        ).get('home_pose', DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_CURRENT_SCENE]['home_pose'])
        home = active_scene_cfg.get('home_pose', {})
        if not isinstance(home, dict):
            home = {}
        self.home_pose = {
            'x': float(home.get('x', default_home.get('x', 200.0))),
            'y': float(home.get('y', default_home.get('y', 0.0))),
            'z': float(home.get('z', default_home.get('z', 200.0))),
            'pitch': float(home.get('pitch', default_home.get('pitch', 0.0))),
            'roll': float(home.get('roll', default_home.get('roll', 0.0))),
            'claw': float(home.get('claw', default_home.get('claw', 0.0))),
            'time_ms': int(float(home.get('time_ms', default_home.get('time_ms', 1500)))),
        }
        if str(active_scene_name) == scene4_runtime.SCENE4_ID:
            self.calibration_pose = scene4_runtime.scene4_calibration_pose(scene_cfg)
        elif isinstance(scene_cfg.get('calibration_pose'), dict):
            raw_pose = scene_cfg.get('calibration_pose', {})
            self.calibration_pose = {
                'x': float(raw_pose.get('x', self.home_pose['x'])),
                'y': float(raw_pose.get('y', self.home_pose['y'])),
                'z': float(raw_pose.get('z', self.home_pose['z'])),
                'pitch': float(raw_pose.get('pitch', self.home_pose['pitch'])),
                'roll': float(raw_pose.get('roll', self.home_pose['roll'])),
                'claw': float(raw_pose.get('claw', self.home_pose['claw'])),
                'time_ms': int(float(raw_pose.get('time_ms', self.home_pose.get('time_ms', 1500)))),
            }
        else:
            self.calibration_pose = dict(self.home_pose, time_ms=int(self.home_pose.get('time_ms', 1500)))
        self.active_scene_name = str(active_scene_name)
        self.controller_prefix = scene5_calibration_controller_prefix(self.active_scene_name)
        self.calibration_scene_name = str(calibration_scene_name)
        self.scene_name = str(active_scene_name)
        self.scene_tag_center_in_map = None
        tag_cfg = scene_cfg.get('calibration_tag', {}) if isinstance(scene_cfg.get('calibration_tag'), dict) else {}
        if not tag_cfg:
            tag_cfg = calibration_scene_cfg.get('calibration_tag', {}) if isinstance(calibration_scene_cfg.get('calibration_tag'), dict) else {}
        try:
            self.calibration_tag_id = int(tag_cfg.get('id', self.calibration_tag_id))
        except Exception:
            self.calibration_tag_id = 1
        try:
            self.tag_outer_size = float(tag_cfg.get('size_m', self.tag_outer_size))
        except Exception:
            self.tag_outer_size = 0.040
        try:
            self.tag_size = float(tag_cfg.get('effective_size_m', self.tag_size))
        except Exception:
            self.tag_size = 0.033
        center_cfg = tag_cfg.get('center_in_map_m', {}) if isinstance(tag_cfg.get('center_in_map_m'), dict) else {}
        try:
            cx = float(center_cfg.get('x'))
            cy = float(center_cfg.get('y'))
            cz = float(center_cfg.get('z', 0.0))
            self.scene_tag_center_in_map = np.array([cx, cy, cz], dtype=np.float64)
        except Exception:
            self.scene_tag_center_in_map = None
        try:
            self.scene_tag_yaw_deg = float(tag_cfg.get('yaw_deg', 0.0))
        except Exception:
            self.scene_tag_yaw_deg = 0.0
        self.get_logger().info(
            f"[SCENE] active={self.active_scene_name}, calibration={self.calibration_scene_name}: "
            f"length={self.white_area_height:.4f}m, width={self.white_area_width:.4f}m, "
            f"home=({self.home_pose['x']:.1f},{self.home_pose['y']:.1f},{self.home_pose['z']:.1f},"
            f"{self.home_pose['pitch']:.1f},{self.home_pose['roll']:.1f},{self.home_pose['claw']:.1f})"
        )
        self._log_geometry_debug(
            f"[CAL_FRAME_DEBUG][SCENE] active_scene={self.active_scene_name}, calibration_scene={self.calibration_scene_name}, "
            f"map_length_x={self.white_area_height:.4f}m, map_width_y={self.white_area_width:.4f}m, "
            f"tag_id={self.calibration_tag_id}, tag_effective={self.tag_size:.4f}m, "
            f"tag_outer={self.tag_outer_size:.4f}m, tag_center_in_map={self._vec_text(self._scene1_tag_center_offset_in_map())}, "
            f"tag_yaw_deg={self.scene_tag_yaw_deg:.2f}",
            min_interval=0.0,
            key='scene',
        )

    # --- 优先使用真实 ArmFullState，回退到最近一次发送位置 ---
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

    def _normalize_white_area_pose_world(self, pose_matrix):
        return np.array(pose_matrix, dtype=np.float64).reshape(4, 4)

    def _expected_calibration_tag_ids(self):
        # 场景配置了 calibration_tag 时，只接受该场景的标定标签。
        if self.scene_tag_center_in_map is not None or self.scene_name == 'scene_1':
            return {int(self.calibration_tag_id)}
        expected = set(self.tag_id)
        expected.add(self.tag_id_2)
        return expected

    def _tag_pose_to_white_area_pose_cam(self, tag_pose_cam):
        """
        将检测到的标签位姿转换为白板中心位姿（相机坐标系下）。
        配置了 calibration_tag.center_in_map_m 的场景：标签在地图上有固定位置，需换算回地图中心。
        """
        pose = np.array(tag_pose_cam, dtype=np.float64).reshape(4, 4)
        if self.scene_tag_center_in_map is None and self.scene_name != 'scene_1':
            return pose

        center_in_tag = self._scene1_map_point_to_tag_frame(np.zeros(3, dtype=np.float64))
        to_center = common.xyz_euler_to_mat(center_in_tag, (0, 0, 0))
        return np.matmul(pose, to_center)

    def _scene1_tag_center_offset_in_map(self):
        """
        标签中心在地图坐标系(以地图中心为原点)下的位置。
        scene_1: 标签位于左上角区域，按标签外框中心建模。
        """
        if self.scene_tag_center_in_map is not None:
            return np.array(self.scene_tag_center_in_map, dtype=np.float64)
        if self.scene_name != 'scene_1':
            return np.zeros(3, dtype=np.float64)
        return np.array([
            -(self.white_area_height / 2.0 - self.tag_outer_size / 2.0),
            self.white_area_width / 2.0 - self.tag_outer_size / 2.0,
            0.0,
        ], dtype=np.float64)

    def _scene1_tag_to_map_rotation(self):
        yaw = math.radians(float(getattr(self, 'scene_tag_yaw_deg', 0.0)))
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def _scene1_tag_point_to_map(self, tag_point):
        tag_point = np.array(tag_point, dtype=np.float64).reshape(3)
        return self._scene1_tag_center_offset_in_map() + self._scene1_tag_to_map_rotation().dot(tag_point)

    def _scene1_map_point_to_tag_frame(self, map_point):
        map_point = np.array(map_point, dtype=np.float64).reshape(3)
        return self._scene1_tag_to_map_rotation().T.dot(map_point - self._scene1_tag_center_offset_in_map())

    def _map_points_to_tag_frame(self, map_points):
        pts = np.array(map_points, dtype=np.float64).reshape((-1, 3))
        return np.array([self._scene1_map_point_to_tag_frame(p) for p in pts], dtype=np.float64)

    def _solve_map_extristric_from_tags(self, tags):
        if not tags:
            return None
        tag_local_corners = np.array([
            (-self.tag_size / 2, -self.tag_size / 2, 0),
            ( self.tag_size / 2, -self.tag_size / 2, 0),
            ( self.tag_size / 2,  self.tag_size / 2, 0),
            (-self.tag_size / 2,  self.tag_size / 2, 0),
        ], dtype=np.float64)
        map_corners = np.array([self._scene1_tag_point_to_map(p) for p in tag_local_corners], dtype=np.float64)
        world_points = np.tile(map_corners, (len(tags), 1)).astype(np.float64)
        image_points = np.array(list(map(lambda tag: tag.corners, tags)), dtype=np.float64).reshape((-1, 2))
        retval, rvec, tvec = cv2.solvePnP(world_points, image_points, self.K, self.D)
        if not retval:
            return None
        rmat, _ = cv2.Rodrigues(rvec)
        return [
            tvec.flatten().tolist(),
            rmat[0].tolist(),
            rmat[1].tolist(),
            rmat[2].tolist(),
        ]

    def _apply_loaded_hand2cam(self, hand2cam_matrix):
        self.hand2cam_tf_matrix_color = np.array(hand2cam_matrix, dtype=np.float64).reshape(4, 4)
        if self.color_to_depth_transform_matrix is not None:
            # Match factory_utils/calibration: depth hand-eye uses static camera
            # transform on the left, then the calibrated color hand-eye matrix.
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
                camera_info_config = yaml.safe_load(f) or {}
            matrix = camera_info_config.get('hand2cam_tf_matrix')
            if matrix is None:
                raise KeyError('camera_info.yaml 中缺少 hand2cam_tf_matrix')
            self._apply_loaded_hand2cam(matrix)
            self.hand2cam_file_mtime = mtime
            return True
        except Exception as exc:
            self.get_logger().error(f'加载 hand2cam_tf_matrix 失败: {exc}')
            return False

    def _vec_text(self, values, precision=4):
        return '(' + ', '.join(f'{float(v):.{precision}f}' for v in values) + ')'

    def _pts_text(self, points, precision=4):
        arr = np.array(points, dtype=np.float64)
        if arr.ndim == 1:
            pts = arr.reshape((1, -1))
        else:
            pts = arr.reshape((-1, arr.shape[-1]))
        return '[' + ', '.join(self._vec_text(p, precision=precision) for p in pts) + ']'

    def _log_geometry_debug(self, message, min_interval=0.5, key='geometry'):
        if not self.geometry_debug:
            return
        now = time.time()
        attr = f'_last_{key}_debug_ts'
        last = getattr(self, attr, 0.0)
        if now - last < min_interval:
            return
        setattr(self, attr, now)
        self.get_logger().warn(message)

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

    def _pose_delta_summary(self, current_pose, known_pose):
        if current_pose is None or known_pose is None:
            return 'current_pose_or_known_pose_missing'
        keys = ['x', 'y', 'z', 'pitch', 'roll', 'claw', 'yaw']
        return ', '.join(
            f"{key}={float(current_pose.get(key, 0.0)) - float(known_pose.get(key, 0.0)):.2f}"
            for key in keys
        )

    def request_real_pose_snapshot(self, timeout_sec=0.6):
        if not self.arm_state_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
            return None
        future = self.arm_state_client.call_async(GetArmFullState.Request())
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'获取真实末端状态失败: {exc}')
            return None
        if response is None or not response.success:
            return None
        pose = {
            'x': float(response.x),
            'y': float(response.y),
            'z': float(response.z),
            'pitch': float(response.pitch),
            'roll': float(response.roll),
            'claw': float(response.claw),
            'yaw': float(response.yaw),
            'joint_angles': [float(v) for v in response.joint_angles],
        }
        self.current_pose = dict(pose)
        return pose

    def get_endpoint_matrix(self, pose=None):
        """使用与手眼求解一致的末端姿态约定构建 4x4 齐次变换矩阵。"""
        p = pose if pose is not None else (self.current_pose if self.current_pose is not None else self.known_pose)
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

    def get_calibration_endpoint_matrix(self, pose=None):
        """标定阶段使用真实末端姿态，保持与原始标定链路一致。"""
        return self.get_endpoint_matrix(pose)


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
        if self.verbose_logs:
            self.get_logger().info(
                f'[CAL_ARM_CMD] send={self._format_pose(self.known_pose)}, time_ms={int(time_ms)}, '
                f'latest_real={self._format_pose(self.current_pose)}, real-minus-cmd=({self._pose_delta_summary(self.current_pose, self.known_pose)})'
            )

    def calibration_proc(self):
        self._load_hand2cam_from_file(force=True)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        endpoint = self.get_calibration_endpoint_matrix(pose_snapshot)
        self.last_calibration_endpoint = np.array(endpoint, dtype=np.float64)
        if self.verbose_logs:
            self.get_logger().info(
                f'[CAL_START] pose_snapshot={self._format_pose(pose_snapshot)}, endpoint={self._matrix_summary(endpoint)}, '
                f'hand2cam_color={self._matrix_summary(self.hand2cam_tf_matrix_color)}, hand2cam_depth={self._matrix_summary(self.hand2cam_tf_matrix_depth)}'
            )

        # 获取标签数据
        t = time.time()
        self.tags = []
        self.calibration_step = 1
        while self.calibration_step == 1 and time.time() - t < 10:
            time.sleep(0.1)

        if len(self.tags) < 5:
            self.err_msg = "Time out, calibrate failed!!!"
            time.sleep(3)
            self.err_msg = None
            self.calibration_step = 0
            self.thread = None
            return

        # 识别区域中心位置标定
        pose = map(lambda tag: common.xyz_rot_to_mat(tag.pose_t, tag.pose_R), self.tags)
        vectors = map(lambda p: p.ravel(), pose)
        avg_tag_pose_cam = np.mean(list(vectors), axis=0).reshape((4, 4))
        # 外参标定：只使用 tag corners + map geometry 的 solvePnP 链路。
        # dt_apriltags 的 tag.pose_R/pose_t 坐标轴约定与这里的 map 坐标
        # 不完全一致，不能再混入板面 ROI 或抓取坐标链路。
        tag_local_corners = np.array([
            (-self.tag_size / 2, -self.tag_size / 2, 0),
            ( self.tag_size / 2, -self.tag_size / 2, 0),
            ( self.tag_size / 2,  self.tag_size / 2, 0),
            (-self.tag_size / 2,  self.tag_size / 2, 0),
        ], dtype=np.float64)
        tag_center_offset = self._scene1_tag_center_offset_in_map()
        map_corners = np.array([self._scene1_tag_point_to_map(p) for p in tag_local_corners], dtype=np.float64)
        world_points = np.tile(map_corners, (len(self.tags), 1)).astype(np.float64)
        image_points = np.array(list(map(lambda tag: tag.corners, self.tags)), dtype=np.float64).reshape((-1, 2))
        first_tag_corners = np.array(self.tags[0].corners, dtype=np.float64).reshape((-1, 2))
        avg_tag_corners = np.mean(
            np.array([tag.corners for tag in self.tags], dtype=np.float64).reshape((-1, 4, 2)),
            axis=0,
        )
        self._log_geometry_debug(
            f"[CAL_FRAME_DEBUG][PNP_INPUT] samples={len(self.tags)}, scene={self.scene_name}, "
            f"map_length_x={self.white_area_height:.4f}, map_width_y={self.white_area_width:.4f}, "
            f"tag_center_offset_map={self._vec_text(tag_center_offset)}, tag_yaw_deg={self.scene_tag_yaw_deg:.2f}, "
            f"tag_local_corners={self._pts_text(tag_local_corners)}, "
            f"map_corners_used_for_tag={self._pts_text(map_corners)}, "
            f"first_tag_px={self._pts_text(first_tag_corners, precision=1)}, "
            f"avg_tag_px={self._pts_text(avg_tag_corners, precision=1)}",
            min_interval=0.0,
            key='pnp_input',
        )
        extristric = self._solve_map_extristric_from_tags(self.tags)
        if extristric is None:
            self.err_msg = 'solvePnP failed, calibrate failed!!!'
            time.sleep(3)
            self.err_msg = None
            self.calibration_step = 0
            self.thread = None
            return
        tvec = np.array(extristric[0], dtype=np.float64).reshape(3, 1)
        rmat = np.array(extristric[1:], dtype=np.float64).reshape(3, 3)
        map_pose_cam = np.eye(4, dtype=np.float64)
        map_pose_cam[:3, :3] = rmat
        map_pose_cam[:3, 3] = tvec.flatten()
        self.white_area_pose_cam = map_pose_cam
        white_area_pose_cam = map_pose_cam.tolist()
        pose_end = np.matmul(self.hand2cam_tf_matrix_color, map_pose_cam)
        pose_world_full = np.matmul(endpoint, pose_end)
        pose_world_saved = self._normalize_white_area_pose_world(pose_world_full)
        if self.geometry_debug:
            self.get_logger().info(
                f'[CAL_RESULT] avg_tag_pose_cam={self._matrix_summary(avg_tag_pose_cam)}, '
                f'map_pose_cam={self._matrix_summary(map_pose_cam)}, '
                f'pose_end={self._matrix_summary(pose_end)}, '
                f'pose_world_full={self._matrix_summary(pose_world_full)}, pose_world_saved={self._matrix_summary(pose_world_saved)}'
            )
        self.white_area_pose_world = pose_world_saved
        white_area_pose_world = pose_world_saved.tolist()
        projected, _ = cv2.projectPoints(map_corners, rmat, tvec, self.K, self.D)
        if self.geometry_debug:
            self.get_logger().warn(
                f"[CAL_TAG_MAP] yaw_deg={self.scene_tag_yaw_deg:.2f}, "
                f"tag_center_map={self._vec_text(tag_center_offset)}, "
                f"tag_local_corners={self._pts_text(tag_local_corners)}, "
                f"tag_corners_as_map={self._pts_text(map_corners)}, "
                f"projected_tag_px={self._pts_text(projected.reshape((-1, 2)), precision=1)}, "
                f"detected_tag_px={self._pts_text(avg_tag_corners, precision=1)}"
            )
        self._log_geometry_debug(
            f"[CAL_FRAME_DEBUG][PNP_RESULT] tvec={self._vec_text(tvec.flatten())}, "
            f"projected_map_corners_px={self._pts_text(projected.reshape((-1, 2)), precision=1)}, "
            f"expected_should_overlap_tag_px={self._pts_text(avg_tag_corners, precision=1)}",
            min_interval=0.0,
            key='pnp_result',
        )
        self.extristric = np.array(extristric)
        corners = self.draw_retangle()

        if self.camera_type == "aurora":
            data = {
                'white_area_pose_cam': white_area_pose_cam,
                'white_area_pose_world': white_area_pose_world,
                'extristric': extristric,
                'corners': corners.tolist(),
                'plane': self.plane.tolist(),
            }
        if self.camera_type == "usb_cam":
            data = {
                'white_area_pose_cam': white_area_pose_cam,
                'white_area_pose_world': white_area_pose_world,
                'extristric': extristric,
                'corners': corners.tolist(),
            }

        if self.verbose_logs:
            self.get_logger().info(
                f'[CAL_SAVE] extristric={self._matrix_summary(self._extristric_to_matrix(self.extristric))}, '
                f'white_area_pose_cam={self._matrix_summary(self.white_area_pose_cam)}, '
                f'white_area_pose_world={self._matrix_summary(self.white_area_pose_world)}'
            )
        self.update_yaml_data(data, self.config_path + self.config_file)
        msg = Bool()
        msg.data = True
        self.finish_pub.publish(msg)
        self.calibration_step = 20
        time.sleep(3)
        self.calibration_step = 0
        self.thread = None

    def update_yaml_data(self, new_data, yaml_file):
        if os.path.exists(yaml_file):
            with open(yaml_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        else:
            data = {}
        data.update(new_data)
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f)
        time.sleep(0.1)

    def start_calibration_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start calibration")
        self._load_scene_config(force=True)
        if not scene5_calibration_allowed(self.active_scene_name):
            response.success = False
            response.message = "scene_5 calibration is only allowed on arm A"
            return response
        self.imgpts = None
        self.imgpts1 = None
        self.extristric = None
        self.white_area_pose_cam = None
        self.white_area_pose_world = None
        if self.image_sub is None:
            err_msg = "Please call enter service first"
            self.get_logger().info(str(err_msg))
            response.success = False
            response.message = "stop"
            return response
        if self.thread is None:
            self.thread = threading.Thread(target=self.calibration_proc)
            self.thread.start()
            response.success = True
            response.message = "start"
            return response
        else:
            msg = "Calibration..."
            self.get_logger().info(msg)
            response.success = False
            response.message = "stop"
            return response

    def enable_calibration_srv_callback(self, request, response):
        response.success = True
        response.message = "start" if bool(request.data) else "stop"
        return response

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter calibration")
        self._load_scene_config(force=True)
        self._init_parameters()
        if self.active_scene_name == scene4_runtime.SCENE4_ID:
            if not scene4_runtime.move_scene4_rail(
                self,
                self.scene4_stepper_client,
                "calibration",
                scene_path=os.path.join(self.config_path, self.scene_config_file),
                logger=self.get_logger(),
                reset_first=False,
            ):
                self.scene4_calibration_rail_ready = False
                response.success = False
                response.message = "scene_4 rail move failed"
                return response
            self.scene4_calibration_rail_ready = True
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        if self.camera_type == "aurora":
            self.image_sub = self.create_subscription(Image, self._camera_topic('rgb/image_raw'), self.image_callback, 1)
            self.camera_info_sub = self.create_subscription(CameraInfo, self._camera_topic('rgb/camera_info'), self.camera_info_callback, 1)
            self.depth_image_sub = self.create_subscription(Image, self._camera_topic('depth/image_raw'), self.depth_image_callback, 1)
            self.depth_camera_info_sub = self.create_subscription(CameraInfo, self._camera_topic('depth/camera_info'), self.depth_camera_info_callback, 1)
        if self.camera_type == "usb_cam":
            self.image_sub = self.create_subscription(Image, self._camera_topic('rgb/image_raw'), self.image_callback, 1)
            self.camera_info_sub = self.create_subscription(CameraInfo, self._camera_topic('rgb/camera_info'), self.camera_info_callback, 1)

        # --- 新底层: 用坐标控制设置初始位置 ---
        wait_start = time.time()
        while self.arm_pub.get_subscription_count() == 0:
            if time.time() - wait_start > 5.0:
                response.success = False
                response.message = "wait ros_robot_controller timeout"
                self.get_logger().error('等待 ros_robot_controller 订阅超时，请确认底层控制节点已启动')
                return response
            self.get_logger().info('等待 ros_robot_controller 订阅...')
            time.sleep(0.5)
        pose = self.calibration_pose
        self.publish_arm(
            pose['x'],
            pose['y'],
            pose['z'],
            pose['pitch'],
            pose['roll'],
            pose['claw'],
            int(pose.get('time_ms', 1500)),
        )
        if self.active_scene_name == scene4_runtime.SCENE4_ID:
            time.sleep(max(0.0, float(pose.get('time_ms', 1500)) / 1000.0))

        self.enter = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit calibration")
            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.destroy_subscription(self.camera_info_sub)
                if self.depth_image_sub is not None:
                    self.destroy_subscription(self.depth_image_sub)
                if self.depth_camera_info_sub is not None:
                    self.destroy_subscription(self.depth_camera_info_sub)
                self.image_sub = None
                self.camera_info_sub = None
                self.depth_image_sub = None
                self.depth_camera_info_sub = None
            except Exception as e:
                self.get_logger().error(str(e))
            self.heart.destroy()
            self.heart = None
            self.enter = False
            self.scene4_calibration_rail_ready = False
        response.success = True
        response.message = "exit"
        return response

    def draw_retangle(self):
        tvec = self.extristric[:1]
        rmat = self.extristric[1:]

        while self.K is None or self.D is None:
            time.sleep(0.5)

        map_corner_order = np.array([
            [self.white_area_height / 2, self.white_area_width / 2, 0.0],
            [-self.white_area_height / 2, self.white_area_width / 2, 0.0],
            [-self.white_area_height / 2, -self.white_area_width / 2, 0.0],
            [self.white_area_height / 2, -self.white_area_width / 2, 0.0],
        ], dtype=np.float64)
        map_center = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        tag_frame_corners = self._map_points_to_tag_frame(map_corner_order)
        tag_frame_center = self._map_points_to_tag_frame(map_center)

        center_imgpts, jac = cv2.projectPoints(map_center, np.array(rmat), np.array(tvec), self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.0)
        imgpts, jac = cv2.projectPoints(map_corner_order, np.array(rmat), np.array(tvec), self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        imgpts, jac = cv2.projectPoints(map_corner_order, np.array(rmat), np.array(tvec), self.K, self.D)
        self.imgpts1 = np.int32(imgpts).reshape(-1, 2)

        imgpts_float = np.array(self.imgpts, dtype=np.float64).reshape((-1, 2))
        imgpts1_float = np.array(self.imgpts1, dtype=np.float64).reshape((-1, 2))
        signed_area = 0.5 * float(np.sum(
            imgpts_float[:, 0] * np.roll(imgpts_float[:, 1], -1)
            - np.roll(imgpts_float[:, 0], -1) * imgpts_float[:, 1]
        ))
        bbox_min = imgpts_float.min(axis=0)
        bbox_max = imgpts_float.max(axis=0)
        self._log_geometry_debug(
            f"[CAL_BOARD_FRAME] yaw_deg={self.scene_tag_yaw_deg:.2f}, "
            f"length_m={self.white_area_height:.4f}, width_m={self.white_area_width:.4f}, "
            f"tag_outer={self.tag_outer_size:.4f}, tag_effective={self.tag_size:.4f}, "
            f"tag_center_map={self._vec_text(self._scene1_tag_center_offset_in_map())}, "
            f"board_map_corners={self._pts_text(map_corner_order)}, "
            f"board_as_tag_frame={self._pts_text(tag_frame_corners)}, "
            f"projected_board_px_RT_LT_LB_RB={self._pts_text(imgpts_float, precision=1)}, "
            f"projected_board_3cm_px_RT_LT_LB_RB={self._pts_text(imgpts1_float, precision=1)}, "
            f"center_px={self._vec_text(self.center_imgpts, precision=1)}, "
            f"bbox_min={self._vec_text(bbox_min, precision=1)}, bbox_max={self._vec_text(bbox_max, precision=1)}, "
            f"signed_area={signed_area:.1f}",
            min_interval=2.0,
            key='board_frame',
        )
        if self.geometry_debug:
            self._log_geometry_debug(
                f"[CAL_FRAME_DEBUG][DRAW] corner_order=RT,LT,LB,RB, "
                f"map_corners={self._pts_text(map_corner_order)}, "
                f"tag_frame_corners={self._pts_text(tag_frame_corners)}, "
                f"projected_0m_px={self._pts_text(imgpts_float, precision=1)}, "
                f"projected_3cm_px={self._pts_text(imgpts1_float, precision=1)}, "
                f"center_px={self._vec_text(self.center_imgpts, precision=1)}, "
                f"bbox_min={self._vec_text(bbox_min, precision=1)}, bbox_max={self._vec_text(bbox_max, precision=1)}, "
                f"signed_area={signed_area:.1f}, "
                f"scene={self.scene_name}, length_x={self.white_area_height:.4f}, width_y={self.white_area_width:.4f}, "
                f"tag_center_offset_map={self._vec_text(self._scene1_tag_center_offset_in_map())}, "
                f"tag_yaw_deg={self.scene_tag_yaw_deg:.2f}, "
                f"tag_frame_center={self._pts_text(tag_frame_center)}",
                min_interval=1.0,
                key='draw',
            )

        return np.vstack([map_corner_order, map_center])

    def _make_plane_roi_from_tag(self, tag, image_shape, source_image_shape=None, prefer_tag_pose=False):
        """基于检测到的 RGB 标签位姿生成深度图白板内部 ROI。"""
        h, w = image_shape[:2]
        if source_image_shape is not None:
            src_h, src_w = source_image_shape[:2]
        else:
            src_h, src_w = h, w
        if src_w <= 0 or src_h <= 0:
            return None

        scale_x = float(w) / float(src_w)
        scale_y = float(h) / float(src_h)

        try:
            margin_x = self.white_area_height * 0.15
            margin_y = self.white_area_width * 0.15
            map_points = np.array([
                [self.white_area_height / 2.0 - margin_x, self.white_area_width / 2.0 - margin_y, 0.0, 1.0],
                [-self.white_area_height / 2.0 + margin_x, self.white_area_width / 2.0 - margin_y, 0.0, 1.0],
                [-self.white_area_height / 2.0 + margin_x, -self.white_area_width / 2.0 + margin_y, 0.0, 1.0],
                [self.white_area_height / 2.0 - margin_x, -self.white_area_width / 2.0 + margin_y, 0.0, 1.0],
            ], dtype=np.float64)
            tag_points = self._map_points_to_tag_frame(map_points[:, :3])
            projection_source = 'current_tag_pose'
            if self.extristric is not None and not prefer_tag_pose:
                rmat = np.array(self.extristric[1:], dtype=np.float64).reshape(3, 3)
                tvec = np.array(self.extristric[:1], dtype=np.float64).reshape(3, 1)
                points_cam = (rmat.dot(map_points[:, :3].T) + tvec).T
                projection_source = 'map_pnp_extristric'
            else:
                tag_pose_cam = common.xyz_rot_to_mat(tag.pose_t, tag.pose_R)
                tag_points_h = np.column_stack([tag_points, np.ones(len(tag_points), dtype=np.float64)])
                points_cam = np.matmul(tag_pose_cam, tag_points_h.T).T[:, :3]
            project_points = points_cam
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            project_scale_x, project_scale_y = scale_x, scale_y
            if (
                self.camera_type == 'aurora'
                and self.plane_roi_depth_frame
                and self.depth_cam_info is not None
                and self.color_to_depth_transform_matrix is not None
            ):
                try:
                    # Project color-frame tag points into the depth optical frame.
                    color_to_depth = np.array(self.color_to_depth_transform_matrix, dtype=np.float64)
                    points_h = np.column_stack([points_cam, np.ones(len(points_cam), dtype=np.float64)])
                    depth_points = np.matmul(color_to_depth, points_h.T).T[:, :3]
                    if np.count_nonzero(depth_points[:, 2] > 1e-6) >= 3:
                        project_points = depth_points
                        k = self.depth_cam_info.k
                        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
                        project_scale_x, project_scale_y = 1.0, 1.0
                        projection_source = f'{projection_source}_depth_frame'
                except Exception as exc:
                    if self.geometry_debug:
                        self.get_logger().warn(f'[PLANE_ROI] color->depth projection failed, fallback to RGB projection: {exc}')
            else:
                projection_source = f'{projection_source}_aligned_depth'

            valid = project_points[:, 2] > 1e-6
            if np.count_nonzero(valid) >= 3:
                project_points = project_points[valid]
                u = fx * project_points[:, 0] / project_points[:, 2] + cx
                v = fy * project_points[:, 1] / project_points[:, 2] + cy
                xs = u * project_scale_x
                ys = v * project_scale_y
                x1 = int(np.floor(np.clip(np.min(xs), 0, w - 1)))
                y1 = int(np.floor(np.clip(np.min(ys), 0, h - 1)))
                x2 = int(np.ceil(np.clip(np.max(xs), 0, w - 1)))
                y2 = int(np.ceil(np.clip(np.max(ys), 0, h - 1)))
                roi_w = x2 - x1 + 1
                roi_h = y2 - y1 + 1
                if roi_w >= 40 and roi_h >= 40:
                    roi = [int(x1), int(y1), int(roi_w), int(roi_h)]
                    if self.geometry_debug:
                        self.get_logger().warn(
                            f'[PLANE_ROI] using projected board inner roi={roi}, '
                            f'source={projection_source}, '
                            f'rgb_size=({src_w},{src_h}), depth_size=({w},{h}), '
                            f'tag_yaw_deg={self.scene_tag_yaw_deg:.2f}, '
                            f'map_inner={self._pts_text(map_points[:, :3])}, '
                            f'tag_inner={self._pts_text(tag_points)}, '
                            f'projected_inner_px={self._pts_text(np.column_stack([xs, ys]), precision=1)}'
                        )
                    return roi
        except Exception as exc:
            if self.geometry_debug:
                self.get_logger().warn(f'[PLANE_ROI] projected board ROI failed, fallback to tag ROI: {exc}')

        try:
            corners = np.array(tag.corners, dtype=np.float64).reshape(-1, 2)
            corners[:, 0] *= scale_x
            corners[:, 1] *= scale_y
            cx = int(np.mean(corners[:, 0]))
            cy = int(np.mean(corners[:, 1]))
        except Exception:
            return None

        # 以标签中心为基准，取较大的矩形区域用于平面拟合
        roi_w = max(220, int(w * 0.35))
        roi_h = max(180, int(h * 0.35))
        x = max(0, cx - roi_w // 2)
        y = max(0, cy - roi_h // 2)
        roi_w = min(roi_w, w - x)
        roi_h = min(roi_h, h - y)
        if roi_w < 40 or roi_h < 40:
            return None
        if self.geometry_debug and (abs(scale_x - 1.0) > 1e-3 or abs(scale_y - 1.0) > 1e-3):
            self.get_logger().info(
                f'[PLANE_ROI] rgb_size=({src_w},{src_h}), depth_size=({w},{h}), '
                f'scale=({scale_x:.4f},{scale_y:.4f}), scaled_tag_center=({cx},{cy})'
            )
        return [int(x), int(y), int(roi_w), int(roi_h)]

    def search_plane(self, depth_image, roi_rect=None):
        p = self.depth_cam_info.p
        fx = p[0]
        fy = p[5]
        cx = p[2]
        cy = p[6]
        height = self.depth_cam_info.height
        width = self.depth_cam_info.width
        camera_intrinsics = [fx, fy, cx, cy]
        searcher = search_plane.SearchPlane(width, height, camera_intrinsics, roi_rect=roi_rect)
        a, m, s = searcher.find_plane(depth_image)
        if m is None:
            if self.geometry_debug:
                self.get_logger().warn(f'[PLANE_ROI] plane fit failed, roi={roi_rect}')
            return False
        self.plane = np.array(m, dtype=np.float64)
        if self.plane[2] > 0:
            self.plane = -self.plane
        xy_tilt = float(np.linalg.norm(self.plane[:2]))
        if self.geometry_debug:
            if roi_rect is not None:
                self.get_logger().info(f'[PLANE_ROI] using roi={roi_rect}, plane={self.plane}, xy_tilt={xy_tilt:.4f}')
            else:
                self.get_logger().info(f'[PLANE_ROI] using full image, plane={self.plane}, xy_tilt={xy_tilt:.4f}')
        if self.geometry_debug and xy_tilt > 0.5:
            self.get_logger().warn(
                f'[PLANE_ROI] fitted plane is steep, check depth/RGB alignment or ROI contamination: '
                f'roi={roi_rect}, plane={self.plane}, xy_tilt={xy_tilt:.4f}'
            )
        return True

    def image_processing(self):
        while self.running:
            if self.enter:
                rgb_image = self.image_queue.get(block=True)
                if self.camera_type == 'aurora':
                    depth_image = self.depth_image_queue.get(block=True)
                result_image = np.copy(rgb_image)
                if self.K is not None:
                    tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), True, (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]), self.tag_size)
                    result_image = common.draw_tags(result_image, tags)

                    if self.calibration_step == 1:
                        expected_ids = self._expected_calibration_tag_ids()
                        if len(tags) == 1 and (tags[0].tag_id in expected_ids):
                            self.err_msg = None
                            if len(self.tags) > 0:
                                if common.distance(self.tags[-1].pose_t, tags[0].pose_t) < 0.003:
                                    self.tags.append(tags[0])
                                else:
                                    self.tags = []
                            else:
                                self.tags.append(tags[0])
                            if len(self.tags) >= 10:
                                if self.verbose_logs:
                                    # print("收集完成")
                                    pass
                                plane_ok = True
                                if self.camera_type == 'aurora':
                                    if self.depth_cam_info is not None and self.depth_cam_info.k is not None:
                                        current_extristric = self._solve_map_extristric_from_tags(self.tags)
                                        if current_extristric is not None:
                                            self.extristric = np.array(current_extristric, dtype=np.float64)
                                        elif self.geometry_debug:
                                            self.get_logger().warn('[PLANE_ROI] current solvePnP failed before plane fit')
                                        roi_rect = self._make_plane_roi_from_tag(
                                            tags[0], depth_image.shape, rgb_image.shape, prefer_tag_pose=False)
                                        plane_ok = self.search_plane(depth_image, roi_rect=roi_rect)
                                    else:
                                        plane_ok = False
                                if plane_ok:
                                    self.calibration_step = 2
                                else:
                                    self.tags = []
                                    self.err_msg = 'Plane fit failed, please check depth image and tag ROI'
                        else:
                            self.tags = []
                            if self.err_msg is None:
                                expected_text = '/'.join(str(x) for x in sorted(expected_ids))
                                self.err_msg = f"Please make sure there is only one tag in the;screen and the tag id is {expected_text}"

                    if self.extristric is not None:
                        tag_local_corners = np.array([
                            (-self.tag_size / 2, -self.tag_size / 2, 0),
                            ( self.tag_size / 2, -self.tag_size / 2, 0),
                            ( self.tag_size / 2,  self.tag_size / 2, 0),
                            (-self.tag_size / 2,  self.tag_size / 2, 0),
                        ], dtype=np.float64)
                        tag_center_offset = self._scene1_tag_center_offset_in_map()
                        world_points = np.array([self._scene1_tag_point_to_map(p) for p in tag_local_corners], dtype=np.float64)
                        image_points, _ = cv2.projectPoints(world_points, self.extristric[1:].reshape(3, 3), self.extristric[:1], self.K, self.D)
                        image_points = image_points.astype(np.int32).reshape((-1, 2)).tolist()
                        for p in image_points:
                            cv2.circle(result_image, tuple(p), 3, (0, 0, 0), -1)

                if self.err_msg is not None:
                    self.get_logger().info(str(self.err_msg))
                    err_msg = self.err_msg.split(';')
                    for i, m in enumerate(err_msg):
                        cv2.putText(result_image, m, (5, 50 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 6)
                        cv2.putText(result_image, m, (5, 50 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                if self.calibration_step != 0:
                    if self.calibration_step == 20:
                        msg = "Calibration finished!"
                        self.draw_retangle()
                        cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA)
                    else:
                        msg = "Calibrating..."
                    cv2.putText(result_image, msg, (5, result_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 6)
                    cv2.putText(result_image, msg, (5, result_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                # Only draw the board plane. The +3cm projection is useful for
                # debugging object height, but it makes the board boundary look
                # too large during calibration.
                if self.imgpts is not None:
                    cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA)
                    for label, pt in zip(('RT', 'LT', 'LB', 'RB'), self.imgpts):
                        cv2.putText(result_image, label, (int(pt[0]) + 4, int(pt[1]) - 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                        cv2.putText(result_image, label, (int(pt[0]) + 4, int(pt[1]) - 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

                self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
            else:
                time.sleep(0.1)

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)

    def depth_camera_info_callback(self, msg):
        self.depth_cam_info = msg

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)

    def depth_image_callback(self, ros_depth_image):
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16,
                                 buffer=ros_depth_image.data)
        if self.depth_image_queue.full():
            self.depth_image_queue.get()
        self.depth_image_queue.put(depth_image)


def main():
    rclpy.init()
    node = CalibrationNode('calibration')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()


if __name__ == "__main__":
    main()
