#!/usr/bin/env python3
# coding: utf8
#目标分拣
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
from app import scene4_runtime, scene_play_registry
from dt_apriltags import Detector
from interfaces.srv import SetStringBool
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from app import calibrated_pose, sorting_coordinator
from app.play_pose import get_use_scene_pose
from app.utils import calculate_grasp_yaw, position_change_detect, pick_and_place, image_process, distortion_inverse_map, utils

DEFAULT_SCENE_ID = 'scene_1'
COLOR_TARGET_LABELS = ('yellow', 'red', 'green', 'blue')
COLOR_CLAW_GRAB_ANGLE = 0.0
COLOR_OBJECT_HEIGHT_M = 0.03
GARBAGE_OBJECT_HEIGHT_M = 0.04
DEFAULT_PLACE_POLICY = {
    'only_left_y_positive': True,
    'min_place_z': 0.015,
}
DEFAULT_SCENE_PLACE_TARGETS = {
    'yellow': [0.14, 0.135, 0.025],
    'red': [-0.03, 0.135, 0.025],
    'green': [0.02, 0.135, 0.025],
    'blue': [0.07, 0.135, 0.025],
    'tag1': [0.14, -0.135, 0.025],
    'tag2': [0.07, -0.135, 0.025],
    'tag3': [0.07, -0.135, 0.025],
}

class ObjectSortingNode(Node):
    place_position = {
                    'yellow': [0.14, 0.135, 0.025],
                    'red': [-0.03, 0.135, 0.025],
                    'green': [0.02, 0.135, 0.025],
                    'blue': [0.07, 0.135, 0.025],
                    'tag1': [0.14, -0.135, 0.025],
                    'tag2': [0.07, -0.135, 0.025],
                    'tag3': [0.07, -0.135, 0.025],
                }

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        # proto_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/deploy.prototxt'
        # model_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/hed_pretrained_bsds.caffemodel'
        # self.image_process = image_process.GetObjectSurface(proto_path, model_path)
        self.image_process = image_process.GetObjectSurface()
        self.at_detector = Detector(searchpath=['apriltags'],
               families='tag36h11',
               nthreads=4,
               quad_decimate=1.0,
               quad_sigma=0.0,
               refine_edges=1,
               decode_sharpening=0.25,
               debug=0)
        self.lock = threading.RLock()
        self.fps = fps.FPS()    # 帧率统计器(frame rate counter)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.image_queue = queue.Queue(maxsize=2)
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.camera_type = os.environ.get('CAMERA_TYPE', '').lower()
        self.scene_config_path = scene4_runtime.scene_config_path()
        self.config_path = os.path.dirname(self.scene_config_path) + "/"
        _app_config = "/home/ubuntu/ros2_ws/src/app/config/"
        self.camera_info_path = "/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml"
        self.lab_config_file = os.path.join(_app_config, "lab_config.yaml")
        self.lab_config_mtime = None
        self.data = {}
        self.lab_data = {}
        self._load_lab_config(force=True)
        self.play_config_path = self.get_string_param('play_config_path', '')
        self.home_pose = self._load_home_pose_from_scene()

        self.tag_size = 0.025
        self.min_area = 500
        self.max_area = 7000
        self.target_labels = {
            "yellow": False,
            "red": False,
            "green": False,
            "blue": False,
            "tag1": False,
            "tag2": False,
            "tag3": False,
        }
        self.running = True
        # 初始化基本参数
        self._init_parameters()

        # sub
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.scene4_stepper_client = scene4_runtime.create_stepper_position_client(self, self.timer_cb_group)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(
            GetArmFullState,
            '/ros_robot_controller/arm/get_full_state',
            callback_group=self.timer_cb_group,
        )
        self.controller_init_client = self.create_client(
            Trigger,
            '/controller_manager/init_finish',
            callback_group=self.timer_cb_group,
        )
        self.kinematics_init_client = self.create_client(
            Trigger,
            '/kinematics/init_finish',
            callback_group=self.timer_cb_group,
        )
        self.scene_runtime_prepare_client = self.create_client(
            Trigger,
            '/ros_robot_controller/scene_runtime/prepare',
            callback_group=self.timer_cb_group,
        )
        # services and topics
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_sorting_srv = self.create_service(SetBool, '~/enable_sorting', self.enable_sorting_srv_callback)
        self.set_target_srv = self.create_service(SetStringBool, '~/set_target', self.set_target_srv_callback)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_string_param(self, name, default=''):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

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
        self.known_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'pitch': float(pitch),
            'roll': float(roll),
            'claw': float(claw),
            'yaw': float(getattr(self, 'known_pose', {}).get('yaw', 0.0)),
        }

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

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _init_parameters(self):
        self.heart = None
        self.endpoint = None
        self.target_miss_count = 0
        self.transport_info = None
        self.intrinsic = None
        self.distortion = None
        self.current_pose = None
        home_pose = getattr(self, 'home_pose', {'x': 110.0, 'y': 0.0, 'z': 220.0, 'pitch': -90.0, 'roll': 0.0, 'claw': 0.0})
        self.known_pose = {
            'x': float(home_pose.get('x', 110.0)),
            'y': float(home_pose.get('y', 0.0)),
            'z': float(home_pose.get('z', 220.0)),
            'pitch': float(home_pose.get('pitch', -90.0)),
            'roll': float(home_pose.get('roll', 0.0)),
            'claw': float(home_pose.get('claw', 0.0)),
            'yaw': 0.0,
        }
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
        self.depth_image_sub = None
        self.depth_image = None
        self.depth_camera_info_sub = None
        self.depth_camera_info = None
        self.camera_info_sub = None
        self.hand2cam_tf_matrix = None
        self.depth_offset = (0.0, 0.0, 0.0)
        self.depth_scale = (1.0, 1.0, 1.0)
        self.kinematics_offset = (0.0, 0.0, 0.0)
        self.kinematics_scale = (1.0, 1.0, 1.0)
        self.active_scene_name = DEFAULT_SCENE_ID
        self.scene4_active = False
        self.scene4_pick_zone = scene4_runtime.SCENE4_PICK_ZONE_LOWER
        self.scene4_pick_cfg = scene4_runtime.scene4_pick_config({})
        self.scene4_upper_current_rail = None
        self.scene4_upper_last_motion_time = 0.0
        self.scene4_upper_center_count = 0
        self.scene4_upper_last_debug_time = 0.0
        self.scene4_upper_last_rgbd_debug_time = 0.0
        self.scene4_lower_last_debug_time = 0.0
        self.scene4_color_debug_last_time = 0.0
        self.sort_claim_target = None
        self._load_lab_config(force=True)

    def _release_sort_claim(self):
        if self.sort_claim_target is not None:
            sorting_coordinator.release_claim(sorting_coordinator.COLOR_GROUP, self.sort_claim_target)
            self.sort_claim_target = None

    def _reset_tracking_state(self):
        self._release_sort_claim()
        self.target_miss_count = 0
        self.transport_info = None
        self.start_transport = False
        self.count_move = 0
        self.count_still = 0
        self.target = None
        self.last_position = None
        self.last_object_info_list = None
        self.scene4_upper_center_count = 0

    def _enabled_targets(self):
        return [key for key, value in self.target_labels.items() if value]

    def _is_scene4_lower_mode(self):
        return self.enter and self.scene4_active and self.scene4_pick_zone == scene4_runtime.SCENE4_PICK_ZONE_LOWER

    def _filter_scene4_lower_targets(self, target_info):
        lower = self.scene4_pick_cfg.get(scene4_runtime.SCENE4_PICK_ZONE_LOWER, {})
        detection = lower.get('detection', {})
        min_v = int(detection.get('min_v', 0))
        max_v = int(detection.get('max_v', 1080))
        kept = []
        dropped = []
        for target in target_info:
            v = int(target[2][1])
            if min_v <= v <= max_v:
                kept.append(target)
            else:
                dropped.append(target)
        if dropped:
            self.get_logger().info(
                '[Scene4Lower] filtered by lower v-range: '
                f'range=({min_v},{max_v}), '
                f'dropped={[f"{t[0]}#{t[1]}@{t[2]}" for t in dropped]}'
            )
        return kept

    def _load_lab_config(self, force=False):
        try:
            mtime = os.path.getmtime(self.lab_config_file)
        except OSError as exc:
            self.get_logger().warn(f'读取 LAB 阈值失败: {exc}')
            return False
        if not force and self.lab_config_mtime is not None and mtime <= self.lab_config_mtime:
            return False
        data = common.get_yaml_data(self.lab_config_file) or {}
        params = data.get('/**', {}).get('ros__parameters', {})
        if 'color_range_list' not in params:
            self.get_logger().warn('LAB 阈值文件缺少 color_range_list')
            return False
        self.data = data
        self.lab_data = params
        self.lab_config_mtime = mtime
        return True

    def _load_scene4_rgbd_parameters(self):
        try:
            camera_info = common.get_yaml_data(self.camera_info_path) or {}
            matrix = camera_info.get('hand2cam_tf_matrix')
            if matrix is not None:
                self.hand2cam_tf_matrix = np.array(matrix, dtype=np.float64)
                self.get_logger().info('[Scene4UpperRGBD] hand2cam_tf_matrix loaded')
            else:
                self.get_logger().warn(f'[Scene4UpperRGBD] hand2cam_tf_matrix missing in {self.camera_info_path}')
        except Exception as exc:
            self.hand2cam_tf_matrix = None
            self.get_logger().warn(f'[Scene4UpperRGBD] load hand2cam_tf_matrix failed: {exc}')

        try:
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file)) or {}
            depth_cfg = calibration.get('depth', {})
            kinematics_cfg = calibration.get('kinematics', {})
            self.depth_offset = tuple(float(v) for v in depth_cfg.get('offset', self.depth_offset))
            self.depth_scale = tuple(float(v) for v in depth_cfg.get('scale', self.depth_scale))
            self.kinematics_offset = tuple(float(v) for v in kinematics_cfg.get('offset', self.kinematics_offset))
            self.kinematics_scale = tuple(float(v) for v in kinematics_cfg.get('scale', self.kinematics_scale))
            self.get_logger().info(
                '[Scene4UpperRGBD] calibration loaded: '
                f'depth_offset={self.depth_offset}, depth_scale={self.depth_scale}, '
                f'kinematics_offset={self.kinematics_offset}, kinematics_scale={self.kinematics_scale}'
            )
        except Exception as exc:
            self.get_logger().warn(f'[Scene4UpperRGBD] load calibration.yaml failed: {exc}')

    def _apply_depth_calibration(self, position):
        calibration = {
            'depth': {
                'offset': self.depth_offset,
                'scale': self.depth_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'depth').tolist()

    def _apply_kinematics_calibration(self, position):
        calibration = {
            'kinematics': {
                'offset': self.kinematics_offset,
                'scale': self.kinematics_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'kinematics').tolist()

    def _request_real_pose_snapshot(self, timeout_sec=0.2):
        if not hasattr(self, 'arm_state_client') or not self.arm_state_client.wait_for_service(timeout_sec=0.05):
            return None
        future = self.arm_state_client.call_async(GetArmFullState.Request())
        end_time = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end_time:
            if future.done():
                try:
                    response = future.result()
                except Exception:
                    return None
                if response is None or not bool(getattr(response, 'success', False)):
                    return None
                return {
                    'x': float(response.x),
                    'y': float(response.y),
                    'z': float(response.z),
                    'pitch': float(response.pitch),
                    'roll': float(response.roll),
                    'claw': float(response.claw),
                    'yaw': float(response.yaw),
                }
            time.sleep(0.01)
        return None

    def _get_pose_snapshot(self):
        return self.current_pose or self._request_real_pose_snapshot() or self.known_pose

    def _get_endpoint_matrix(self):
        pose = self._get_pose_snapshot()
        x_m = float(pose['x']) / 1000.0
        y_m = float(pose['y']) / 1000.0
        z_m = float(pose['z']) / 1000.0
        yaw_deg = float(pose.get('yaw', math.degrees(math.atan2(y_m, x_m if abs(x_m) > 1e-6 else 1e-6))))
        pitch_deg = float(pose['pitch'])
        roll_deg = float(pose['roll'])

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
        return endpoint, pose

    def _load_active_scene_config(self):
        cfg = {}
        try:
            cfg = common.get_yaml_data(self.scene_config_path) or {}
        except Exception:
            cfg = {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            scenes = {DEFAULT_SCENE_ID: {}}
        scene_name = scene_play_registry.active_scene_from_env()
        if scene_name not in scenes:
            scene_name = str(cfg.get('current_scene', DEFAULT_SCENE_ID))
        if scene_name not in scenes:
            scene_name = next(iter(scenes.keys()))
        scene_cfg = scenes.get(scene_name, {})
        if not isinstance(scene_cfg, dict):
            scene_cfg = {}
        play_cfg = scene_play_registry.load_play_config(scene_name, self.play_config_path or None)
        return scene_name, scene_play_registry.merge_play_into_scene(scene_name, scene_cfg, play_cfg)

    def _load_scene_config(self):
        return self._load_active_scene_config()[1]

    def _load_home_pose_from_scene(self):
        default_pose = {
            'x': 110.0,
            'y': 0.0,
            'z': 220.0,
            'pitch': -90.0,
            'roll': 0.0,
            'claw': 0.0,
        }
        if not get_use_scene_pose(self):
            return default_pose
        scene_cfg = self._load_scene_config()
        if isinstance(scene_cfg, dict) and 'scene4_pick' in scene_cfg:
            pose = scene4_runtime.scene4_observation_pose(scene_cfg)
            return {
                'x': float(pose.get('x', default_pose['x'])),
                'y': float(pose.get('y', default_pose['y'])),
                'z': float(pose.get('z', default_pose['z'])),
                'pitch': float(pose.get('pitch', default_pose['pitch'])),
                'roll': float(pose.get('roll', default_pose['roll'])),
                'claw': float(pose.get('claw', default_pose['claw'])),
            }
        home = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
        return {
            'x': float(home.get('x', default_pose['x'])),
            'y': float(home.get('y', default_pose['y'])),
            'z': float(home.get('z', default_pose['z'])),
            'pitch': float(home.get('pitch', default_pose['pitch'])),
            'roll': float(home.get('roll', default_pose['roll'])),
            'claw': float(home.get('claw', default_pose['claw'])),
        }

    def _load_scene4_pick_cfg(self):
        scene_cfg = self._load_scene_config()
        return scene4_runtime.scene4_pick_config(scene_cfg)

    def _scene4_upper_roi(self, pick_cfg=None):
        pick_cfg = pick_cfg or self._load_scene4_pick_cfg()
        roi_cfg = pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]['roi']
        x = max(0, int(roi_cfg.get('x', 0)))
        y = max(0, int(roi_cfg.get('y', 0)))
        w = max(1, int(roi_cfg.get('w', 640)))
        h = max(1, int(roi_cfg.get('h', 260)))
        return np.array([y, y + h, x, x + w], dtype=np.int32)

    def _configure_scene4_pick_mode(self):
        scene_name, scene_cfg = self._load_active_scene_config()
        self.active_scene_name = scene_name
        self.scene4_active = scene_name == scene4_runtime.SCENE4_ID
        self.scene4_pick_cfg = scene4_runtime.scene4_pick_config(scene_cfg if self.scene4_active else {})
        self.scene4_pick_zone = self.scene4_pick_cfg.get(
            'active_zone',
            scene4_runtime.SCENE4_PICK_ZONE_LOWER,
        )
        self.scene4_upper_current_rail = None
        self.start_get_roi = True
        if self.scene4_active:
            pose = scene4_runtime.scene4_observation_pose(scene_cfg)
            rail = scene4_runtime.scene4_rail_config(scene_cfg)
            self.get_logger().info(
                '[Scene4Lower] enter mode: '
                f'rail_calibration={rail["calibration_abs_position"]}, '
                f'view_pose=({pose["x"]:.1f},{pose["y"]:.1f},{pose["z"]:.1f},'
                f'{pose["pitch"]:.1f}), enabled_targets={self._enabled_targets()}, '
                f'scene_config={self.scene_config_path}, config_path={self.config_path}'
            )
            scene4_runtime.move_scene4_rail(
                self,
                self.scene4_stepper_client,
                "calibration",
                scene_path=self.scene_config_path,
                logger=self.get_logger(),
                reset_first=False,
            )
            return dict(pose)
        return self._load_home_pose_from_scene()

    def resolve_place_position(self, target_key):
        scene_name, scene_cfg = self._load_active_scene_config()
        if scene_name == scene4_runtime.SCENE4_ID:
            fixed_position = scene4_runtime.scene4_fixed_place_position(scene_cfg, target_key)
            if fixed_position is not None:
                return scene_play_registry.apply_global_place_offset(
                    fixed_position,
                    self.scene_config_path,
                )

        place_targets = scene_cfg.get('place_targets', {}) if isinstance(scene_cfg.get('place_targets'), dict) else {}
        raw = place_targets.get(target_key, self.place_position.get(target_key, DEFAULT_SCENE_PLACE_TARGETS.get(target_key)))
        if raw is None:
            return None
        try:
            pos = [float(raw[0]), float(raw[1]), float(raw[2])]
        except Exception:
            fallback = self.place_position.get(target_key, DEFAULT_SCENE_PLACE_TARGETS.get(target_key))
            if fallback is None:
                return None
            pos = [float(fallback[0]), float(fallback[1]), float(fallback[2])]

        policy = DEFAULT_PLACE_POLICY.copy()
        if isinstance(scene_cfg.get('place_policy'), dict):
            policy.update(scene_cfg['place_policy'])
        if bool(policy.get('only_left_y_positive', False)) and pos[1] < 0.0:
            pos[1] = abs(pos[1])
        try:
            min_place_z = float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z']))
            if pos[2] < min_place_z:
                pos[2] = min_place_z
        except Exception:
            pass
        return scene_play_registry.apply_global_place_offset(pos, self.scene_config_path)

    def resolve_place_pitch(self, target_key, default_pitch=80.0):
        _, scene_cfg = self._load_active_scene_config()
        return scene_play_registry.resolve_place_pitch(scene_cfg, target_key, default_pitch)

    def resolve_place_roll(self, target_key):
        _, scene_cfg = self._load_active_scene_config()
        return scene_play_registry.resolve_place_roll(scene_cfg, target_key)

    def _scene3_direct_place(self, target_key, claw_hold):
        if getattr(pick_and_place, 'stop', False):
            self.get_logger().warn(f'[Scene3Place] skipped (stop=True): target={target_key}')
            return False
        scene_name, scene_cfg = self._load_active_scene_config()
        self.get_logger().info(f'[Scene3Place] scene_name={scene_name}, target={target_key}, claw_hold={claw_hold}')
        place_targets = scene_cfg.get('place_targets', {})
        raw = place_targets.get(target_key)
        self.get_logger().info(f'[Scene3Place] raw yaml place_target[{target_key}] = {raw}')
        if raw is None:
            self.get_logger().warn(f'[Scene3Place] unknown target: {target_key}, available={list(place_targets.keys())}')
            return False
        try:
            x_mm = float(raw[0]) * 1000.0
            y_mm = float(raw[1]) * 1000.0
            z_mm = float(raw[2]) * 1000.0
        except Exception as e:
            self.get_logger().error(f'[Scene3Place] bad coordinate in yaml: {raw}, err={e}')
            return False
        pitch_deg = scene_play_registry.resolve_place_pitch(scene_cfg, target_key, -90.0)
        roll_raw = scene_play_registry.resolve_place_roll(scene_cfg, target_key)
        roll_deg = float(roll_raw) if roll_raw is not None else 0.0
        self.get_logger().info(
            f'[Scene3Place] RESOLVED: target={target_key}, '
            f'x={x_mm:.1f}mm y={y_mm:.1f}mm z={z_mm:.1f}mm '
            f'pitch={pitch_deg:.1f}deg roll_raw={roll_raw} roll={roll_deg:.1f}deg '
            f'claw_hold={claw_hold}'
        )
        self.get_logger().info(f'[Scene3Place] Step1: move to place pos, roll=0')
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, 0.0, claw_hold, 1500)
        time.sleep(2.0)
        if getattr(pick_and_place, 'stop', False):
            self.get_logger().warn(f'[Scene3Place] interrupted after Step1')
            return False
        self.get_logger().info(f'[Scene3Place] Step2: open claw')
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, 0.0, pick_and_place.CLAW_OPEN, 400)
        time.sleep(0.5)
        self.get_logger().info(f'[Scene3Place] Step3: lift')
        self.publish_arm(x_mm, y_mm, z_mm + 50.0, pitch_deg, 0.0, pick_and_place.CLAW_OPEN, 800)
        time.sleep(1.0)
        self.get_logger().info(f'[Scene3Place] done: target={target_key}')
        return True

    def _resolve_scene4_shelf_place(self, target_key, place_position):
        if not self._is_scene4_lower_mode():
            return None
        try:
            return scene4_runtime.scene4_shelf_place(self._load_scene_config(), target_key, place_position)
        except Exception as exc:
            self.get_logger().warn(f'[Scene4Place] resolve shelf place failed: target={target_key}, error={exc}')
            return None

    def _execute_scene4_shelf_place(self, target_key, shelf_place):
        rail_position = int(shelf_place['rail_position'])
        pose = dict(shelf_place['pose'])
        approach_pose = shelf_place.get('approach_pose')
        if approach_pose is not None:
            approach_pose = dict(approach_pose)
        move_ms = int(pose.get('time_ms', 2000))
        destination = shelf_place.get('destination', 'shelf')
        target_position = shelf_place.get('target_position', [0.0, 0.0, 0.0])
        claw_hold = float(shelf_place.get('claw_hold', pick_and_place.CLAW_GRAB))

        self.get_logger().info(
            '[Scene4Shelf] move rail to upper place: '
            f'target={target_key}, destination={destination}, rail={rail_position}, '
            f'host_pos=({target_position[0]:.3f},{target_position[1]:.3f},{target_position[2]:.3f}), '
            f'pose=({pose["x"]:.1f},{pose["y"]:.1f},{pose["z"]:.1f},{pose["pitch"]:.1f}), '
            f'claw_hold={claw_hold:.1f}'
        )
        claw_open = pick_and_place.CLAW_OPEN
        scene4_runtime.publish_scene4_transfer_pose(self.publish_arm, claw_hold, self._load_scene_config())
        if not scene4_runtime.move_scene4_rail_to_position(
            self,
            self.scene4_stepper_client,
            rail_position,
            scene_path=self.scene_config_path,
            logger=self.get_logger(),
            reset_first=False,
        ):
            self.get_logger().warn('[Scene4Shelf] move rail to upper place failed, returning to observation')
            self.return_to_observation_pose(True)
            return False

        if approach_pose is not None:
            approach_ms = int(approach_pose.get('time_ms', move_ms))
            self.publish_arm(
                approach_pose['x'],
                approach_pose['y'],
                approach_pose['z'],
                approach_pose['pitch'],
                approach_pose['roll'],
                claw_hold,
                approach_ms,
            )
            time.sleep(max(0.5, approach_ms / 1000.0))

        self.publish_arm(pose['x'], pose['y'], pose['z'], pose['pitch'], pose['roll'], claw_hold, move_ms)
        time.sleep(max(0.5, move_ms / 1000.0))
        self.publish_arm(pose['x'], pose['y'], pose['z'], pose['pitch'], pose['roll'], claw_open, 500)
        time.sleep(0.8)
        if approach_pose is not None:
            retract_ms = int(approach_pose.get('time_ms', move_ms))
            self.publish_arm(
                approach_pose['x'],
                approach_pose['y'],
                approach_pose['z'],
                approach_pose['pitch'],
                approach_pose['roll'],
                claw_open,
                retract_ms,
            )
            time.sleep(max(0.5, retract_ms / 1000.0))
        scene4_runtime.publish_scene4_transfer_pose(self.publish_arm, claw_open, self._load_scene_config())
        self.return_to_observation_pose(False)
        return True

    def init_process(self):
        self.timer.cancel()

        self.wait_for_motion_ready()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        if self.get_parameter('start').value:
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            req = SetBool.Request()
            req.data = True
            res = SetBool.Response()
            self.enable_sorting_srv_callback(req, res)

        if not self.get_parameter('broadcast').value:
            target_list = ["yellow", "red", "green", "blue"]
            req = SetStringBool.Request()
            req.data_bool = True
            for i in target_list:
                req.data_str = i
                res = SetBool.Response()
                self.set_target_srv_callback(req, res)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def wait_for_motion_ready(self):
        self.get_logger().info('等待底层初始化完成...')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.05)

    def prepare_scene_runtime(self, timeout_sec=40.0):
        label = '/ros_robot_controller/scene_runtime/prepare'
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

    def go_home(self, interrupt=True):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        if self.target is not None:
            if self.target[0] in ["bule", "tag1"]:
                t = 1.6
        elif self.target is not None:
            if self.target[0] in ["green", "tag2"]:
                t = 1.3
        elif self.target is not None:
            if self.target[0] in ["red", "tag3"]:
                t = 1.0
        else :
            t = 1.0
        if interrupt:
            self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 500)
            time.sleep(0.5)

        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1)

        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1)

    def return_to_observation_pose(self, interrupt=True):
        returned = scene4_runtime.return_scene4_to_calibration_pose(
            self,
            self.scene4_stepper_client,
            self.publish_arm,
            scene_path=self.scene_config_path,
            logger=self.get_logger(),
        )
        if returned is True:
            return
        self.go_home(interrupt)

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]  # 取第一行
        rmat = extristric[1:]  # 取后面三行

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # 裁切出ROI区域(crop RIO region)
        x_min = min(imgpts, key=lambda p: p[0])[0] # x轴最小值(the minimum value of X-axis)
        x_max = max(imgpts, key=lambda p: p[0])[0] # x轴最大值(the maximum value of X-axis)
        y_min = min(imgpts, key=lambda p: p[1])[1] # y轴最小值(the minimum value of Y-axis)
        y_max = max(imgpts, key=lambda p: p[1])[1] # y轴最大值(the maximum value of Y-axis)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

        self.roi = roi

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter object sorting")
        ok, msg = self.prepare_scene_runtime()
        if not ok:
            response.success = False
            response.message = msg
            return response
        self._init_parameters()
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        for k, v in self.target_labels.items():
            self.target_labels[k] = False
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.home_pose = self._configure_scene4_pick_mode()
        scene_name, scene_cfg = self._load_active_scene_config()
        sorting_coordinator.start_session(
            scene_name,
            sorting_coordinator.priority_from_scene_config(scene_cfg),
        )
        hp = self.home_pose
        move_ms = int(hp.get('time_ms', 2000))
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], move_ms)
        if self.scene4_active:
            time.sleep(max(0.0, move_ms / 1000.0) + 0.3)

        self.enter = True

        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit  object sorting")
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            if self.depth_image_sub is not None:
                self.destroy_subscription(self.depth_image_sub)
                self.depth_image_sub = None
            if self.depth_camera_info_sub is not None:
                self.destroy_subscription(self.depth_camera_info_sub)
                self.depth_camera_info_sub = None
            if self.camera_info_sub is not None:
                self.destroy_subscription(self.camera_info_sub)
                self.camera_info_sub = None
            self.heart.destroy()
            self.enter = False
            self.start_transport = False
            pick_and_place.interrupt(True)

        response.success = True
        response.message = "start"
        return response

    def enable_sorting_srv_callback(self, request, response):
        if request.data:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start  object sorting')
            pick_and_place.interrupt(False)
            self.enable_sorting = True
            if self._is_scene4_lower_mode():
                self.get_logger().info(
                    f'[Scene4Lower] sorting enabled: enabled_targets={self._enabled_targets()}, roi={list(self.roi)}'
                )
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop  object sorting')
            pick_and_place.interrupt(True)
            self.enable_sorting = False
            self._reset_tracking_state()

        response.success = True
        response.message = "start"
        return response

    def set_target_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32mset target %s %s\033[0m' % (str(request.data_str), str(request.data_bool)))
        if request.data_str in self.target_labels:
            self.target_labels[request.data_str] = request.data_bool
            if self._is_scene4_lower_mode() and (request.data_bool or self.enable_sorting):
                self._reset_tracking_state()
                self.get_logger().info(
                    f'[Scene4Lower] target updated: {request.data_str}={request.data_bool}, '
                    f'enabled_targets={self._enabled_targets()}'
                )

        response.success = True
        response.message = "start"
        return response

    def get_object_pixel_position(self, bgr_image, roi):
        self._load_lab_config()
        target_info = []
        draw_image = bgr_image.copy()
        roi_raw = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        roi_img = self.image_process.get_top_surface(roi_raw)
        # cv2.imshow('roi_img', roi_img)
        image_lab = cv2.cvtColor(cv2.GaussianBlur(roi_img, (3, 3), 3), cv2.COLOR_BGR2LAB)  # 转换到 LAB 空间(convert to LAB space)
        raw_image_lab = cv2.cvtColor(cv2.GaussianBlur(roi_raw, (3, 3), 3), cv2.COLOR_BGR2LAB)
        debug_items = []

        def filtered_contours(mask):
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour_areas = [(math.fabs(cv2.contourArea(c)), c) for c in contours]
            kept = [(area, c) for area, c in contour_areas if self.min_area <= area <= self.max_area]
            return kept, contour_areas

        for i in ['yellow', 'red', 'green', 'blue']:
            if i not in self.lab_data.get('color_range_list', {}):
                continue
            index = 0
            mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][i]['min']), tuple(self.lab_data['color_range_list'][i]['max']))  # 二值化
            kept, contour_areas = filtered_contours(mask)
            mask_pixels = int(cv2.countNonZero(mask))
            source = 'surface'
            if not kept:
                raw_mask = cv2.inRange(raw_image_lab, tuple(self.lab_data['color_range_list'][i]['min']), tuple(self.lab_data['color_range_list'][i]['max']))
                raw_kept, raw_contour_areas = filtered_contours(raw_mask)
                if raw_kept:
                    kept = raw_kept
                    contour_areas = raw_contour_areas
                    mask_pixels = int(cv2.countNonZero(raw_mask))
                    source = 'raw'
            if self._is_scene4_lower_mode():
                largest = max([area for area, _c in contour_areas], default=0.0)
                debug_items.append(
                    f'{i}:src={source},mask={mask_pixels},contours={len(contour_areas)},'
                    f'kept={len(kept)},max_area={largest:.0f}'
                )
            for _area, c in kept:
                rect = cv2.minAreaRect(c)  # 获取最小外接矩形(obtain the minimum bounding rectangle)
                (center_x, center_y), _ = cv2.minEnclosingCircle(c)
                center_x, center_y = roi[2] + center_x, roi[0] + center_y
                # cv2.circle(draw_image, (int(center_x), int(center_y)), 8, (0, 0, 0), -1)
                corners = list(map(lambda p: (roi[2] + p[0], roi[0] + p[1]), cv2.boxPoints(rect)))  # 获取最小外接矩形的四个角点, 转换回原始图的坐标(obtain the four corner points of the minimum rectangle and convert to the coordinates of the original image)
                cv2.drawContours(draw_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓(draw rectangle contour)

                # cv2.line(draw_image, (int(center_x), 0), (int(center_x), 480), (255, 255, 0), 2, cv2.LINE_AA)
                index += 1  # 序号递增(incremental numbering)
                angle = int(round(rect[2]))
                rect_size = rect[1]
                target_info.append([i, index, (int(center_x), int(center_y)), (int(rect_size[0]), int(rect_size[1])), angle])
        if self._is_scene4_lower_mode() and debug_items:
            now = time.time()
            if now - self.scene4_color_debug_last_time > 1.0:
                self.scene4_color_debug_last_time = now
                self.get_logger().info(
                    '[Scene4ColorDetect] '
                    f'roi={list(map(int, roi))}, area_range=({self.min_area},{self.max_area}), '
                    + '; '.join(debug_items)
                )
        return draw_image, target_info

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.02):
        config_data = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
        return calibrated_pose.pixel_to_calibrated_world(
            position,
            intrinsic,
            extristric,
            white_area_center,
            config_data,
            height=height,
        )

    def calculate_pick_grasp_yaw(self, position, target, target_info, intrinsic, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        gripper_size = [common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
                        common.calculate_pixel_length(0.015, intrinsic, projection_matrix)]

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

    def _sample_scene4_upper_depth(self, center, rgb_shape=None):
        depth_image = self.depth_image
        if depth_image is None:
            return None
        upper = self.scene4_pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]
        depth_cfg = upper['depth']
        half = max(1, int(depth_cfg.get('sample_px', 15)) // 2)
        h, w = depth_image.shape[:2]
        if rgb_shape is not None:
            rgb_h, rgb_w = rgb_shape[:2]
            sx = w / max(1.0, float(rgb_w))
            sy = h / max(1.0, float(rgb_h))
            cx = int(round(float(center[0]) * sx))
            cy = int(round(float(center[1]) * sy))
        else:
            cx, cy = int(center[0]), int(center[1])
        x0 = max(0, cx - half)
        x1 = min(w, cx + half + 1)
        y0 = max(0, cy - half)
        y1 = min(h, cy + half + 1)
        sample = depth_image[y0:y1, x0:x1].astype(np.float64)
        sample = sample[np.isfinite(sample)]
        sample = sample[sample > 0]
        if sample.size == 0:
            return None
        depth_m = float(np.median(sample))
        if depth_m > 10.0:
            depth_m /= 1000.0
        if depth_m < float(depth_cfg['min_m']) or depth_m > float(depth_cfg['max_m']):
            return None
        return depth_m

    def _scene4_upper_depth_pixel(self, center, rgb_shape=None):
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        if rgb_shape is not None:
            rgb_h, rgb_w = rgb_shape[:2]
            sx = w / max(1.0, float(rgb_w))
            sy = h / max(1.0, float(rgb_h))
            x = int(round(float(center[0]) * sx))
            y = int(round(float(center[1]) * sy))
        else:
            x = int(round(float(center[0])))
            y = int(round(float(center[1])))
        x = max(0, min(w - 1, x))
        y = max(0, min(h - 1, y))
        return x, y

    def _calculate_scene4_upper_rgbd_position(self, center, depth_m, rgb_shape=None):
        if self.hand2cam_tf_matrix is None:
            return None, 'missing hand2cam_tf_matrix'
        if self.depth_camera_info is None:
            return None, 'missing depth camera_info'
        depth_pixel = self._scene4_upper_depth_pixel(center, rgb_shape)
        if depth_pixel is None:
            return None, 'missing depth image'

        depth_x, depth_y = depth_pixel
        camera_position = utils.convert_depth_to_camera_coords(
            [depth_x, depth_y, float(depth_m)],
            self.depth_camera_info.k,
        )
        endpoint, pose_snapshot = self._get_endpoint_matrix()
        pose_end = np.matmul(
            self.hand2cam_tf_matrix,
            common.xyz_euler_to_mat(camera_position, (0, 0, 0)),
        )
        world_pose = np.matmul(endpoint, pose_end)
        world_position, _ = common.mat_to_xyz_euler(world_pose)
        raw_position = [float(world_position[0]), float(world_position[1]), float(world_position[2])]
        depth_corrected = self._apply_depth_calibration(raw_position)
        kinematics_corrected = self._apply_kinematics_calibration(depth_corrected)
        return {
            'rgb_pixel': (int(center[0]), int(center[1])),
            'depth_pixel': depth_pixel,
            'depth_m': float(depth_m),
            'camera_m': [float(v) for v in camera_position],
            'raw_m': raw_position,
            'depth_corrected_m': depth_corrected,
            'kinematics_m': kinematics_corrected,
            'pose_snapshot': pose_snapshot,
        }, None

    def _calculate_scene4_upper_grasp_pose(self, center, depth_m):
        pose, _debug = self._calculate_scene4_upper_grasp(center, depth_m)
        return pose

    def _calculate_scene4_upper_grasp(self, center, depth_m):
        upper = self.scene4_pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]
        # The rail does the horizontal tracking. The upper shelf grasp pose is
        # the calibrated pose; depth is used to validate/log the target instead
        # of inventing an uncalibrated x/z correction.
        base = dict(upper.get('grasp_pose') or upper['view_pose'])
        tracking = upper['tracking_grasp']
        depth_delta_m = float(depth_m) - float(tracking['reference_depth_m'])
        error_v = float(center[1]) - float(tracking['target_v'])
        raw_x = float(base['x'])
        raw_z = float(base['z'])
        x = raw_x
        z = raw_z
        pose = {
            'x': x,
            'y': float(base['y']),
            'z': z,
            'pitch': float(base['pitch']),
            'roll': float(base['roll']),
            'claw': float(base.get('claw', 0.0)),
            'time_ms': int(base.get('time_ms', 1000)),
        }
        debug = {
            'base_x': float(base['x']),
            'base_z': float(base['z']),
            'reference_depth_m': float(tracking['reference_depth_m']),
            'depth_delta_m': depth_delta_m,
            'error_v': error_v,
            'target_v': float(tracking['target_v']),
            'raw_x': raw_x,
            'raw_z': raw_z,
            'x_clamped': False,
            'z_clamped': False,
        }
        return pose, debug

    def _move_scene4_upper_rail(self, position):
        position = int(max(0, min(position, scene4_runtime.scene4_rail_config(self._load_scene_config())['total_steps'])))
        ok = scene4_runtime.move_scene4_rail_to_position(
            self,
            self.scene4_stepper_client,
            position,
            scene_path=self.scene_config_path,
            logger=self.get_logger(),
            reset_first=False,
        )
        if ok:
            self.scene4_upper_current_rail = position
            self.scene4_upper_last_motion_time = time.time()
        return ok

    def _process_scene4_upper_frame(self, bgr_image, roi):
        display_image, target_info = self.get_object_pixel_position(bgr_image, roi)
        upper = self.scene4_pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]
        servo = upper['servo']
        scan = upper['scan']
        target_u = int(servo['target_u'])
        tolerance = int(servo['tolerance_px'])
        cv2.line(display_image, (target_u, 0), (target_u, display_image.shape[0]), (255, 0, 255), 2, cv2.LINE_AA)
        cv2.rectangle(display_image, (int(roi[2]), int(roi[0])), (int(roi[3]), int(roi[1])), (255, 255, 255), 2)

        active_target = None
        for target in target_info:
            if self.target_labels.get(target[0], False):
                active_target = target
                break

        now = time.time()
        debug_due = now - self.scene4_upper_last_debug_time > 0.5
        detected = [f'{target[0]}#{target[1]}@{target[2]}' for target in target_info]
        if active_target is None:
            self.scene4_upper_center_count = 0
            if debug_due:
                self.scene4_upper_last_debug_time = now
                enabled = [key for key, value in self.target_labels.items() if value]
                self.get_logger().info(
                    '[Scene4Upper] no active target: '
                    f'enabled={enabled}, detected={detected}, '
                    f'rail={self.scene4_upper_current_rail}'
                )
            if now - self.scene4_upper_last_motion_time > 1.0:
                current = self.scene4_upper_current_rail
                if current is None:
                    current = int(scan['start_abs_position'])
                end = int(scan['end_abs_position'])
                step = int(scan['step'])
                direction = -1 if end < current else 1
                next_position = current + direction * step
                if (direction < 0 and next_position < end) or (direction > 0 and next_position > end):
                    next_position = int(scan['start_abs_position'])
                self.get_logger().info(
                    '[Scene4Upper] scan move: '
                    f'current={current}, next={next_position}, end={end}, step={step}'
                )
                self._move_scene4_upper_rail(next_position)
            return display_image

        center = active_target[2]
        cv2.circle(display_image, (int(center[0]), int(center[1])), 8, (0, 0, 255), -1)
        error_u = int(center[0]) - target_u
        target_v = float(upper['tracking_grasp']['target_v'])
        error_v = float(center[1]) - target_v
        cv2.putText(
            display_image,
            f'upper {active_target[0]} err={error_u}',
            (max(0, int(center[0]) - 80), max(20, int(center[1]) - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        if abs(error_u) > tolerance:
            self.scene4_upper_center_count = 0
            settle_elapsed = now - self.scene4_upper_last_motion_time
            if settle_elapsed > 0.6:
                units_per_px = float(servo['rail_units_per_pixel'])
                max_step = int(servo['max_step'])
                configured_direction = int(servo.get('direction', -1))
                delta = int(round(-error_u * units_per_px))
                delta = max(-max_step, min(max_step, delta))
                self.get_logger().info(
                    '[Scene4Upper] center adjust: '
                    f'target={active_target[0]}, center={center}, '
                    f'target_u={target_u}, error_u={error_u}, tolerance={tolerance}, '
                    f'error_v={error_v:.1f}, delta={delta}, configured_direction={configured_direction}, '
                    f'rail={self.scene4_upper_current_rail}, '
                    f'detected={detected}'
                )
                if delta != 0:
                    current = self.scene4_upper_current_rail
                    if current is None:
                        current = int(scan['start_abs_position'])
                    next_position = current + delta
                    total_steps = scene4_runtime.scene4_rail_config(self._load_scene_config())['total_steps']
                    bounded_position = int(max(0, min(next_position, total_steps)))
                    if bounded_position == current:
                        self.get_logger().warn(
                            '[Scene4Upper] rail limit reached during center adjust: '
                            f'current={current}, requested={next_position}, error_u={error_u}, delta={delta}'
                        )
                        self.scene4_upper_last_motion_time = now
                    else:
                        self._move_scene4_upper_rail(bounded_position)
            elif debug_due:
                self.scene4_upper_last_debug_time = now
                self.get_logger().info(
                    '[Scene4Upper] wait rail settle before depth grasp: '
                    f'target={active_target[0]}, center={center}, target_u={target_u}, '
                    f'error_u={error_u}, tolerance={tolerance}, wait={0.6 - settle_elapsed:.2f}s, '
                    f'rail={self.scene4_upper_current_rail}'
                )
            return display_image

        depth_m = self._sample_scene4_upper_depth(center, bgr_image.shape)
        if depth_m is None:
            self.scene4_upper_center_count = 0
            if debug_due:
                self.scene4_upper_last_debug_time = now
                self.get_logger().info(
                    '[Scene4Upper] centered but depth invalid: '
                    f'target={active_target[0]}, center={center}, '
                    f'error_u={error_u}, error_v={error_v:.1f}, '
                    f'depth_range=({upper["depth"]["min_m"]:.2f},'
                    f'{upper["depth"]["max_m"]:.2f}), rail={self.scene4_upper_current_rail}'
                )
            cv2.putText(display_image, 'depth invalid', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return display_image

        rgbd_pose, rgbd_error = self._calculate_scene4_upper_rgbd_position(center, depth_m, bgr_image.shape)
        if rgbd_pose is None:
            self.scene4_upper_center_count = 0
            if debug_due:
                self.scene4_upper_last_debug_time = now
                self.get_logger().warn(
                    '[Scene4UpperRGBD] unable to calculate arm position: '
                    f'reason={rgbd_error}, target={active_target[0]}, center={center}, '
                    f'depth={depth_m:.3f}, rail={self.scene4_upper_current_rail}'
                )
            cv2.putText(display_image, 'rgbd pose invalid', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return display_image

        self.scene4_upper_center_count += 1
        expected = self.scene4_pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]['grasp_pose']
        raw_mm = [value * 1000.0 for value in rgbd_pose['raw_m']]
        depth_mm = [value * 1000.0 for value in rgbd_pose['depth_corrected_m']]
        kin_mm = [value * 1000.0 for value in rgbd_pose['kinematics_m']]
        if now - self.scene4_upper_last_rgbd_debug_time > 0.5:
            self.scene4_upper_last_rgbd_debug_time = now
            pose_snapshot = rgbd_pose['pose_snapshot']
            self.get_logger().info(
                '[Scene4UpperRGBD] centered target position: '
                f'target={active_target[0]}, confirm={self.scene4_upper_center_count}/4, '
                f'rgb={rgbd_pose["rgb_pixel"]}, depth_px={rgbd_pose["depth_pixel"]}, '
                f'error_u={error_u}, error_v={error_v:.1f}, depth={depth_m:.3f}m, '
                f'camera_m=({rgbd_pose["camera_m"][0]:.4f},{rgbd_pose["camera_m"][1]:.4f},{rgbd_pose["camera_m"][2]:.4f}), '
                f'raw_mm=({raw_mm[0]:.1f},{raw_mm[1]:.1f},{raw_mm[2]:.1f}), '
                f'depth_cal_mm=({depth_mm[0]:.1f},{depth_mm[1]:.1f},{depth_mm[2]:.1f}), '
                f'kin_mm=({kin_mm[0]:.1f},{kin_mm[1]:.1f},{kin_mm[2]:.1f}), '
                f'expected_mm=({expected["x"]:.1f},{expected["y"]:.1f},{expected["z"]:.1f}), '
                f'arm_pose=({pose_snapshot["x"]:.1f},{pose_snapshot["y"]:.1f},{pose_snapshot["z"]:.1f},'
                f'{pose_snapshot["pitch"]:.1f},{pose_snapshot["roll"]:.1f}), '
                f'rail={self.scene4_upper_current_rail}'
            )
        cv2.putText(
            display_image,
            f'rgbd=({kin_mm[0]:.0f},{kin_mm[1]:.0f},{kin_mm[2]:.0f})',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        if self.scene4_upper_center_count >= 4:
            self.scene4_upper_center_count = 0
            grasp_pose = self._calculate_scene4_upper_grasp_pose(center, depth_m)
            self.transport_info = ['__scene4_upper__', active_target[0], depth_m, center, grasp_pose]
            self.start_transport = True
            self.get_logger().info(
                '[Scene4Upper] grasp scheduled after centered RGBD validation: '
                f'target={active_target[0]}, center={center}, depth={depth_m:.3f}m, '
                f'fixed_pose=({grasp_pose["x"]:.1f},{grasp_pose["y"]:.1f},'
                f'{grasp_pose["z"]:.1f},{grasp_pose["pitch"]:.1f}), '
                f'rgbd_kin_mm=({kin_mm[0]:.1f},{kin_mm[1]:.1f},{kin_mm[2]:.1f})'
            )
        return display_image

    def _execute_scene4_upper_transport(self, target_key, depth_m, center, tracked_grasp_pose):
        upper = self.scene4_pick_cfg[scene4_runtime.SCENE4_PICK_ZONE_UPPER]
        view_pose = dict(upper['view_pose'])
        grasp_pose = dict(tracked_grasp_pose)
        retreat_pose = dict(upper['retreat_pose'])
        claw_open = pick_and_place.CLAW_OPEN
        claw_grab = pick_and_place.CLAW_GRAB
        self.get_logger().info(
            f'[Scene4Upper] execute grasp: target={target_key}, center={center}, depth={depth_m:.3f}m, '
            f'pose=({grasp_pose["x"]:.1f},{grasp_pose["y"]:.1f},{grasp_pose["z"]:.1f},'
            f'{grasp_pose["pitch"]:.1f})'
        )

        self.publish_arm(view_pose['x'], view_pose['y'], view_pose['z'], view_pose['pitch'], view_pose['roll'], claw_open, int(view_pose.get('time_ms', 1000)))
        time.sleep(max(0.3, float(view_pose.get('time_ms', 1000)) / 1000.0))
        self.get_logger().info(
            '[Scene4Upper] move to tracked grasp pose: '
            f'x={grasp_pose["x"]:.1f}, y={grasp_pose["y"]:.1f}, '
            f'z={grasp_pose["z"]:.1f}, pitch={grasp_pose["pitch"]:.1f}'
        )
        self.publish_arm(grasp_pose['x'], grasp_pose['y'], grasp_pose['z'], grasp_pose['pitch'], grasp_pose['roll'], claw_open, int(grasp_pose.get('time_ms', 1000)))
        time.sleep(max(0.3, float(grasp_pose.get('time_ms', 1000)) / 1000.0))
        self.get_logger().info('[Scene4Upper] close claw')
        self.publish_arm(grasp_pose['x'], grasp_pose['y'], grasp_pose['z'], grasp_pose['pitch'], grasp_pose['roll'], claw_grab, 500)
        time.sleep(0.8)
        self.get_logger().info(
            '[Scene4Upper] retreat: '
            f'x={retreat_pose["x"]:.1f}, y={retreat_pose["y"]:.1f}, '
            f'z={retreat_pose["z"]:.1f}, pitch={retreat_pose["pitch"]:.1f}'
        )
        self.publish_arm(retreat_pose['x'], retreat_pose['y'], retreat_pose['z'], retreat_pose['pitch'], retreat_pose['roll'], claw_grab, int(retreat_pose.get('time_ms', 1000)))
        time.sleep(max(0.5, float(retreat_pose.get('time_ms', 1000)) / 1000.0))

        if not scene4_runtime.move_scene4_rail(
            self,
            self.scene4_stepper_client,
            "place",
            scene_path=self.scene_config_path,
            logger=self.get_logger(),
        ):
            self.return_to_observation_pose(True)
            return False

        position = self.resolve_place_position(target_key)
        if position is None:
            self.get_logger().warn(f'未知放置目标: {target_key}')
            self.return_to_observation_pose(True)
            return False

        cfg_roll = self.resolve_place_roll(target_key)
        yaw = float(cfg_roll) if cfg_roll is not None else self.calculate_place_grasp_yaw(position, 0)
        place_pitch = self.resolve_place_pitch(target_key, 80.0)
        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['kinematics']['offset'])
        scale = tuple(config_data['kinematics']['scale'])
        angle = math.degrees(math.atan2(position[1], position[0]))
        if angle > 45:
            position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
            position = [position[0] + offset[1], position[1] + offset[0], position[2] + offset[2]]
        elif angle < -45:
            position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
            position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
        else:
            position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
            position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

        finish = pick_and_place.place(position, place_pitch, yaw, 200, self.arm_pub)
        self.return_to_observation_pose(not finish)
        return bool(finish)

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                if self.transport_info is None or len(self.transport_info) == 0:
                    self.get_logger().warn('transport requested but transport_info is empty')
                    self._reset_tracking_state()
                    continue
                if isinstance(self.transport_info[0], str) and self.transport_info[0] == '__scene4_upper__':
                    self.get_logger().warn('scene_4 upper pick transport is disabled; ignoring stale request')
                    self._reset_tracking_state()
                    continue
                position, yaw, target = self.transport_info
                scene4_lower = self._is_scene4_lower_mode()
                _scene_name_dbg = self._load_active_scene_config()[0]
                self.get_logger().info(
                    f'[Transport] scene={_scene_name_dbg}, target={target[0]}#{target[1]}, '
                    f'pick_pos=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f}), '
                    f'pick_yaw={yaw:.1f}'
                )
                if scene4_lower:
                    self.get_logger().info(
                        '[Scene4Lower] execute pick: '
                        f'target={target[0]}#{target[1]}, '
                        f'position=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f}), '
                        f'yaw={yaw:.1f}'
                    )
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics').tolist()
                self.get_logger().info(
                    f'[Transport] calibrated pick_pos=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f})'
                )

                pick_kwargs = {}
                place_kwargs = {}
                if target[0] in COLOR_TARGET_LABELS:
                    pick_kwargs = {
                        'claw_grab_angle': COLOR_CLAW_GRAB_ANGLE,
                    }
                    place_kwargs = {
                        'claw_hold_angle': COLOR_CLAW_GRAB_ANGLE,
                    }
                self.get_logger().info(
                    f'[Transport] pick: pick_kwargs={pick_kwargs}, place_kwargs={place_kwargs}'
                )
                finish = pick_and_place.pick(position, 90, yaw, 540, 0.02, self.arm_pub, **pick_kwargs)
                if finish:
                    position = self.resolve_place_position(target[0])
                    if position is None:
                        self.get_logger().warn(f'未知放置目标: {target[0]}')
                        self.return_to_observation_pose(True)
                        self._reset_tracking_state()
                        continue
                    shelf_place = self._resolve_scene4_shelf_place(target[0], position)
                    if shelf_place is not None:
                        if 'claw_hold_angle' in place_kwargs:
                            shelf_place = dict(shelf_place)
                            shelf_place['claw_hold'] = place_kwargs['claw_hold_angle']
                        self._execute_scene4_shelf_place(target[0], shelf_place)
                        self._reset_tracking_state()
                        continue
                    scene3_active = self._load_active_scene_config()[0] == scene_play_registry.SCENE3_ID
                    self.get_logger().info(
                        f'[Transport] post-pick: scene3_active={scene3_active}, target={target[0]}'
                    )
                    if scene3_active:
                        claw_hold = place_kwargs.get('claw_hold_angle', pick_and_place.CLAW_GRAB)
                        self.get_logger().info(
                            f'[Transport/Scene3] direct place: target={target[0]}, claw_hold={claw_hold}'
                        )
                        finish = self._scene3_direct_place(target[0], claw_hold)
                        self.get_logger().info(f'[Transport/Scene3] place finish={finish}')
                        self.return_to_observation_pose(not finish)
                        self._reset_tracking_state()
                        continue
                    if scene4_lower:
                        self.get_logger().info(f'[Scene4Lower] pick finished, move rail to frame place for {target[0]}')
                    if not scene4_runtime.move_scene4_rail(
                        self,
                        self.scene4_stepper_client,
                        "place",
                        scene_path=self.scene_config_path,
                        logger=self.get_logger(),
                    ):
                        if scene4_lower:
                            self.get_logger().warn('[Scene4Lower] move rail to frame place failed, returning to observation')
                        self.return_to_observation_pose(True)
                        self._reset_tracking_state()
                        continue
                    if scene4_lower:
                        self.get_logger().info(
                            '[Scene4Lower] place target: '
                            f'target={target[0]}, position=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f})'
                        )

                    if scene4_lower:
                        yaw = 0.0
                    else:
                        cfg_roll = self.resolve_place_roll(target[0])
                        yaw = float(cfg_roll) if cfg_roll is not None else self.calculate_place_grasp_yaw(position, 0)
                    place_pitch = self.resolve_place_pitch(target[0], 80.0)
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['kinematics']['offset'])
                    scale = tuple(config_data['kinematics']['scale'])
                    angle = math.degrees(math.atan2(position[1], position[0]))
                    if angle > 45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] + offset[0], position[2] + offset[2]]
                    elif angle < -45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
                    else:
                        position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
                        position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

                    finish = pick_and_place.place(position, place_pitch, yaw, 200, self.arm_pub, **place_kwargs)
                    if finish:
                        if scene4_lower:
                            self.get_logger().info('[Scene4Lower] place finished, return to lower observation')
                        self.return_to_observation_pose(False)
                    else:
                        if scene4_lower:
                            self.get_logger().warn('[Scene4Lower] place failed, return to lower observation with interrupt')
                        self.return_to_observation_pose(True)
                else:
                    if scene4_lower:
                        self.get_logger().warn('[Scene4Lower] pick failed, return to observation')
                        self.return_to_observation_pose(True)
                    else:
                        self.go_home(True)
                self._reset_tracking_state()
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
                    if self._is_scene4_lower_mode():
                        self.get_logger().info(f'[Scene4Lower] roi ready: roi={list(self.roi)}')
                roi = self.roi.copy()
                intrinsic = self.intrinsic
                if len(roi) > 0 and self.enable_sorting and not self.start_transport:
                    display_image, target_info = self.get_object_pixel_position(bgr_image, roi)
                    if self._is_scene4_lower_mode():
                        target_info = self._filter_scene4_lower_targets(target_info)
                    if self._is_scene4_lower_mode():
                        now = time.time()
                        if now - self.scene4_lower_last_debug_time > 1.0:
                            self.scene4_lower_last_debug_time = now
                            detected = [f'{target[0]}#{target[1]}@{target[2]}' for target in target_info]
                            current = None if self.target is None else f'{self.target[0]}#{self.target[1]}'
                            self.get_logger().info(
                                '[Scene4Lower] frame: '
                                f'enabled={self._enabled_targets()}, detected={detected}, '
                                f'current={current}, still={self.count_still}, move={self.count_move}, '
                                f'miss={self.target_miss_count}, transport={self.start_transport}'
                            )

                    tags = self.at_detector.detect(cv2.cvtColor(bgr_image, cv2.COLOR_RGB2GRAY), True, (intrinsic[0,0], intrinsic[1,1], intrinsic[0,2], intrinsic[1,2]), self.tag_size)
                    if len(tags) > 0:
                        index = 0
                        for tag in tags:
                            if 'tag%d'%tag.tag_id in self.target_labels:
                                corners = tag.corners.astype(int)
                                cv2.drawContours(display_image, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
                                rect = cv2.minAreaRect(np.array(tag.corners).astype(np.float32))
                                # rect 包含 (中心点, (宽度, 高度), 旋转角度)
                                (center, (width, height), _) = rect
                                angle = utils.get_long_edge_angle(rect)
                                index += 1
                                target_info.append(['tag%d'%tag.tag_id, index, (int(center[0]), int(center[1])), (int(width), int(height)), angle])
                    if target_info:
                        if self.last_object_info_list:
                            # 对比上一次的物体的位置来重新排序
                            target_info = position_change_detect.position_reorder(target_info, self.last_object_info_list, 20)
                    scene_name, scene_cfg = self._load_active_scene_config()
                    sort_priority = sorting_coordinator.priority_from_scene_config(scene_cfg)
                    detected_color_targets = [
                        target[0] for target in target_info
                        if target[0] in COLOR_TARGET_LABELS and self.target_labels.get(target[0], False)
                    ]
                    sorting_coordinator.report_detections(
                        sorting_coordinator.COLOR_GROUP,
                        detected_color_targets,
                        scene_name,
                        sort_priority,
                    )
                    target_info = sorting_coordinator.sort_items(target_info, lambda item: item[0], sort_priority)
                    self.last_object_info_list = copy.deepcopy(target_info)
                    for target in target_info:
                        cv2.putText(display_image, '{}'.format(target[0]),(target[2][0] - 4 * len(target[0] + str(target[1])), target[2][1] + 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    target_miss = True
                    for target in target_info:  # detect

                        if self.target_labels[target[0]]:  # app set
                            if self.target is not None:  # 如果已经有了目标，其他物体就直接跳过
                                if self.target[0] != target[0] or self.target[1] != target[1]:
                                    continue
                                else:
                                    target_miss = False
                                    self.target = target

                            if self.camera_type == 'usb_cam':
                                x, y = distortion_inverse_map.undistorted_to_distorted_pixel(target[2][0], target[2][1], self.intrinsic, self.distortion)
                                target[2] = (x, y)

                            object_height = COLOR_OBJECT_HEIGHT_M if target[0] in COLOR_TARGET_LABELS else GARBAGE_OBJECT_HEIGHT_M
                            position, projection_matrix = self.get_object_world_position(
                                target[2],
                                intrinsic,
                                self.extristric,
                                self.white_area_center,
                                height=object_height,
                            )
                            result = self.calculate_pick_grasp_yaw(position, target, target_info, intrinsic, projection_matrix)
                            if result is not None and self.target is None:
                                self.target = target
                                if self._is_scene4_lower_mode():
                                    self.target_miss_count = 0
                                    self.get_logger().info(
                                        '[Scene4Lower] lock target: '
                                        f'target={target[0]}#{target[1]}, pixel={target[2]}, '
                                        f'position=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f})'
                                    )
                                break

                            if self.last_position is not None and self.target is not None and result is not None:
                                e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                    pow(self.last_position[1] - position[1], 2)), 5)
                                # self.get_logger().info(f'e_distance: {e_distance}')
                                if e_distance <= 0.005:  # 欧式距离小于2mm, 防止物体还在移动时就去夹取了
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
                                    # self.get_logger().info(f'pick:{position}')
                                    if target[0] in COLOR_TARGET_LABELS:
                                        claimed, claim_msg = sorting_coordinator.try_claim(
                                            sorting_coordinator.COLOR_GROUP,
                                            target[0],
                                            scene_name,
                                            sort_priority,
                                        )
                                        if not claimed:
                                            self.get_logger().info(f'[SortPriority] skip color target={target[0]}: {claim_msg}')
                                            self.target = None
                                            self.last_position = None
                                            continue
                                        self.sort_claim_target = target[0]
                                    self.target = target
                                    yaw = utils.normalize_gripper_roll_deg(result[0])
                                    self.get_logger().info(
                                        f'[PickYaw] target={target[0]}, '
                                        f'raw_angle={result[0]:.1f}, normalized_yaw={yaw:.1f}, '
                                        f'pos=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f})'
                                    )
                                    self.transport_info = [position, yaw, target]
                                    self.start_transport = True
                                    if self._is_scene4_lower_mode():
                                        self.get_logger().info(
                                            '[Scene4Lower] grasp scheduled: '
                                            f'target={target[0]}#{target[1]}, pixel={target[2]}, '
                                            f'position=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f}), '
                                            f'yaw={yaw:.1f}'
                                        )
                            self.last_position = position
                    if target_miss:
                        self.target_miss_count += 1
                    if self.target_miss_count > 10:
                        if self._is_scene4_lower_mode() and self.target is not None:
                            self.get_logger().info(
                                '[Scene4Lower] drop lost target: '
                                f'target={self.target[0]}#{self.target[1]}, enabled={self._enabled_targets()}'
                            )
                        self.target_miss_count = 0
                        self.target = None
                else:
                    display_image = bgr_image.copy()
                # self.fps.update()
                # display_image = self.fps.show_fps(display_image)
                if bgr_image is not None and self.get_parameter('display').value:
                    cv2.imshow('result_image', display_image)
                    cv2.waitKey(1)
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
            else:
                time.sleep(0.1)

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def depth_camera_info_callback(self, msg):
        self.depth_camera_info = msg

    def image_callback(self, ros_rgb_image):
        # 将ros格式图像转换为opencv格式(convert the image from ros format to opencv format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((bgr_image))

    def depth_image_callback(self, ros_depth_image):
        if ros_depth_image.encoding in ('16UC1', 'mono16'):
            depth_image = np.ndarray(
                shape=(ros_depth_image.height, ros_depth_image.width),
                dtype=np.uint16,
                buffer=ros_depth_image.data,
            ).copy()
        else:
            depth_image = self.bridge.imgmsg_to_cv2(ros_depth_image, desired_encoding='passthrough')
            depth_image = np.array(depth_image)
        self.depth_image = depth_image

def main():
    rclpy.init()
    node = ObjectSortingNode('object_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False  # 停止线程标志
        executor.shutdown()

if __name__ == "__main__":
    main()
