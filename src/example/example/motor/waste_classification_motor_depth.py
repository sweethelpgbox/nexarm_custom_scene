#!/usr/bin/env python3
# encoding: utf-8
# YOLO 前视识别抓取（使用 yolo/object_detect 的 box 中心点替代颜色块中心点）

import os
import cv2
import math
import time
import yaml
import json
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from cv_bridge import CvBridge
from rclpy.node import Node
from sdk import common, fps
from sdk.scene_context import load_scene_environment
from app.utils import utils
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, SetBool
from interfaces.srv import SetString, SetStringList
from interfaces.msg import ObjectsInfo
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


SCENE5_PREVIEW_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


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


class TrackAndGrabNode(Node):
    INIT_X = 180.0
    INIT_Y = 0.0
    INIT_Z = 230.0
    INIT_PITCH = -90.0
    INIT_ROLL = 0.0
    INIT_CLAW = -90.0
    GRAB_CLAW = -45.0
    OPEN_CLAW = -90.0
    DEFAULT_HOME = {
        'x': INIT_X,
        'y': INIT_Y,
        'z': INIT_Z,
        'pitch': INIT_PITCH,
        'roll': INIT_ROLL,
        'claw': INIT_CLAW,
        'time_ms': 1500,
    }

    PLACE_X = 100.0
    PLACE_Y = -150.0
    PLACE_Z = 80.0
    PLACE_LIFT_Z = 150.0
    PLACE_PITCH = -60.0

    WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
    }
    COLOR_CLASSES = ('red', 'green', 'blue', 'yellow')

    SCENE5_WASTE_SLOT_POSITIONS = [
        [-0.05, -0.460, 0.27],
        [-0.05, -0.360, 0.27],
        [0.06, -0.460, 0.27],
        [0.06, -0.360, 0.27]
    ]

    place_position = {
        'food_waste': [-0.05, -0.360, 0.27],
        'residual_waste': [0.06, -0.460, 0.27],
        'hazardous_waste': [-0.05, -0.460, 0.27],
        'recyclable_waste': [0.06, -0.360, 0.27]
    }
    DEFAULT_FIXED_PICK = {
        'enabled': True,
        'x': 200.0,
        'y': 15.0,
        'z': 100.0,
        'approach_z_offset': 10.0,
        'lift_z_offset': 30.0,
        'transfer_x': 200.0,
        'transfer_y': 0.0,
        'transfer_z': 200.0,
        'pitch': -90.0,
        'roll': 0.0,
        'pre_grab_roll': 0.0,
        'open_claw': -60.0,
        'color_close_claw': -17.0,
        'waste_close_claw': -45.0,
        'close_claw': -45.0,
        'trigger_center_x': 320.0,
        'trigger_center_y': 280.0,
        'trigger_tolerance_x': 20.0,
        'trigger_tolerance_y': 120.0,
    }

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.running = True
        self.start = False
        self.enter = False
        self.moving = False
        self.display = self.get_bool_param('display', True)
        self.preview_compressed_quality = max(1, min(95, self.get_int_param('preview_compressed_quality', 65)))
        self.stable_count = 0
        self.last_world_position = None
        self.last_target_class = None
        self.last_pick_stamp = time.time()
        self.start_stamp = time.time() + 1.0
        self.endpoint = None
        self.current_pose = None
        self.color_min_area = 500
        self.color_max_area = 15000
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
        self.scene_config_path = self.get_string_param(
            'config_path',
            '/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml',
        )
        self.play_config_path = self.get_string_param(
            'play_config_path',
            '/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml',
        )
        self.home_pose = self.load_home_config()
        self.known_pose.update({key: self.home_pose[key] for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw')})
        self.scene_env = load_scene_environment()
        self.calibration_file = 'calibration.yaml'
        self.transform_file = 'transform.yaml'
        self.camera_info_path = '/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml'
        self.lab_config_path = os.path.join(self.config_path, 'lab_config.yaml')
        self.lab_data = {}
        self.hand2cam_tf_matrix = None
        self.plane = None
        self.depth_offset = (0.0, 0.0, 0.0)
        self.depth_scale = (1.0, 1.0, 1.0)
        self.kinematics_offset = (0.0, 0.0, 0.0)
        self.kinematics_scale = (1.0, 1.0, 1.0)
        self.image_queue = queue.Queue(maxsize=2)

        # YOLO 结果缓存
        # 你的话题结构：
        # objects:
        # - class_name: Umbrella
        #   box: [360, 81, 94, 85]   # 左上角x、左上角y、框宽、框高
        #   score: 0.948
        #   width: 640               # YOLO图像宽
        #   height: 400              # YOLO图像高
        self.yolo_object = None
        self.yolo_lock = threading.Lock()
        self.yolo_target_class = None  # None表示抓取最高置信度目标；也可通过 ~/set_color 服务设置类别名
        self.target_list = []
        self.scene5_slot_order = list(self.WASTE_CLASSES.keys())
        self.scene5_place_targets = {
            'color': self.load_scene5_place_targets('color'),
            'waste': self.load_scene5_place_targets('waste'),
        }
        self.fixed_pick = self.load_fixed_pick_config()
        self.fixed_pick_enabled = bool(self.fixed_pick.get('enabled', True))
        default_service_prefix = self.scene_env.role_namespace if self.scene_env.is_scene5 else ''
        default_object_topic = (
            self.scene_env.topic('yolo/object_detect')
            if self.scene_env.is_scene5 else '/yolo/object_detect'
        )
        default_yolo_start_service = (
            self.scene_env.topic('yolo/start')
            if self.scene_env.is_scene5 else 'yolo/start'
        )
        default_yolo_stop_service = (
            self.scene_env.topic('yolo/stop')
            if self.scene_env.is_scene5 else 'yolo/stop'
        )
        default_rgb_topic = (
            self.scene_env.camera_topic('rgb/image_raw')
            if self.scene_env.is_scene5 else 'depth_cam/rgb/image_raw'
        )
        default_depth_topic = (
            self.scene_env.camera_topic('depth/image_raw')
            if self.scene_env.is_scene5 else 'depth_cam/depth/image_raw'
        )
        default_depth_info_topic = (
            self.scene_env.camera_topic('depth/camera_info')
            if self.scene_env.is_scene5 else 'depth_cam/depth/camera_info'
        )
        self.service_prefix = self.get_string_param('service_prefix', default_service_prefix)
        self.object_topic = self.get_string_param('object_topic', default_object_topic)
        self.yolo_start_service = self.get_string_param('yolo_start_service', default_yolo_start_service)
        self.yolo_stop_service = self.get_string_param('yolo_stop_service', default_yolo_stop_service)
        self.rgb_topic = self.get_string_param('rgb_topic', default_rgb_topic)
        self.depth_topic = self.get_string_param('depth_topic', default_depth_topic)
        self.depth_info_topic = self.get_string_param('depth_info_topic', default_depth_info_topic)
        default_controller_prefix = (
            self.scene_env.controller_prefix
            if self.scene_env.is_scene5 else '/ros_robot_controller'
        )
        try:
            value = self.get_parameter('controller_prefix').value
            self.controller_prefix = str(value or default_controller_prefix).rstrip('/')
        except Exception:
            self.controller_prefix = default_controller_prefix
        if not self.controller_prefix.startswith('/'):
            self.controller_prefix = '/' + self.controller_prefix
        self.control_conveyor = self.get_bool_param('control_conveyor', True)

        signal.signal(signal.SIGINT, self.shutdown)
        self.timer_cb_group = ReentrantCallbackGroup()

        self.conveyor_pub = (
            self.create_publisher(Int8, self.ctl_topic('conveyor/set'), 1)
            if self.control_conveyor else None
        )
        self.arm_pub = self.create_publisher(ArmCoords, self.ctl_topic('arm/set_coords'), 5)
        self.create_subscription(ArmFullState, self.ctl_topic('arm/full_state'), self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(
            GetArmFullState,
            self.ctl_topic('arm/get_full_state'),
            callback_group=self.timer_cb_group,
        )
        self.controller_init_client = self.create_client(
            Trigger,
            self.ctl_topic('controller_manager/init_finish'),
            callback_group=self.timer_cb_group,
        )
        self.kinematics_init_client = self.create_client(
            Trigger,
            self.ctl_topic('kinematics/init_finish'),
            callback_group=self.timer_cb_group,
        )
        self.scene_runtime_prepare_client = self.create_client(
            Trigger,
            self.ctl_topic('scene_runtime/prepare'),
            callback_group=self.timer_cb_group,
        )

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback)
        self.create_service(Trigger, self.service_name('scene5_waste_classification/enter'), self.enter_srv_callback)
        self.create_service(Trigger, self.service_name('scene5_waste_classification/exit'), self.exit_srv_callback)
        self.create_service(SetBool, self.service_name('scene5_waste_classification/enable_transport'), self.start_srv_callback)
        self.create_service(SetStringList, self.service_name('scene5_waste_classification/set_target'), self.set_target_srv_callback)
        self.create_service(SetStringList, self.service_name('scene5_waste_classification/set_slot_order'), self.on_scene5_set_slot_order)
        self.create_service(SetString, self.service_name('scene5_waste_classification/set_place_targets'), self.on_scene5_set_place_targets)
        self.create_service(SetString, self.service_name('scene5_waste_classification/set_fixed_pick'), self.on_set_fixed_pick)

        self.object_sub = self.create_subscription(
            ObjectsInfo,
            self.object_topic,
            self.get_object_callback,
            1
        )

        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, self.depth_info_topic)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.2)
        sync.registerCallback(self.multi_callback)

        self.start_yolo_client = self.create_client(Trigger, self.yolo_start_service, callback_group=self.timer_cb_group)
        self.start_yolo_client.wait_for_service()
        self.stop_yolo_client = self.create_client(Trigger, self.yolo_stop_service, callback_group=self.timer_cb_group)
        self.stop_yolo_client.wait_for_service()

        self.result_image_pub = self.create_publisher(Image, '~/result_image', 1)
        self.result_image_compressed_pub = self.create_publisher(
            CompressedImage,
            '~/result_image/compressed',
            SCENE5_PREVIEW_QOS,
        )
        self.bridge = CvBridge()

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            if isinstance(value, str):
                return value.strip().lower() in ('1', 'true', 'yes', 'on')
            return bool(value)
        except Exception:
            return default

    def get_string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

    def get_int_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return int(value)
        except Exception:
            pass
        return int(default)

    def publish_preview_images(self, result_bgr):
        try:
            stamp = self.get_clock().now().to_msg()
            image_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(result_bgr, cv2.COLOR_BGR2RGB), encoding='rgb8')
            image_msg.header.stamp = stamp
            self.result_image_pub.publish(image_msg)

            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.format = 'jpeg'
            ok, encoded = cv2.imencode('.jpg', result_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.preview_compressed_quality])
            if not ok:
                self.get_logger().warn('encode compressed result_image failed')
                return
            compressed_msg.data = encoded.tobytes()
            self.result_image_compressed_pub.publish(compressed_msg)
        except Exception as exc:
            self.get_logger().warn(f'publish result_image failed: {exc}')

    def load_scene5_place_targets(self, group):
        try:
            config = {}
            if self.play_config_path and os.path.exists(self.play_config_path):
                with open(self.play_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
            targets = (
                config.get('scene5_dual_arm', {})
                .get('arm_b_place_targets', {})
                .get(group, {})
            )
            if not isinstance(targets, dict):
                with open(self.scene_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
                targets = (
                    config.get('scenes', {})
                    .get('scene_5', {})
                    .get('scene5_dual_arm', {})
                    .get('arm_b_place_targets', {})
                    .get(group, {})
                )
            if not isinstance(targets, dict):
                return {}
            return self.normalize_scene5_place_targets(targets, group)
        except Exception as ex:
            self.get_logger().warn(f'load scene5 {group} place targets failed: {ex}')
            return {}

    def load_lab_config(self):
        try:
            data = common.get_yaml_data(self.lab_config_path)
            params = data.get('/**', {}).get('ros__parameters', {})
            ranges = params.get('color_range_list', {})
            if isinstance(ranges, dict):
                self.lab_data = ranges
        except Exception as ex:
            self.lab_data = {}
            self.get_logger().warn(f'load lab_config.yaml failed: {ex}')

    def load_fixed_pick_config(self):
        cfg = dict(self.DEFAULT_FIXED_PICK)
        try:
            config = {}
            if self.play_config_path and os.path.exists(self.play_config_path):
                with open(self.play_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
            fixed_pick = (
                config.get('scene5_dual_arm', {})
                .get('arm_b_fixed_pick', {})
            )
            if not isinstance(fixed_pick, dict):
                fixed_pick = {}
            for key, default_value in self.DEFAULT_FIXED_PICK.items():
                value = fixed_pick.get(key, default_value)
                if key == 'enabled':
                    cfg[key] = bool(value)
                else:
                    cfg[key] = float(value)
        except Exception as ex:
            self.get_logger().warn(f'load scene5 fixed pick failed: {ex}')
        return cfg

    def load_home_config(self):
        cfg = dict(self.DEFAULT_HOME)
        try:
            config = {}
            if self.play_config_path and os.path.exists(self.play_config_path):
                with open(self.play_config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
            home = (
                config.get('scene5_dual_arm', {})
                .get('arm_b', {})
                .get('home', {})
            )
            if not isinstance(home, dict):
                home = {}
            for key, default_value in self.DEFAULT_HOME.items():
                cfg[key] = float(home.get(key, default_value))
            cfg['time_ms'] = int(cfg.get('time_ms', 1500))
        except Exception as ex:
            self.get_logger().warn(f'load scene5 arm_b home failed: {ex}')
        return cfg

    def fixed_pick_close_claw(self, class_name):
        key = 'color_close_claw' if class_name in self.COLOR_CLASSES else 'waste_close_claw'
        return float(self.fixed_pick.get(key, self.fixed_pick.get('close_claw', self.GRAB_CLAW)))

    def normalize_scene5_place_targets(self, targets, group='waste'):
        if not isinstance(targets, dict):
            return {}
        clean = {}
        categories = self.COLOR_CLASSES if group == 'color' else self.WASTE_CLASSES.keys()
        for category in categories:
            value = targets.get(category)
            if not isinstance(value, (list, tuple)) or len(value) < 2:
                continue
            try:
                x = float(value[0])
                y = float(value[1])
                if abs(x) > 2.0 or abs(y) > 2.0:
                    x /= 1000.0
                    y /= 1000.0
                if len(value) >= 3:
                    z = float(value[2])
                else:
                    fallback_z = 0.06 if group == 'color' else 0.27
                    fallback = self.place_position.get(category, [0.0, 0.0, fallback_z])
                    z = float(fallback[2])
                clean[category] = [round(x, 3), round(y, 3), round(z, 3)]
            except Exception:
                continue
        return clean

    def ctl_topic(self, suffix):
        return f'{self.controller_prefix}/{suffix.lstrip("/")}'

    def service_name(self, suffix):
        prefix = str(self.service_prefix or '').strip().strip('/')
        suffix = str(suffix).strip().strip('/')
        return f'/{prefix}/{suffix}' if prefix else suffix

    def should_display(self):
        return self.display

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum=None, frame=None):
        self.running = False

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
        self.get_logger().info('等待底层控制初始化...')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            self.get_logger().info('等待机械臂坐标控制订阅...')
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
        except Exception as exc:
            self.get_logger().warn(f'加载 plane 失败: {exc}')

        try:
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
            self.depth_offset = tuple(float(v) for v in calibration['depth']['offset'])
            self.depth_scale = tuple(float(v) for v in calibration['depth']['scale'])
            self.kinematics_offset = tuple(float(v) for v in calibration['kinematics']['offset'])
            self.kinematics_scale = tuple(float(v) for v in calibration['kinematics']['scale'])
        except Exception as exc:
            self.get_logger().warn(f'加载 calibration.yaml 失败: {exc}')

    def apply_depth_calibration(self, position):
        result = list(position)
        for i in range(3):
            result[i] = result[i] * self.depth_scale[i]
            result[i] = result[i] + self.depth_offset[i]
        return result

    def apply_kinematics_calibration(self, position):
        result = list(position)
        for i in range(3):
            result[i] = result[i] * self.kinematics_scale[i]
            result[i] = result[i] + self.kinematics_offset[i]
        return result

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        self.load_calibration_parameters()
        self.load_lab_config()
        self.kinematics_offset = (
            self.kinematics_offset[0] - 0.05,
            self.kinematics_offset[1],
            self.kinematics_offset[2],
        )
        start_on_boot = self.get_bool_param('start', False)
        if start_on_boot:
            ok, msg = self.prepare_and_home()
            if ok:
                self.enter = True
                self.start = True
                self.send_request(self.start_yolo_client, Trigger.Request())
            else:
                self.start = False
                self.get_logger().warn(msg)
        elif self.get_bool_param('auto_home_on_start', True):
            ok, msg = self.prepare_and_home()
            if not ok:
                self.get_logger().warn(msg)
            try:
                self.yolo_target_class = str(self.get_parameter('color').value)
            except Exception:
                self.yolo_target_class = None
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'waste_classification_motor_depth ready')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

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
        self.go_home(wait_time=1.5)
        return True, msg

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
            'yaw': float(self.known_pose.get('yaw', 0.0)),
        }

    def place_by_category(self, class_name):
        pos = self.get_scene5_place_position(class_name)
        if pos is None:
            return
        pos = [x * 1000 for x in pos]
        x, y, z = pos
        close_claw = self.fixed_pick_close_claw(class_name)
        if class_name in self.COLOR_CLASSES:
            transfer_z = max(200.0, z + 120.0)
            self.publish_arm(x, y, transfer_z, -90.0, 0.0, close_claw, 1500)
            time.sleep(1.8)
            self.publish_arm(x, y, z, -90.0, 0.0, close_claw, 800)
            time.sleep(1.0)
            self.publish_arm(x, y, z, -90.0, 0.0, self.OPEN_CLAW, 500)
            time.sleep(0.6)
            self.publish_arm(x, y, transfer_z, -90.0, 0.0, self.OPEN_CLAW, 800)
            time.sleep(1.0)
            return
        self.publish_arm(x, y, z, 0.0, 0.0, close_claw, 1500)
        time.sleep(1.8)
        self.publish_arm(x, y, z, 0.0, 0.0, self.OPEN_CLAW, 800)
        time.sleep(1.0)
        self.publish_arm(x, -200.0, z, self.PLACE_PITCH, 0.0, self.OPEN_CLAW, 800)
        time.sleep(1.0)

    def go_home(self, wait_time=1.0):
        home = self.home_pose
        self.publish_arm(
            home['x'],
            home['y'],
            home['z'],
            home['pitch'],
            home['roll'],
            home['claw'],
            home.get('time_ms', 1500),
        )
        time.sleep(wait_time)
        self.get_endpoint_matrix()

    def get_endpoint_matrix(self):
        p = self.get_pose_snapshot()
        if p is None:
            p = self.known_pose
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

    def get_endpoint(self):
        return self.get_endpoint_matrix()

    def set_color_srv_callback(self, request, response):
        # 兼容原来的 set_color 服务：现在用 data 设置 YOLO 目标类别。
        # 例如 data 为 Umbrella，则只抓 class_name == Umbrella 的目标。
        # 如果 data 为空字符串，则抓最高置信度目标。
        self.yolo_target_class = request.data.strip() if request.data is not None else None
        if self.yolo_target_class == '':
            self.yolo_target_class = None
        self.start = True
        self.stable_count = 0
        self.last_world_position = None
        response.success = True
        response.message = 'set_yolo_class'
        self.get_logger().info('\033[1;32mset yolo class: %s\033[0m' % str(self.yolo_target_class))
        return response

    def start_srv_callback(self, request, response):
        self.start = bool(getattr(request, 'data', True))
        if self.start and not self.enter:
            ok, msg = self.prepare_and_home()
            if not ok:
                self.start = False
                response.success = False
                response.message = msg
                return response
            self.send_request(self.start_yolo_client, Trigger.Request())
            self.enter = True
        if not self.start:
            self.moving = False
            self.stable_count = 0
            self.last_world_position = None
        response.success = True
        response.message = 'start' if self.start else 'stop'
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        self.moving = False
        self.stable_count = 0
        self.last_world_position = None
        with self.yolo_lock:
            self.yolo_object = None
        self.go_home(wait_time=1.0)
        response.success = True
        response.message = 'stop'
        return response

    def enter_srv_callback(self, request, response):
        ok, msg = self.prepare_and_home()
        if not ok:
            response.success = False
            response.message = msg
            return response
        self.enter = True
        self.start = False
        self.moving = False
        self.target_list = []
        self.yolo_target_class = None
        self.send_request(self.start_yolo_client, Trigger.Request())
        response.success = True
        response.message = 'scene5 enter'
        self.get_logger().info('scene5_waste_classification enter')
        return response

    def exit_srv_callback(self, request, response):
        self.enter = False
        self.start = False
        self.moving = False
        self.stable_count = 0
        self.last_world_position = None
        with self.yolo_lock:
            self.yolo_object = None
        self.send_request(self.stop_yolo_client, Trigger.Request())
        if self.conveyor_pub is not None:
            conveyor_msg = Int8()
            conveyor_msg.data = 0
            self.conveyor_pub.publish(conveyor_msg)
        response.success = True
        response.message = 'scene5 exit'
        self.get_logger().info('scene5_waste_classification exit')
        return response

    def set_target_srv_callback(self, request, response):
        target_list = []
        for category in request.data:
            if not isinstance(category, str):
                continue
            if category in self.COLOR_CLASSES:
                target_list.append(category)
            elif category in self.WASTE_CLASSES:
                target_list.extend(list(self.WASTE_CLASSES[category]))
        self.target_list = target_list
        response.success = True
        response.message = 'set target'
        self.get_logger().info('scene5_waste_classification target list: %s' % str(self.target_list))
        return response

    def on_scene5_set_slot_order(self, request, response):
        self.get_logger().info('received scene5_waste_classification slot order from upstream: %s' % str(request.data))
        order = [item for item in request.data if isinstance(item, str) and item in self.WASTE_CLASSES]
        if len(order) == len(self.WASTE_CLASSES) and set(order) == set(self.WASTE_CLASSES.keys()):
            self.scene5_slot_order = order
            response.success = True
            response.message = 'set slot order'
            self.get_logger().info('scene5_waste_classification slot order set: %s' % str(self.scene5_slot_order))
        else:
            response.success = False
            response.message = 'invalid slot order'
            self.get_logger().warn('scene5_waste_classification invalid slot order: %s' % str(request.data))
        return response

    def on_scene5_set_place_targets(self, request, response):
        try:
            payload = json.loads(request.data or '{}')
            if not isinstance(payload, dict):
                response.success = False
                response.message = 'invalid place targets'
                return response
            clean = {}
            if isinstance(payload.get('color'), dict):
                clean['color'] = self.normalize_scene5_place_targets(payload.get('color'), 'color')
            if isinstance(payload.get('waste'), dict):
                clean['waste'] = self.normalize_scene5_place_targets(payload.get('waste'), 'waste')
            if not clean:
                clean['waste'] = self.normalize_scene5_place_targets(payload, 'waste')
            if not any(clean.values()):
                response.success = False
                response.message = 'invalid place targets'
                return response
            for group, targets in clean.items():
                if targets:
                    self.scene5_place_targets[group] = targets
            response.success = True
            response.message = 'set place targets'
            self.get_logger().info('scene5_waste_classification place targets set: %s' % str(self.scene5_place_targets))
        except Exception as ex:
            response.success = False
            response.message = str(ex)
            self.get_logger().warn('scene5_waste_classification invalid place targets: %s' % str(ex))
        return response

    def on_set_fixed_pick(self, request, response):
        try:
            payload = json.loads(request.data or '{}')
            if not isinstance(payload, dict):
                response.success = False
                response.message = 'invalid fixed_pick payload'
                return response
            float_keys = [k for k in self.DEFAULT_FIXED_PICK if k != 'enabled']
            updated = []
            for key in float_keys:
                if key in payload:
                    self.fixed_pick[key] = float(payload[key])
                    updated.append(f'{key}={self.fixed_pick[key]}')
            if 'enabled' in payload:
                self.fixed_pick['enabled'] = bool(payload['enabled'])
                self.fixed_pick_enabled = self.fixed_pick['enabled']
                updated.append(f'enabled={self.fixed_pick_enabled}')
            response.success = True
            response.message = 'fixed_pick updated: ' + ', '.join(updated)
            self.get_logger().info(f'[set_fixed_pick] {response.message}')
        except Exception as ex:
            response.success = False
            response.message = str(ex)
            self.get_logger().warn(f'[set_fixed_pick] error: {ex}')
        return response

    def get_scene5_place_position(self, class_name):
        if class_name in self.COLOR_CLASSES:
            pos = self.scene5_place_targets.get('color', {}).get(class_name)
            if pos is not None:
                self.get_logger().info(f'scene5 color place position: color={class_name}, pos={pos}')
                return pos
            return None
        for category, classes in self.WASTE_CLASSES.items():
            if class_name in classes:
                pos = self.scene5_place_targets.get('waste', {}).get(category)
                if pos is not None:
                    self.get_logger().info(f'scene5 place position fixed: category={category}, pos={pos}')
                    return pos
                # If GUI provided a slot order, map category -> slot index -> slot coordinates
                try:
                    if isinstance(self.scene5_slot_order, (list, tuple)) and category in self.scene5_slot_order:
                        index = int(self.scene5_slot_order.index(category))
                        if 0 <= index < len(self.SCENE5_WASTE_SLOT_POSITIONS):
                            pos = self.SCENE5_WASTE_SLOT_POSITIONS[index]
                            self.get_logger().info(f'scene5 place position mapped: category={category}, slot={index}, pos={pos}')
                            return pos
                except Exception as e:
                    self.get_logger().warn(f'error mapping scene5 slot order: {e}')
                # Fallback: use fixed place_position map
                pos = self.place_position.get(category)
                self.get_logger().info(f'scene5 place position fallback: category={category}, pos={pos}')
                return pos
        return None

    def get_object_callback(self, msg):
        if len(msg.objects) == 0:
            with self.yolo_lock:
                self.yolo_object = None
            return

        target = None
        for obj in msg.objects:
            class_name = str(obj.class_name)
            if self.target_list and class_name not in self.target_list:
                continue
            if self.yolo_target_class and class_name != self.yolo_target_class:
                continue
            if target is None or float(obj.score) > float(target.score):
                target = obj

        if target is None:
            with self.yolo_lock:
                self.yolo_object = None
            return

        box = list(target.box)
        if len(box) < 4:
            with self.yolo_lock:
                self.yolo_object = None
            return

        center_x = float(box[0])
        center_y = float(box[1])
        box_w = float(box[2])
        box_h = float(box[3])

        box_x = center_x - box_w / 2.0
        box_y = center_y - box_h / 2.0

        with self.yolo_lock:
            self.yolo_object = {
                'class_name': str(target.class_name),
                'score': float(target.score),
                'box_x': box_x,
                'box_y': box_y,
                'box_w': box_w,
                'box_h': box_h,
                'center_x': center_x,
                'center_y': center_y,
                'image_width': float(target.width),
                'image_height': float(target.height),
                'angle': float(target.angle),
                'stamp': time.time(),
            }

    def normalize_roll_angle(self, angle):
        angle = float(angle)

        # 归一化到 -180 ~ 180
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0

        return angle

    def yolo_angle_to_arm_roll(self, yolo_angle):
        # 根据实测：
        # YOLO angle = 38  时，roll = -33 爪子方向一致
        # YOLO angle = -38 时，roll = -33 爪子方向也一致
        # 所以这里忽略 angle 正负号，只使用绝对值。
        roll_offset = 5.0

        roll = -abs(float(yolo_angle)) + roll_offset

        roll = self.normalize_roll_angle(roll)

        # 夹爪 180 度对称，限制到 -90 ~ 90
        if roll > 90.0:
            roll -= 180.0
        elif roll < -90.0:
            roll += 180.0

        roll = max(-90.0, min(90.0, roll))

        return roll

    def fixed_pick_target_in_window(self, center_x, center_y):
        p = self.fixed_pick
        trigger_x = float(p.get('trigger_center_x', 320.0))
        trigger_y = float(p.get('trigger_center_y', 240.0))
        tolerance_x = float(p.get('trigger_tolerance_x', 20.0))
        tolerance_y = float(p.get('trigger_tolerance_y', 20.0))
        return abs(float(center_x) - trigger_x) <= tolerance_x and abs(float(center_y) - trigger_y) <= tolerance_y

    def adaptive_threshold(self, gray_image):
        return cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)

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
        return cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)

    def detect_color_targets(self, rgb_image, result_bgr):
        if not self.lab_data:
            return []
        roi_rgb = self.get_top_surface(rgb_image)
        image_lab = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2LAB)
        targets = []
        for color in self.COLOR_CLASSES:
            color_range = self.lab_data.get(color)
            if not isinstance(color_range, dict):
                continue
            try:
                mask = cv2.inRange(
                    image_lab,
                    tuple(color_range['min']),
                    tuple(color_range['max']),
                )
            except Exception:
                continue
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            for contour in contours:
                area = math.fabs(cv2.contourArea(contour))
                if not self.color_min_area <= area <= self.color_max_area:
                    continue
                rect = cv2.minAreaRect(contour)
                (center_x, center_y), _ = cv2.minEnclosingCircle(contour)
                corners = cv2.boxPoints(rect)
                cv2.drawContours(result_bgr, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(result_bgr, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                targets.append({
                    'class_name': color,
                    'score': area,
                    'box_x': float(rect[0][0] - rect[1][0] / 2.0),
                    'box_y': float(rect[0][1] - rect[1][1] / 2.0),
                    'box_w': float(rect[1][0]),
                    'box_h': float(rect[1][1]),
                    'center_x': float(center_x),
                    'center_y': float(center_y),
                    'image_width': float(rgb_image.shape[1]),
                    'image_height': float(rgb_image.shape[0]),
                    'angle': float(rect[2]),
                    'stamp': time.time(),
                    'source': 'color',
                })
        targets.sort(key=lambda item: item['score'], reverse=True)
        return targets

    def color_target_allowed(self, class_name):
        if self.target_list and class_name not in self.target_list:
            return False
        if self.yolo_target_class and self.yolo_target_class in self.COLOR_CLASSES and class_name != self.yolo_target_class:
            return False
        if self.yolo_target_class and self.yolo_target_class not in self.COLOR_CLASSES:
            return False
        return True

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except Exception:
                pass
        rgb_image, _ = decode_color_image(ros_rgb_image)
        depth_image = np.ndarray(
            shape=(ros_depth_image.height, ros_depth_image.width),
            dtype=np.uint16,
            buffer=ros_depth_image.data,
        )
        self.image_queue.put((np.copy(rgb_image), np.copy(depth_image), depth_camera_info))

    def pick(self, position,class_name, grab_roll=0.0):
        try:
            roll = float(grab_roll)
            position = list(position)
            if position[0] > 0.22:
                position[2] += 0.01
            position = self.apply_kinematics_calibration(position)

            x_mm = float(position[0]) * 1000.0
            y_mm = float(position[1]) * 1000.0 + 1
            z_mm = float(position[2]) * 1000.0
            pitch = -90.0

            z_down = z_mm - 18.0
            self.publish_arm(x_mm, y_mm, z_down, pitch, roll, self.OPEN_CLAW, 800)
            time.sleep(1.0)

            time.sleep(1.0)
            close_claw = self.fixed_pick_close_claw(class_name)
            self.publish_arm(x_mm, y_mm, z_down, pitch, roll, close_claw, 700)
            time.sleep(0.8)

            self.publish_arm(x_mm, y_mm, z_mm + 40.0, pitch, roll, close_claw, 1000)
            time.sleep(1.2)

            self.place_by_category(class_name)

            self.go_home(wait_time=1.5)
        finally:
            self.stable_count = 0
            self.last_world_position = None
            self.last_pick_stamp = time.time()
            self.moving = False

    def pick_fixed(self, class_name, grab_roll=None):
        try:
            p = self.fixed_pick
            x_mm = float(p.get('x', 200.0))
            y_mm = float(p.get('y', 15.0))
            z_mm = float(p.get('z', 80.0))
            pitch = float(p.get('pitch', -90.0))
            roll = float(p.get('roll', 0.0))
            pre_grab_roll = float(p.get('pre_grab_roll', 0.0))
            open_claw = float(p.get('open_claw', self.OPEN_CLAW))
            close_claw = self.fixed_pick_close_claw(class_name)
            approach_z = z_mm + float(p.get('approach_z_offset', 10.0))
            lift_z = z_mm + float(p.get('lift_z_offset', 30.0))

            self.get_logger().info(f'scene5 fixed pick: class={class_name}, x={x_mm}, y={y_mm}, z={z_mm}, roll={pre_grab_roll}')
            self.publish_arm(x_mm, y_mm, approach_z, pitch, roll, open_claw, 1500)
            time.sleep(1.8)
            self.publish_arm(x_mm, y_mm, approach_z, pitch, pre_grab_roll, open_claw, 1000)
            time.sleep(1.2)
            self.publish_arm(x_mm, y_mm, z_mm, pitch, pre_grab_roll, open_claw, 800)
            time.sleep(1.0)
            self.publish_arm(x_mm, y_mm, z_mm, pitch, pre_grab_roll, close_claw, 500)
            time.sleep(0.6)
            carry_roll = 0.0 if class_name in self.COLOR_CLASSES else roll
            self.publish_arm(x_mm, y_mm, lift_z, pitch, carry_roll, close_claw, 800)
            time.sleep(1.0)
            self.publish_arm(
                float(p.get('transfer_x', 200.0)),
                float(p.get('transfer_y', 0.0)),
                float(p.get('transfer_z', 200.0)),
                pitch,
                carry_roll,
                close_claw,
                1000,
            )
            time.sleep(1.2)
            self.place_by_category(class_name)
            self.go_home(wait_time=1.5)
        finally:
            self.stable_count = 0
            self.last_world_position = None
            self.last_target_class = None
            self.last_pick_stamp = time.time()
            self.moving = False

    def main(self):
        while self.running:
            try:
                rgb_image, depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                result_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                h, w = depth_image.shape[:2]
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000.0 * 255.0
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                if not self.moving and self.start and time.time() > self.start_stamp:
                    with self.yolo_lock:
                        yolo_object = None if self.yolo_object is None else dict(self.yolo_object)
                    color_targets = [
                        target for target in self.detect_color_targets(rgb_image, result_bgr)
                        if self.color_target_allowed(target.get('class_name'))
                    ]

                    # 防止使用过期 YOLO 结果
                    if yolo_object is not None and time.time() - yolo_object['stamp'] > 0.5:
                        yolo_object = None

                    candidate_objects = []
                    if yolo_object is not None:
                        candidate_objects.append(yolo_object)
                    candidate_objects.extend(color_targets)

                    selected_object = None
                    if self.fixed_pick_enabled:
                        for target in candidate_objects:
                            target_w = target['image_width'] if target['image_width'] > 0 else float(w)
                            target_h = target['image_height'] if target['image_height'] > 0 else float(h)
                            target_center_x = int(np.clip(target['center_x'] * float(w) / target_w, 0, w - 1))
                            target_center_y = int(np.clip(target['center_y'] * float(h) / target_h, 0, h - 1))
                            if self.fixed_pick_target_in_window(target_center_x, target_center_y):
                                selected_object = target
                                break
                    if selected_object is None:
                        selected_object = candidate_objects[0] if candidate_objects else None

                    if selected_object is not None:
                        # 如果识别图像尺寸和当前深度图尺寸不同，按比例映射到深度图坐标系。
                        target_w = selected_object['image_width'] if selected_object['image_width'] > 0 else float(w)
                        target_h = selected_object['image_height'] if selected_object['image_height'] > 0 else float(h)
                        scale_x = float(w) / target_w
                        scale_y = float(h) / target_h

                        center_x = int(np.clip(selected_object['center_x'] * scale_x, 0, w - 1))
                        center_y = int(np.clip(selected_object['center_y'] * scale_y, 0, h - 1))
                        box_x = int(np.clip(selected_object['box_x'] * scale_x, 0, w - 1))
                        box_y = int(np.clip(selected_object['box_y'] * scale_y, 0, h - 1))
                        box_w = int(max(1, selected_object['box_w'] * scale_x))
                        box_h = int(max(1, selected_object['box_h'] * scale_y))
                        x1 = int(np.clip(box_x, 0, w - 1))
                        y1 = int(np.clip(box_y, 0, h - 1))
                        x2 = int(np.clip(box_x + box_w, 0, w - 1))
                        y2 = int(np.clip(box_y + box_h, 0, h - 1))
                        class_name = selected_object.get("class_name")
                        source = selected_object.get('source', 'yolo')

                        cv2.rectangle(result_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            result_bgr,
                            f"{class_name} {selected_object['score']:.2f}",
                            (x1, max(20, y1 - 5)),
                            cv2.FONT_HERSHEY_PLAIN,
                            1.5,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA
                        )

                        if self.fixed_pick_enabled:
                            image_position = [float(center_x), float(center_y), 0.0]
                            if self.last_target_class is not None and self.last_target_class != class_name:
                                self.stable_count = 0
                            else:
                                self.stable_count += 1
                            self.last_world_position = image_position
                            self.last_target_class = class_name
                            in_pick_window = self.fixed_pick_target_in_window(center_x, center_y)
                            p = self.fixed_pick
                            self.get_logger().info(
                                f'[b_pick] detected={class_name} center=({center_x:.0f},{center_y:.0f}) '
                                f'window=({float(p.get("trigger_center_x",320)):.0f}±{float(p.get("trigger_tolerance_x",160)):.0f},'
                                f'{float(p.get("trigger_center_y",280)):.0f}±{float(p.get("trigger_tolerance_y",120)):.0f}) '
                                f'in_window={in_pick_window} stable={self.stable_count}'
                            )

                            if self.stable_count > 5 and in_pick_window and time.time() - self.last_pick_stamp > 0.8:
                                self.stable_count = 0
                                self.moving = True
                                threading.Thread(target=self.pick_fixed, args=(class_name,), daemon=True).start()
                        else:
                            roi = [center_y - 5, center_y + 5, center_x - 5, center_x + 5]
                            roi[0] = max(0, roi[0])
                            roi[1] = min(h, roi[1])
                            roi[2] = max(0, roi[2])
                            roi[3] = min(w, roi[3])
                            roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
                            valid_depths = roi_distance[np.logical_and(roi_distance > 0, roi_distance < 10000)]

                            if valid_depths.size > 0 and self.hand2cam_tf_matrix is not None and self.plane is not None:
                                depth_value = float(np.median(valid_depths))
                                endpoint = self.get_endpoint()
                                pose_t = utils.calculate_world_position(
                                    center_x,
                                    center_y,
                                    depth_value,
                                    self.plane,
                                    endpoint,
                                    self.hand2cam_tf_matrix,
                                    depth_camera_info.k,
                                )
                                pose_t = self.apply_depth_calibration(list(pose_t))

                                txt = 'Dist: {}mm'.format(int(np.median(valid_depths)))
                                position_text = f'x:{pose_t[0]:.3f}m y:{pose_t[1]:.3f}m z:{pose_t[2]:.3f}m'
                                cv2.circle(result_bgr, (center_x, center_y), 5, (255, 255, 255), -1)
                                cv2.circle(depth_color_map, (center_x, center_y), 5, (255, 255, 255), -1)
                                cv2.putText(depth_color_map, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 8, cv2.LINE_AA)
                                cv2.putText(depth_color_map, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                                cv2.putText(result_bgr, position_text, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 6, cv2.LINE_AA)
                                cv2.putText(result_bgr, position_text, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2, cv2.LINE_AA)

                                if self.last_world_position is not None:
                                    if self.last_target_class is not None and self.last_target_class != class_name:
                                        self.stable_count = 0
                                    delta = math.sqrt(
                                        (self.last_world_position[0] - pose_t[0]) ** 2 +
                                        (self.last_world_position[1] - pose_t[1]) ** 2 +
                                        (self.last_world_position[2] - pose_t[2]) ** 2
                                    )
                                    if delta < 0.005:
                                        self.stable_count += 1
                                    else:
                                            self.stable_count = 0
                                self.last_world_position = pose_t
                                self.last_target_class = class_name

                                if self.stable_count > 8 and time.time() - self.last_pick_stamp > 0.8:
                                    self.stable_count = 0
                                    self.moving = True
                                    grab_roll = selected_object.get('angle') if source == 'color' else self.yolo_angle_to_arm_roll(selected_object.get('angle', 0.0))
                                    threading.Thread(target=self.pick, args=(list(pose_t), class_name, grab_roll), daemon=True).start()
                            else:
                                self.stable_count = 0
                                self.last_world_position = None
                                self.last_target_class = None
                    else:
                        self.stable_count = 0
                        self.last_world_position = None
                        self.last_target_class = None

                self.fps.update()
                result_bgr = self.fps.show_fps(result_bgr)
                result_image = np.concatenate([result_bgr, depth_color_map], axis=1)
                self.publish_preview_images(result_bgr)

                if self.should_display():
                    cv2.imshow('depth', result_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        self.running = False
                    elif key == ord('s'):
                        self.start = True
                    elif key == ord('a'):
                        self.start = False
                        self.moving = False
            except Exception as e:
                self.get_logger().info('error: ' + str(e))

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrackAndGrabNode('waste_classification_motor_depth')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
