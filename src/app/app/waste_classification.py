# #!/usr/bin/env python3
# # encoding: utf-8
# # 垃圾分类

import os
import cv2
import time
import math
import copy
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sdk import common
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetStringList
from ros_robot_controller_msgs.msg import ArmCoords
from app import calibrated_pose, scene4_runtime, scene_play_registry, sorting_coordinator
from app.play_pose import load_play_home_pose
from app.utils import calculate_grasp_yaw, pick_and_place, distortion_inverse_map

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}
DEFAULT_PLACE_POLICY = {
    'only_left_y_positive': True,
    'min_place_z': 0.015,
}
WASTE_CARD_HEIGHT_M = 0.04

class WasteClassificationNode(Node):
    place_position = {
        'residual_waste': [0.095, -0.214, 0.02],
        'food_waste': [0.040, -0.214, 0.02],
        'hazardous_waste': [-0.018, -0.214, 0.02],
        'recyclable_waste': [-0.07, -0.214, 0.02]
    }

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.grasp_finish = True
        self._init_parameters()
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.scene_config_path = scene4_runtime.scene_config_path()
        self.config_path = os.path.dirname(self.scene_config_path) + "/"
        self.play_config_path = self.get_string_param('play_config_path', '')
        self.home_pose = self._load_home_pose_from_scene()
        self.camera_type = os.environ['CAMERA_TYPE']
        self.classes =  ['BananaPeel','BrokenBones','CigaretteEnd','DisposableChopsticks','Ketchup',
                         'Marker','OralLiquidBottle','PlasticBottle','Plate','StorageBattery','Toothbrush', 'Umbrella']
        self.target_1 = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.yolo_image_queue = queue.Queue(maxsize=2)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.known_pose = {
            'x': float(self.home_pose['x']),
            'y': float(self.home_pose['y']),
            'z': float(self.home_pose['z']),
            'pitch': float(self.home_pose['pitch']),
            'roll': float(self.home_pose['roll']),
        }
        self.timer_cb_group = ReentrantCallbackGroup()
        self.scene4_stepper_client = scene4_runtime.create_stepper_position_client(self, self.timer_cb_group)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_srv = self.create_service(SetBool, '~/enable_transport', self.start_srv_callback)
        self.create_service(SetStringList, '~/set_target', self.set_target_srv_callback)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        self.start_yolo_client = self.create_client(Trigger, 'yolo/start', callback_group=self.timer_cb_group)
        self.start_yolo_client.wait_for_service()
        self.stop_yolo_client = self.create_client(Trigger, 'yolo/stop', callback_group=self.timer_cb_group)
        self.stop_yolo_client.wait_for_service()
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish', callback_group=self.timer_cb_group)
        self.scene_runtime_prepare_client = self.create_client(
            Trigger,
            '/ros_robot_controller/scene_runtime/prepare',
            callback_group=self.timer_cb_group,
        )
        self.kinematics_client = None
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_string_param(self, name, default=''):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

    def _init_parameters(self):
        self.heart = None
        self.target_list_temp = []
        self.target_list = []
        self.start_transport = False
        self.enable_transport = False
        self.waste_category = None
        self.count_move = 0
        self.count_still = 0
        self.count_miss = 0
        self.last_position = None
        self.start_get_roi = False
        self.target_object_info = None
        self.intrinsic = None
        self.distortion = None
        self.extristric = None
        self.white_area_center = None
        self.roi = None
        self.enter = False
        self.image_sub = None
        self.yolo_image_sub = None
        self.object_sub = None
        self.camera_info_sub = None
        self.display = self.get_parameter('display').value
        self.app = self.get_parameter('app').value
        self.static_start_time = None
        self.grasping = False
        self.target_lost_time = None
        self.sort_claim_target = None
        self.last_yolo_image = None
        self.last_yolo_image_time = 0.0

    def _release_sort_claim(self):
        if self.sort_claim_target is not None:
            sorting_coordinator.release_claim(sorting_coordinator.WASTE_GROUP, self.sort_claim_target)
            self.sort_claim_target = None

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        if self.get_parameter('start').value:
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            req = SetBool.Request()
            req.data = True 
            self.start_srv_callback(req, SetBool.Response())
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _load_home_pose_from_scene(self):
        return load_play_home_pose(
            self.scene_config_path,
            {
                'x': 200.0,
                'y': 0.0,
                'z': 200.0,
                'pitch': -90.0,
                'roll': 0.0,
                'claw': -60.0,
            },
            default_scene='scene_0',
        )

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
        self.known_pose = {'x': float(x), 'y': float(y), 'z': float(z), 'pitch': float(pitch), 'roll': float(roll)}

    def go_home(self, interrupt=True):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        if self.waste_category == "recyclable_waste":
            t = 2.5
        elif self.waste_category == "hazardous_waste":
            t = 2.2
        elif self.waste_category == "food_waste":
            t = 1.8
        elif self.waste_category == "residual_waste":
            t = 1.5
        else:
            t = 1.5
        if interrupt:
            self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], 30, 1000)
            time.sleep(1.0)
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1500)
        time.sleep(1.5)
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], int(t * 1000))
        time.sleep(t)

    def _load_active_scene_config(self):
        try:
            cfg = common.get_yaml_data(self.scene_config_path) or {}
        except Exception:
            cfg = {}
        scene_name, scene_cfg = scene4_runtime.active_scene_from_data(cfg)
        play_cfg = scene_play_registry.load_play_config(scene_name, self.play_config_path or None)
        return scene_name, scene_play_registry.merge_play_into_scene(scene_name, scene_cfg, play_cfg)

    def _sorting_priority(self):
        scene_name, scene_cfg = self._load_active_scene_config()
        return scene_name, sorting_coordinator.priority_from_scene_config(scene_cfg)

    def _waste_category_for_class(self, class_name):
        for category, classes in WASTE_CLASSES.items():
            if class_name in classes:
                return category
        return None

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

    def _configure_scene4_observation_pose(self):
        scene_name, scene_cfg = self._load_active_scene_config()
        if scene_name == scene4_runtime.SCENE4_ID:
            pose = scene4_runtime.scene4_observation_pose(scene_cfg)
            rail = scene4_runtime.scene4_rail_config(scene_cfg)
            self.get_logger().info(
                '[Scene4Waste] enter mode: '
                f'rail_calibration={rail["calibration_abs_position"]}, '
                f'view_pose=({pose["x"]:.1f},{pose["y"]:.1f},{pose["z"]:.1f},'
                f'{pose["pitch"]:.1f},{pose["roll"]:.1f})'
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

    def _resolve_scene4_shelf_place(self, target_key, place_position):
        scene_name, scene_cfg = self._load_active_scene_config()
        if scene_name != scene4_runtime.SCENE4_ID:
            return None
        try:
            return scene4_runtime.scene4_shelf_place(scene_cfg, target_key, place_position)
        except Exception as exc:
            self.get_logger().warn(f'[Scene4WastePlace] resolve shelf place failed: target={target_key}, error={exc}')
            return None

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
        raw = place_targets.get(target_key, self.place_position.get(target_key))
        if raw is None:
            return None
        try:
            pos = [float(raw[0]), float(raw[1]), float(raw[2])]
        except Exception:
            fallback = self.place_position.get(target_key)
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

    def resolve_place_roll(self, target_key, default_roll=None):
        _, scene_cfg = self._load_active_scene_config()
        return scene_play_registry.resolve_place_roll(scene_cfg, target_key, default_roll)

    def scene1_pick_hold_yaw(self, position, yaw, target_key):
        roll_deg = float(yaw)
        self.get_logger().info(f'[Scene1Pick] hold yaw pick: target={target_key}, yaw={roll_deg:.1f}°')
        finish = pick_and_place.pick_without_back(
            position,
            80,
            roll_deg,
            470,
            0.02,
            self.arm_pub,
            None,
            claw_grab_angle=-45.0,
        )
        if not finish or getattr(pick_and_place, 'stop', False):
            return False
        hp = self._load_home_pose_from_scene()
        try:
            move_ms = min(int(float(hp.get('time_ms', 1000))), 1000)
        except Exception:
            move_ms = 1000
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], roll_deg, -30.0, move_ms)
        time.sleep(max(0.0, float(move_ms) / 1000.0) + 0.1)
        return not getattr(pick_and_place, 'stop', False)

    # 场景3放置坐标（固定，不依赖yaml）x,y,z单位米，pitch单位度
    SCENE3_PLACE_POSES = {
        'food_waste':       (0.305, -0.19,  0.18,  -43.0),
        'recyclable_waste': (0.195, -0.205, 0.18,  -43.0),
        'hazardous_waste':  (0.085, -0.21,  0.185, -45.0),
        'residual_waste':   (-0.05, -0.21,  0.18,  -45.0),
        'yellow':           (-0.08,  0.29,  0.065, -65.0),
        'red':              (0.09,   0.29,  0.06,  -65.0),
        'green':            (0.29,   0.30,  0.07,  -65.0),
        'blue':             (0.185,  0.30,  0.06,  -65.0),
    }

    def _scene3_direct_place(self, target_key):
        if getattr(pick_and_place, 'stop', False):
            return False
        pose = self.SCENE3_PLACE_POSES.get(target_key)
        if pose is None:
            self.get_logger().warn(f'[Scene3Place] unknown target: {target_key}')
            return False
        x_mm, y_mm, z_mm, pitch_deg = pose[0]*1000, pose[1]*1000, pose[2]*1000, pose[3]
        self.get_logger().info(
            f'[Scene3Place] target={target_key}, pos=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}), pitch={pitch_deg:.1f}, roll=0'
        )
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, 0.0, -45.0, 1500)
        time.sleep(2.0)
        if getattr(pick_and_place, 'stop', False):
            return False
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, 0.0, pick_and_place.CLAW_OPEN, 400)
        time.sleep(0.5)
        self.publish_arm(x_mm, y_mm, z_mm + 50.0, pitch_deg, 0.0, pick_and_place.CLAW_OPEN, 800)
        time.sleep(1.0)
        return True

    def scene1_direct_place(self, position, pitch, roll, target_key):
        if getattr(pick_and_place, 'stop', False):
            return False
        x_mm = float(position[0]) * 1000.0
        y_mm = float(position[1]) * 1000.0
        z_mm = float(position[2]) * 1000.0
        pitch_deg = -abs(float(pitch))
        roll_deg = float(roll)
        claw_hold = -45.0
        self.get_logger().info(
            f'[Scene1Place] direct place: target={target_key}, '
            f'pos=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}), pitch={pitch_deg:.1f}, roll={roll_deg:.1f}'
        )
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, roll_deg, claw_hold, 800)
        time.sleep(0.9)
        if getattr(pick_and_place, 'stop', False):
            return False
        self.publish_arm(x_mm, y_mm, z_mm, pitch_deg, roll_deg, pick_and_place.CLAW_OPEN, 300)
        time.sleep(0.4)
        self.publish_arm(x_mm, y_mm, z_mm + 30.0, pitch_deg, roll_deg, pick_and_place.CLAW_OPEN, 500)
        time.sleep(0.6)
        return True

    def _execute_scene4_shelf_place(self, target_key, shelf_place):
        rail_position = int(shelf_place['rail_position'])
        pose = dict(shelf_place['pose'])
        approach_pose = shelf_place.get('approach_pose')
        if approach_pose is not None:
            approach_pose = dict(approach_pose)
        move_ms = int(pose.get('time_ms', 2000))
        destination = shelf_place.get('destination', 'shelf')
        claw_hold = float(shelf_place.get('claw_hold', pick_and_place.CLAW_GRAB))

        self.get_logger().info(
            '[Scene4WastePlace] fixed shelf place: '
            f'target={target_key}, destination={destination}, rail={rail_position}, '
            f'pose=({pose["x"]:.1f},{pose["y"]:.1f},{pose["z"]:.1f},{pose["pitch"]:.1f}), '
            f'claw_hold={claw_hold:.1f}'
        )
        claw_open = pick_and_place.CLAW_OPEN
        scene4_runtime.publish_scene4_transfer_pose(self.publish_arm, claw_hold)
        if not scene4_runtime.move_scene4_rail_to_position(
            self,
            self.scene4_stepper_client,
            rail_position,
            scene_path=self.scene_config_path,
            logger=self.get_logger(),
            reset_first=False,
        ):
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
        scene4_runtime.publish_scene4_transfer_pose(self.publish_arm, claw_open)
        self.return_to_observation_pose(False)
        return True

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def get_roi(self):
        with open(os.path.join(self.config_path, self.config_file), 'r') as f:
            config = common.get_yaml_data(os.path.join(self.config_path, self.config_file))
            extristric = np.array(config['extristric'])
            self.white_area_center = np.array(config['white_area_pose_world'])
            corners = np.array(config['corners']).reshape(-1, 3)
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)
        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3,1)), np.array(rmat), WASTE_CARD_HEIGHT_M)
        extristric = tvec, rmat
        self.extristric = extristric
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3,1)), np.array(rmat), 0.05)
        imgpts, _ = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        x_min = min(imgpts, key=lambda p:p[0])[0]
        x_max = max(imgpts, key=lambda p:p[0])[0]
        y_min = min(imgpts, key=lambda p:p[1])[1]
        y_max = max(imgpts, key=lambda p:p[1])[1]
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
        self.roi = roi

    def enter_srv_callback(self, request, response):
        ok, msg = self.prepare_scene_runtime()
        if not ok:
            response.success = False
            response.message = msg
            return response
        self._init_parameters()
        self.home_pose = self._configure_scene4_observation_pose()
        scene_name, scene_cfg = self._load_active_scene_config()
        sorting_coordinator.start_session(
            scene_name,
            sorting_coordinator.priority_from_scene_config(scene_cfg),
        )
        hp = self.home_pose
        move_ms = int(hp.get('time_ms', 2000))
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], move_ms)
        time.sleep(max(0.0, move_ms / 1000.0) + 0.3)
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))
        self.camera_info_sub = self.create_subscription(CameraInfo, 'depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.yolo_image_sub = self.create_subscription(Image, 'yolo/object_image', self.yolo_image_callback, 1)
        self.object_sub = self.create_subscription(ObjectsInfo, 'yolo/object_detect', self.get_object_callback, 1)
        # self.start_yolo_client.call_async(Trigger.Request())
        self.send_request(self.start_yolo_client, Trigger.Request())   # 在enter时启动YOLOv8检测
        self.get_logger().info('\033[1;32m%s\033[0m' % 'enter app')
        self.enter = True
        self.start_get_roi = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.destroy_subscription(self.yolo_image_sub)
                self.destroy_subscription(self.object_sub)
                self.destroy_subscription(self.camera_info_sub)
                self.image_sub = None
                self.object_sub = None
                self.camera_info_sub = None
            self.send_request(self.stop_yolo_client, Trigger.Request())  # 退出时停止YOLOv8检测
            self.heart.destroy()
            self.heart = None
            pick_and_place.interrupt(True)
            self.enter = False
            self.start_transport = False
            self._release_sort_claim()
        response.success = True
        response.message = "exit"
        return response

    def start_srv_callback(self, request, response):
        if request.data:
            if self.app:
                target_list = []
                for category in WASTE_CLASSES.values():
                    target_list.extend(category)
                self.target_list = target_list
                self.target_list_temp = copy.deepcopy(self.target_list)
            pick_and_place.interrupt(False)
            self.enable_transport = True
            response.message = "start"
        else:
            pick_and_place.interrupt(True)
            self.enable_transport = False
            self.start_transport = False
            self._release_sort_claim()
            response.message = "stop"
        response.success = True
        return response


    def set_target_srv_callback(self, request, response):
        target_list = []
        for i in request.data:
            target_list.extend(list(WASTE_CLASSES.get(i, [])))
        self.target_list = target_list
        self.target_list_temp = copy.deepcopy(self.target_list)
        response.success = True
        response.message = "set target"
        return response

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                position, yaw, target = self.transport_info
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                kinematics_cfg = config_data.get('kinematics', {}) if isinstance(config_data, dict) else {}
                offset = tuple(float(v) for v in kinematics_cfg.get('offset', (0.0, 0.0, 0.0)))
                scale = tuple(float(v) for v in kinematics_cfg.get('scale', (1.0, 1.0, 1.0)))
                pos_raw = [round(p, 4) for p in position]
                position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics').tolist()
                self.get_logger().info(
                    f'[PickPos] raw={pos_raw}, '
                    f'kinematics_offset={list(offset)}, kinematics_scale={list(scale)}, '
                    f'after_calibration={[round(p,4) for p in position]}'
                )
                self.get_logger().info(f'pick: pos={[round(p,4) for p in position]}, yaw={yaw:.1f}°, target={target}')
                scene_name, _ = self._load_active_scene_config()
                if scene_name == scene_play_registry.SCENE1_ID:
                    finish = self.scene1_pick_hold_yaw(position, yaw, target)
                else:
                    finish = pick_and_place.pick(position, 90, yaw, 470, 0.02, self.arm_pub, None,
                                                 claw_grab_angle=-45.0)
                if finish:
                    place_pos = self.resolve_place_position(target)
                    if place_pos is None:
                        self.get_logger().error(f'no waste place position configured for target: {target}')
                        self.go_home(True)
                        if self.enter:
                            if self.app:
                                self.target_list = copy.deepcopy(self.target_list_temp)
                            self.waste_category = None
                            self.start_transport = False
                            self.static_start_time = None
                            self.grasping = False
                            self.target_object_info = None
                        continue
                    scene_name, scene_cfg = self._load_active_scene_config()
                    if scene_name == scene4_runtime.SCENE4_ID:
                        fixed_position = scene4_runtime.scene4_fixed_place_position(scene_cfg, target)
                        if fixed_position is not None:
                            place_pos = scene_play_registry.apply_global_place_offset(
                                fixed_position,
                                self.scene_config_path,
                            )
                    shelf_place = self._resolve_scene4_shelf_place(target, place_pos)
                    if shelf_place is not None:
                        self._execute_scene4_shelf_place(target, shelf_place)
                        if self.enter:
                            if self.app:
                                self.target_list = copy.deepcopy(self.target_list_temp)
                            self.waste_category = None
                            self.start_transport = False
                            self.static_start_time = None
                            self.grasping = False
                            self.target_object_info = None
                        continue
                    scene4_active = scene_name == scene4_runtime.SCENE4_ID
                    if scene4_active:
                        if not scene4_runtime.move_scene4_rail(
                            self,
                            self.scene4_stepper_client,
                            "place",
                            scene_path=self.scene_config_path,
                            logger=self.get_logger(),
                        ):
                            self.return_to_observation_pose(True)
                            if self.enter:
                                if self.app:
                                    self.target_list = copy.deepcopy(self.target_list_temp)
                                self.waste_category = None
                                self.start_transport = False
                                self.static_start_time = None
                                self.grasping = False
                                self.target_object_info = None
                            continue
                    place_pitch = self.resolve_place_pitch(target, 80.0)
                    if scene_name == scene_play_registry.SCENE3_ID:
                        finish = self._scene3_direct_place(target)
                    else:
                        place_roll = self.resolve_place_roll(target, None)
                        yaw = place_roll if place_roll is not None else self.calculate_place_grasp_yaw(place_pos, 0)
                        angle = math.degrees(math.atan2(place_pos[1], place_pos[0]))
                        if angle > 45:
                            place_pos = [place_pos[0]*scale[1] + offset[1], place_pos[1]*scale[0] + offset[0], place_pos[2]*scale[2] + offset[2]]
                        elif angle < -45:
                            place_pos = [place_pos[0]*scale[1] + offset[1], place_pos[1]*scale[0] - offset[0], place_pos[2]*scale[2] + offset[2]]
                        else:
                            place_pos = [place_pos[0]*scale[0] + offset[0], place_pos[1]*scale[1] + offset[1], place_pos[2]*scale[2] + offset[2]]
                        if scene_name == scene_play_registry.SCENE1_ID:
                            finish = self.scene1_direct_place(place_pos, place_pitch, yaw, target)
                        else:
                            finish = pick_and_place.place(place_pos, place_pitch, yaw, 200, self.arm_pub, None,
                                                          claw_hold_angle=-45.0)
                    if finish:
                        if scene4_active:
                            self.return_to_observation_pose(False)
                        else:
                            self.go_home(False)
                    else:
                        if scene4_active:
                            self.return_to_observation_pose(True)
                        else:
                            self.go_home(True)
                else:
                    self.go_home(True)
                if self.enter:
                    if self.app:
                        self.target_list = copy.deepcopy(self.target_list_temp)
                    self.waste_category = None
                    self.start_transport = False
                    self.static_start_time = None
                    self.grasping = False
                    self.target_object_info = None
                    self._release_sort_claim()
            else:
                time.sleep(0.1)

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=WASTE_CARD_HEIGHT_M):
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
            yaw += 180
        elif position[0] < 0 and position[1] > 0:
            yaw -= 180
        gripper_size = [common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
                        common.calculate_pixel_length(0.02, intrinsic, projection_matrix)]
        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info, gripper_size, yaw)

    def calculate_place_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw += 180
        elif position[0] < 0 and position[1] > 0:
            yaw -= 180
        yaw1 = yaw + angle
        yaw2 = yaw1 + (90 if yaw < 0 else -90)
        yaw = yaw1 if abs(yaw1) < abs(yaw2) else yaw2
        self.get_logger().info(f'放置 yaw 角度: {yaw:.1f}°')
        return yaw  # 直接返回角度值(度)

    def get_display_image(self, fallback):
        """优先取 YOLO 带检测框的画面，没有则用原始画面"""
        now = time.monotonic()
        try:
            image = self.yolo_image_queue.get_nowait()
            self.last_yolo_image = image
            self.last_yolo_image_time = now
            return image
        except queue.Empty:
            if self.last_yolo_image is not None and now - self.last_yolo_image_time <= 0.8:
                return self.last_yolo_image
            return fallback

    def main(self):
        while self.running:
            if self.enter:
                if self.start_get_roi:
                    self.get_roi()
                    self.start_get_roi = False
                try:
                    bgr_image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue
                roi = self.roi
                display_image = self.get_display_image(bgr_image)
                if self.grasping and self.start_transport:
                    if self.display:
                        cv2.imshow('image', display_image)
                        cv2.waitKey(1)
                    else:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
                    time.sleep(0.01)
                    continue
                if roi is not None:
                    if self.target_object_info is not None and self.target_1 is not None:
                        target_object_info = copy.deepcopy(self.target_object_info)
                        center = target_object_info[0][2]
                        if self.camera_type == 'usb_cam':
                            x, y = distortion_inverse_map.undistorted_to_distorted_pixel(center[0], center[1], self.intrinsic, self.distortion)
                            center = (x, y)
                        intrinsic = self.intrinsic
                        if roi[2] < center[0] < roi[3] and roi[0] < center[1] < roi[1]:
                            position, projection_matrix = self.get_object_world_position(target_object_info[0][2], intrinsic, self.extristric, self.white_area_center, WASTE_CARD_HEIGHT_M)
                            result = self.calculate_pick_grasp_yaw(position, target_object_info[0], target_object_info[1], intrinsic, projection_matrix)
                            if result is not None:
                                cv2.line(display_image, result[1][0], result[1][1], (0, 255, 255), 2, cv2.LINE_AA)
                            if result is not None and not self.grasping:
                                if self.last_position is not None:
                                    e_distance = math.sqrt(pow(self.last_position[0] - position[0], 2) + pow(self.last_position[1] - position[1], 2))
                                    if e_distance <= 0.005:
                                        if self.static_start_time is None:
                                            self.static_start_time = time.time()
                                        elif time.time() - self.static_start_time >= 1.5:
                                            waste_category = self._waste_category_for_class(target_object_info[0][0])
                                            if waste_category is not None:
                                                scene_name, sort_priority = self._sorting_priority()
                                                claimed, claim_msg = sorting_coordinator.try_claim(
                                                    sorting_coordinator.WASTE_GROUP,
                                                    waste_category,
                                                    scene_name,
                                                    sort_priority,
                                                )
                                                if claimed:
                                                    self.sort_claim_target = waste_category
                                                    self.grasping = True
                                                    self.waste_category = waste_category
                                                    raw_yolo_angle = float(target_object_info[0][5]) if len(target_object_info[0]) > 5 else float(target_object_info[0][4])
                                                    yolo_angle = float(target_object_info[0][4])
                                                    yaw = result[0]  # 已经是角度值(度)
                                                    self.get_logger().info(
                                                        f'[WastePickAngle] class={target_object_info[0][0]}, '
                                                        f'raw_angle={raw_yolo_angle:.1f}°, '
                                                        f'transformed_angle={yolo_angle:.1f}°, '
                                                        f'pick_yaw={yaw:.1f}°'
                                                    )
                                                    self.get_logger().info(f'抓取 yaw 角度: {yaw:.1f}°')
                                                    self.transport_info = [position, yaw, self.waste_category]
                                                    self.start_transport = True
                                                else:
                                                    self.get_logger().info(f'[SortPriority] skip waste target={waste_category}: {claim_msg}')
                                                    self.static_start_time = None
                                                    self.last_position = None
                                    else:
                                        self.static_start_time = None
                                self.last_position = position
                        else:
                            self.static_start_time = None
                            self.last_position = None
                    else:
                        self.static_start_time = None
                        self.last_position = None
                        if self.enable_transport:
                            self.count_miss += 1
                            if self.count_miss > 2:
                                self.target_list = copy.deepcopy(self.target_list_temp)
                                self.count_miss = 0
                        time.sleep(0.02)
                    if self.display:
                        cv2.imshow('image', display_image)
                        cv2.waitKey(1)
                    else:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
                else:
                    time.sleep(0.02)
                if self.grasping and not self.start_transport:
                    self.grasping = False
                    self.target_lost_time = None
                    self.static_start_time = None
                    self.last_position = None
                    if self.app:
                        self.target_list = copy.deepcopy(self.target_list_temp)
                    self.waste_category = None
                    self.target_object_info = None
                    self._release_sort_claim()
            else:
                time.sleep(0.1)

    def get_object_callback(self, msg):
        objects = msg.objects
        if not self.enable_transport:
            return
        local_objects_list = []
        candidates = []
        for i in objects:
            raw_angle = float(i.angle)
            if i.angle < 0:
                i.angle = 90 - abs(i.angle)
            target = [i.class_name, 0, (int(i.box[0]), int(i.box[1])), (int(i.box[2]), int(i.box[3])), i.angle, raw_angle]
            if i.class_name in self.target_list:
                category = self._waste_category_for_class(i.class_name)
                if category is not None:
                    candidates.append((category, target, float(getattr(i, 'score', 0.0)), list(i.box)))
            local_objects_list.append(target)

        scene_name, sort_priority = self._sorting_priority()
        sorting_coordinator.report_detections(
            sorting_coordinator.WASTE_GROUP,
            [item[0] for item in candidates],
            scene_name,
            sort_priority,
        )
        if candidates:
            candidates = sorting_coordinator.sort_items(candidates, lambda item: item[0], sort_priority)
            _category, local_object_info, score, box = candidates[0]
            self.target_1 = [score, box]
            # 确保 box 有 5 个元素 [cx, cy, w, h, angle]，缺少 angle 时补 0
            if len(box) == 4:
                self.target_1[1] = list(box) + [local_object_info[4]]
            self.target_object_info = copy.deepcopy([local_object_info, local_objects_list])
            self.target_lost_time = None
        else:
            if self.target_lost_time is None:
                self.target_lost_time = time.time()
            elif time.time() - self.target_lost_time > 0.5:
                self.target_object_info = None
                self.target_lost_time = None

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    def yolo_image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.yolo_image_queue.full():
            self.yolo_image_queue.get()
        self.yolo_image_queue.put(bgr_image)

def main():
    rclpy.init()
    node = WasteClassificationNode('waste_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()

if __name__ == "__main__":
    main()
