#!/usr/bin/env python3
# coding: utf-8
"""custom_object_sorting — scene_6 detection + pick-and-place behavior.

Detects a single YOLO object class, "strawberry shortcake ice cream bar"
(swap in a real trained model, see
``src/example/example/yolo_detect/models/``) and moves every instance it
sees to one fixed place target defined in
``src/app/config/plays/scene6_custom_object_sorting.yaml``.

Modeled on the working scene_1/scene_3 pattern in ``app/waste_classification.py``:
same camera calibration files (``src/app/config/transform.yaml``,
``calibration.yaml``), and the same ``example.yolo_detect.yolo_node``
detector (see ``custom_object_sorting.launch.py``), just given its own
node name/services (``DETECT_NODE_NAME``) so it doesn't collide with
waste_classification's always-on ``yolo`` instance. Reuses the shared
pick()/place() motion helpers in ``app.utils.pick_and_place``. Trimmed to
a single class and a single destination, so it skips the multi-target
coordinator/heartbeat machinery the bigger multi-class nodes need.
"""

import os
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo
from interfaces.msg import ObjectsInfo
from ros_robot_controller_msgs.msg import ArmCoords

from sdk import common
from app import calibrated_pose, scene4_runtime, scene_play_registry
from app.utils import pick_and_place

SCENE_ID = 'scene_6'
DETECT_CLASS = 'strawberry shortcake ice cream bar'
# Must match custom_object_sorting.launch.py's yolo_node `name=` and
# start_service/stop_service -- the shared yolo_node executable defaults to
# node name 'yolo' and services '/yolo/start'|'/yolo/stop', which collide
# with waste_classification.launch.py's always-on yolo_node instance.
DETECT_NODE_NAME = 'strawberry_shortcake_detect'
OBJECT_HEIGHT_M = 0.03
PICK_PITCH_DEG = 80.0
PICK_GRIPPER_ANGLE = 500
PICK_GRIPPER_DEPTH_M = 0.02
DEFAULT_PLACE_TARGET = [0.15, 0.15, 0.02]
MIN_DETECT_SCORE = 0.5


class CustomObjectSortingNode(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.enter = False
        self.busy = False
        self.lock = threading.RLock()

        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.scene_config_path = scene4_runtime.scene_config_path()
        self.config_path = os.path.dirname(self.scene_config_path) + "/"
        self.play_config_path = self.get_string_param('play_config_path', '')

        self.intrinsic = None
        self.distortion = None
        self.extristric = None
        self.white_area_center = None

        self.place_target = self._load_place_target()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.start_yolo_client = self.create_client(
            Trigger, f'/{DETECT_NODE_NAME}/start', callback_group=self.timer_cb_group)
        self.stop_yolo_client = self.create_client(
            Trigger, f'/{DETECT_NODE_NAME}/stop', callback_group=self.timer_cb_group)
        self.controller_init_client = self.create_client(
            Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)

        self.camera_info_sub = None
        self.object_sub = None

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None and value != '':
                return str(value)
        except Exception:
            pass
        return str(default)

    def send_request(self, client, msg, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'service {client.srv_name} unavailable')
            return None
        future = client.call_async(msg)
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        return future.result()

    # -- lifecycle -----------------------------------------------------

    def _load_place_target(self):
        try:
            cfg = scene_play_registry.load_play_config(SCENE_ID, self.play_config_path or None)
            target = (cfg.get('place_targets') or {}).get(DETECT_CLASS)
            if target and len(target) == 3:
                return [float(v) for v in target]
        except Exception as e:
            self.get_logger().warn(f'failed to load {SCENE_ID} place config, using default: {e}')
        return list(DEFAULT_PLACE_TARGET)

    def init_process(self):
        self.timer.cancel()
        self.get_logger().info('waiting for arm controller...')
        self.controller_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.05)
        home = pick_and_place.load_scene_home_pose()
        pick_and_place.publish_arm(
            self.arm_pub, home['x'], home['y'], home['z'],
            home['pitch'], home['roll'], home.get('claw', 0.0),
            int(home.get('time_ms', 2000)))
        time.sleep(max(1.0, home.get('time_ms', 2000) / 1000.0))
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        if bool(self.get_parameter('start').value):
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
        self.get_logger().info('\033[1;32m%s\033[0m' % f'{SCENE_ID} (custom_object_sorting) ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def enter_srv_callback(self, request, response):
        if not self.enter:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, 'depth_cam/rgb/camera_info', self.camera_info_callback, 1)
            self.object_sub = self.create_subscription(
                ObjectsInfo, f'/{DETECT_NODE_NAME}/object_detect', self.object_callback, 1)
            self.enter = True
            threading.Thread(target=self.get_roi, daemon=True).start()
        self.send_request(self.start_yolo_client, Trigger.Request())
        response.success = True
        response.message = 'entered scene_6'
        return response

    def exit_srv_callback(self, request, response):
        self.send_request(self.stop_yolo_client, Trigger.Request())
        if self.object_sub is not None:
            self.destroy_subscription(self.object_sub)
            self.object_sub = None
        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None
        self.enter = False
        response.success = True
        response.message = 'exited scene_6'
        return response

    # -- calibration -----------------------------------------------------

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def get_roi(self):
        config = common.get_yaml_data(os.path.join(self.config_path, self.config_file)) or {}
        extristric = np.array(config.get('extristric', []))
        white_area_center = np.array(config.get('white_area_pose_world', []))
        if extristric.size == 0 or white_area_center.size == 0:
            self.get_logger().error(
                f'{self.config_path}{self.config_file} is missing extristric/white_area_pose_world; '
                'run the calibration GUI (software/calibration) before using scene_6.')
            return
        self.white_area_center = white_area_center
        while self.intrinsic is None or self.distortion is None:
            time.sleep(0.1)
        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), OBJECT_HEIGHT_M)
        self.extristric = (tvec, rmat)

    def get_object_world_position(self, pixel, height=OBJECT_HEIGHT_M):
        calibration = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
        return calibrated_pose.pixel_to_calibrated_world(
            pixel, self.intrinsic, self.extristric, self.white_area_center, calibration, height=height)

    @staticmethod
    def _position_yaw(position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw += 180
        elif position[0] < 0 and position[1] > 0:
            yaw -= 180
        return yaw

    # -- detection -> pick/place -----------------------------------------------------

    def object_callback(self, msg):
        if self.busy or self.extristric is None:
            return
        best = None
        for obj in msg.objects:
            if obj.class_name != DETECT_CLASS or obj.score < MIN_DETECT_SCORE:
                continue
            if best is None or obj.score > best.score:
                best = obj
        if best is None:
            return
        x1, y1, x2, y2 = best.box
        center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        with self.lock:
            if self.busy:
                return
            self.busy = True
        threading.Thread(target=self._pick_and_place, args=(center,), daemon=True).start()

    def _pick_and_place(self, pixel_center):
        try:
            self.send_request(self.stop_yolo_client, Trigger.Request())
            position, _ = self.get_object_world_position(pixel_center)
            position = np.asarray(position, dtype=np.float64).tolist()
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file)) or {}
            position = calibrated_pose.apply_axis_calibration(position, calibration, 'kinematics').tolist()
            yaw = self._position_yaw(position)

            picked = pick_and_place.pick(
                position, PICK_PITCH_DEG, yaw, PICK_GRIPPER_ANGLE, PICK_GRIPPER_DEPTH_M, self.arm_pub)
            if picked:
                place_yaw = self._position_yaw(self.place_target)
                pick_and_place.place(
                    self.place_target, PICK_PITCH_DEG, place_yaw, PICK_GRIPPER_ANGLE, self.arm_pub)
                self.get_logger().info(f'placed {DETECT_CLASS} at {self.place_target}')
            else:
                self.get_logger().warn(f'pick failed for {DETECT_CLASS} at {position}')
        except Exception as e:
            self.get_logger().error(f'pick/place error: {e}')
        finally:
            with self.lock:
                self.busy = False
            if self.enter:
                self.send_request(self.start_yolo_client, Trigger.Request())


def main():
    rclpy.init()
    node = CustomObjectSortingNode('custom_object_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
