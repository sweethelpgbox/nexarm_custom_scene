#!/usr/bin/env python3
# encoding: utf-8
# 目标跟踪 — 统一使用 ArmCoords 坐标控制

import cv2
import time
import queue
import os
import yaml
import rclpy
import threading
import numpy as np
import sdk.fps as fps
from rclpy.node import Node
from app.common import Heart
from app.play_pose import get_use_scene_pose, load_play_home_pose
from cv_bridge import CvBridge
import app.face_tracker as face_tracker
import app.color_tracker as color_tracker
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image
from interfaces.srv import SetFloat64
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState

INIT_X = 110.0
INIT_Y = 0.0
INIT_Z = 220.0
INIT_PITCH = 0.0
INIT_ROLL = 0.0
INIT_CLAW = -82.5
TRACK_TIME_MS = 50
TRACK_PUBLISH_INTERVAL = 0.06
MIN_MOVE_DELTA_MM = 1.5


class ObjectTrackingNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.tracker = None
        self.enable_color_tracking = False
        self.enable_face_tracking = False
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.image_queue = queue.Queue(maxsize=2)
        self.thread_started = True
        self.thread = None
        self.image_sub = None
        self.target_color_name = ''
        self.last_track_publish_time = 0.0
        self.last_track_pose = None
        self.current_pose = None
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        if self.chassis_type == 'Slide_Rails':
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
        else:
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
        self.home_pose = self._load_home_pose_from_scene()

        self.arm_pub = self.create_publisher(ArmCoords, 'ros_robot_controller/arm/set_coords', 5)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_detect_srv = self.create_service(SetBool, '~/enable_color_tracking', self.enable_color_srv_callback)
        self.enable_transport_srv = self.create_service(SetBool, '~/enable_face_tracking', self.enable_face_srv_callback)
        self.set_target_color_srv = self.create_service(SetFloat64, '~/set_target_color', self.set_target_color_srv_callback)

        self.controller_init_client = self.create_client(Trigger, 'controller_manager/init_finish')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client = self.create_client(Trigger, 'kinematics/init_finish')
        self.kinematics_init_client.wait_for_service()

        if self.get_parameter('start').value:
            threading.Thread(target=self.enter_srv_callback, args=(Trigger.Request(), Trigger.Response()), daemon=True).start()
            time.sleep(1.0)
            req = SetBool.Request()
            req.data = True
            self.enable_color_srv_callback(req, SetBool.Response())

        if not self.get_parameter('broadcast').value:
            req = SetFloat64.Request()
            req.data = 1.0
            self.set_target_color_srv_callback(req, SetFloat64.Response())

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

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
        }

    def _load_scene_config(self):
        try:
            with open(self.scene_config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            cfg = {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            scenes = {'scene_1': {}}
        scene_name = str(cfg.get('current_scene', 'scene_1'))
        if scene_name not in scenes:
            scene_name = next(iter(scenes.keys()))
        return scenes.get(scene_name, {})

    def _load_home_pose_from_scene(self):
        return load_play_home_pose(
            self.scene_config_path,
            {
                'x': INIT_X,
                'y': INIT_Y,
                'z': INIT_Z,
                'pitch': INIT_PITCH,
                'roll': INIT_ROLL,
                'claw': INIT_CLAW,
            },
            pitch_override=INIT_PITCH,
            use_scene=get_use_scene_pose(self),
        )

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

    def goback(self):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        self.last_track_pose = None
        self.last_track_publish_time = 0.0
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1500)
        self.wait_until_home_reached(hp)

    def wait_until_home_reached(self, home_pose, timeout_sec=3.0):
        deadline = time.time() + float(timeout_sec)
        saw_state = False
        while time.time() < deadline:
            pose = self.current_pose
            if pose is not None:
                saw_state = True
                if (
                    abs(pose['x'] - float(home_pose['x'])) < 10.0
                    and abs(pose['y']) < 5.0
                    and abs(pose['z'] - float(home_pose['z'])) < 10.0
                    and abs(pose['pitch'] - 0.0) < 5.0
                ):
                    return True
            time.sleep(0.05)
        if not saw_state:
            time.sleep(1.5)
        return False

    def publish_track_pose(self, target_pose):
        now = time.time()
        if now - self.last_track_publish_time < TRACK_PUBLISH_INTERVAL:
            return
        if self.last_track_pose is not None:
            delta = max(abs(target_pose[i] - self.last_track_pose[i]) for i in range(3))
            if delta < MIN_MOVE_DELTA_MM:
                return
        hp = self.home_pose
        self.publish_arm(
            target_pose[0], target_pose[1], target_pose[2],
            hp['pitch'], hp['roll'], hp['claw'], TRACK_TIME_MS)
        self.last_track_publish_time = now
        self.last_track_pose = target_pose

    def to_face_tracking_base(self):
        try:
            with self.lock:
                self.enable_color_tracking = False
                self.enable_face_tracking = False
                self.tracker = None
            self.goback()
            tracker = face_tracker.FaceTracker(self.home_pose['x'])
            with self.lock:
                self.tracker = tracker
                self.enable_face_tracking = True
                self.enable_color_tracking = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start face tracking')
        finally:
            with self.lock:
                self.thread = None

    def to_color_tracking_base(self):
        try:
            with self.lock:
                self.enable_face_tracking = False
                self.enable_color_tracking = False
                self.tracker = None
                target_color_name = self.target_color_name or 'red'
            self.goback()
            tracker = color_tracker.ColorTracker(target_color_name, self.home_pose['x'])
            with self.lock:
                self.tracker = tracker
                self.enable_color_tracking = True
                self.enable_face_tracking = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start color tracking')
        finally:
            with self.lock:
                self.thread = None

    def enable_color_srv_callback(self, request, response):
        with self.lock:
            if request.data:
                if self.thread is None:
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'start color tracking...')                    
                    self.thread = threading.Thread(target=self.to_color_tracking_base, daemon=True)
                    self.thread.start()
                    response.success = True
                    response.message = 'enter'
                    return response
                msg = 'Enable Color Tracking, 有其他操作正在进行, 请稍后重试'
                self.get_logger().info(msg)
                response.success = True
                response.message = 'enter'
                return response

            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop color tracking...')
            if self.enable_color_tracking:
                self.enable_color_tracking = False
                self.tracker = None
            response.success = True
            response.message = 'enter'
            return response

    def enable_face_srv_callback(self, request, response):
        with self.lock:
            if self.thread is None:
                if request.data:
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'start face tracking...')
                    self.thread = threading.Thread(target=self.to_face_tracking_base, daemon=True)
                    self.thread.start()
                else:
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'stop face tracking...')
                    if self.enable_face_tracking:
                        self.enable_face_tracking = False
                        self.tracker = None
                response.success = True
                response.message = 'enter'
                return response

            msg = 'Enable Face Tracking, 有其他操作正在进行, 请稍后重试'
            self.get_logger().info(msg)
            response.success = True
            response.message = 'enter'
            return response

    def set_target_color_srv_callback(self, request, response):
        with self.lock:
            if request.data != 0:
                colors = ['None', 'red', 'green', 'blue', 'yellow']
                color_index = int(request.data)
                if color_index < 0 or color_index >= len(colors):
                    response.success = False
                    response.message = 'invalid color index'
                    return response
                self.target_color_name = colors[color_index]
                self.get_logger().info('\033[1;32mset target color: ' + self.target_color_name + '\033[0m')
                if self.enable_color_tracking:
                    self.tracker = color_tracker.ColorTracker(self.target_color_name, self.home_pose['x'])
            else:
                self.target_color_name = None
                self.tracker = None
            response.success = True
            response.message = 'enter'
            return response

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'loading object tracking')
        self.heart = Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        threading.Thread(target=self.goback, daemon=True).start()
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        with self.lock:
            self.enable_face_tracking = False
            self.enable_color_tracking = False
            self.tracker = None
        self.thread_started = False
        response.success = True
        response.message = 'enter'
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'exit object tracking')
        self.goback()
        with self.lock:
            self.enable_face_tracking = False
            self.enable_color_tracking = False
            self.tracker = None
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.heart.destroy()
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))

        response.success = True
        response.message = 'exit'
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'rgb8')
        rgb_image = np.array(cv_image, dtype=np.uint8)
        with self.lock:
            if not self.thread_started:
                threading.Thread(target=self.image_processing, args=(ros_image,), daemon=True).start()
                self.thread_started = True

        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)

    def image_processing(self, ros_image):
        while rclpy.ok():
        # while True:
            rgb_image = self.image_queue.get()
            result_image = np.copy(rgb_image)

            if self.thread is None and self.tracker is not None:
                if self.tracker.tracker_type == 'color' and self.enable_color_tracking:
                    result_image, target_pose = self.tracker.proc(rgb_image, result_image)  
                    self.get_logger().info('color tracking target pose: ' + str(target_pose))              
                    if target_pose is not None:
                        self.publish_track_pose(target_pose)
                # elif tracker.tracker_type == 'face' and enable_face_tracking:
                elif self.tracker.tracker_type == 'face' and self.enable_face_tracking:
                    result_image, target_pose = self.tracker.proc(rgb_image, result_image)  
                    self.get_logger().info('face tracking target pose: ' + str(target_pose))
                    if target_pose is not None:
                        self.publish_track_pose(target_pose)

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, 'rgb8'))
            if result_image is not None and self.get_parameter('display').value:
                display_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
                cv2.imshow('result_image', display_image)
                cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectTrackingNode('object_tracking')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
