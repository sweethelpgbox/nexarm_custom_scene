#!/usr/bin/env python3
# coding: utf8
import cv2
import time
import enum
import os
import yaml
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
import sdk.buzzer as buzzer
from rclpy.node import Node
from app.common import Heart
from app.play_pose import get_use_scene_pose, load_play_home_pose
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from sdk.common import distance, vector_2d_angle
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import ArmCoords


def get_hand_landmarks(img_size, landmarks):
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    angle_list = []
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = 'fist'
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = 'gun'
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = 'hand_heart'
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = 'one'
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = 'two'
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = 'three'
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = 'OK'
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = 'four'
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = 'five'
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = 'six'
    else:
        gesture_str = 'none'

    return gesture_str


class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int64)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


GESTURE_ACTIONS = {
    'fist': [(200, 0, 80, -90, 0, 30, 800)],
    'five': [(200, 0, 300, -45, 0, -60, 800)],
    'one': [(200, 0, 200, -90, 0, -60, 600), (200, 0, 100, -90, 0, -60, 600), (200, 0, 200, -90, 0, -60, 600)],
    'two': [(200, 80, 200, -90, 0, -60, 800)],
    'three': [(200, -80, 200, -90, 0, -60, 800)],
    'gun': [(250, 0, 150, -90, 0, -60, 600), (150, 0, 150, -90, 0, -60, 600)],
    'six': [(200, 0, 200, -90, 90, -60, 800)],
    'OK': [(200, 0, 150, -90, 0, 0, 800)],
    'four': [(200, 0, 200, -45, 0, -60, 800)],
    'hand_heart': [(200, 0, 100, -90, 0, 30, 600), (200, 0, 100, -90, 0, -60, 600)],
}

INIT_POSE = (105, 0, 200, 0, 0, -60, 1500)


class HandGestureNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5,
        )
        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.buzzer = buzzer.BuzzerController()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = False
        self.start = False
        self.draw = True
        self.state = State.NULL
        self.points = [[0, 0]]
        self.no_finger_timestamp = time.time()
        self.one_count = 0
        self.count = 0
        self.direction = ''
        self.last_gesture = 'none'
        self.image_sub = None
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        if self.chassis_type == 'Slide_Rails':
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
        else:
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
        self.home_pose = self._load_home_pose_from_scene()

        self.timer_cb_group = ReentrantCallbackGroup()
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/enable_trace', self.start_srv_callback, callback_group=self.timer_cb_group)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.controller_init_client.wait_for_service()

        threading.Thread(target=self.main, daemon=True).start()
        
        if self.get_parameter('start').value:
            threading.Thread(target=self.enter_srv_callback, args=(Trigger.Request(), Trigger.Response()), daemon=True).start()
            req = SetBool.Request()
            req.data = True
            self.start_srv_callback(req, SetBool.Response())

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
        defaults = {
            'x': float(INIT_POSE[0]),
            'y': float(INIT_POSE[1]),
            'z': float(INIT_POSE[2]),
            'pitch': float(INIT_POSE[3]),
            'roll': float(INIT_POSE[4]),
            'claw': float(INIT_POSE[5]),
        }
        pose = load_play_home_pose(
            self.scene_config_path,
            defaults,
            pitch_override=INIT_POSE[3],
            use_scene=get_use_scene_pose(self),
        )
        return (
            pose['x'],
            pose['y'],
            pose['z'],
            pose['pitch'],
            pose['roll'],
            pose['claw'],
            INIT_POSE[6],
        )

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Loading hand gesture')
        self.running = True
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)
        response.success = True
        response.message = 'enter'
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'exit hand gesture')
        self.running = False
        self.start = False
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

    def init_process(self):
        self.timer.cancel()
        self.init_action()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.home_pose = self._load_home_pose_from_scene()
        self.publish_arm(*self.home_pose)
        time.sleep(1.8)

    def start_srv_callback(self, request, response):
        if request.data:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start hand gesture')
            self.start = True
            response.success = True
            response.message = 'start'
            return response

        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop hand gesture')
        self.start = False
        response.success = False
        response.message = 'stop'
        return response

    def do_act(self, gesture):
        actions = GESTURE_ACTIONS.get(gesture)
        if actions:
            for act in actions:
                self.publish_arm(*act)
                time.sleep(act[6] / 1000.0 + 0.3)
            time.sleep(0.5)
            self.home_pose = self._load_home_pose_from_scene()
            self.publish_arm(*self.home_pose)
            time.sleep(1.5)
        self.count = 0
        self.last_gesture = 'none'
        self.state = State.NULL
        self.draw = True

    def buzzer_task(self):
        self.buzzer.set_buzzer(500, 0.1, 0.5, 1)
        time.sleep(0.5)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    def main(self):
        while True:
            if not self.running:
                time.sleep(0.02)
                continue

            bgr_image = cv2.flip(self.image_queue.get(), 1)
            result_image = np.copy(bgr_image)

            if time.time() - self.no_finger_timestamp > 2:
                self.direction = ''
            elif self.direction != '':
                cv2.putText(result_image, self.direction, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            if self.start:
                try:
                    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                    results = self.hand_detector.process(rgb_image)
                    if results.multi_hand_landmarks and self.draw:
                        gesture = 'none'
                        index_finger_tip = [0, 0]
                        for hand_landmarks in results.multi_hand_landmarks:
                            self.drawing.draw_landmarks(result_image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
                            h, w = bgr_image.shape[:2]
                            landmarks = get_hand_landmarks((w, h), hand_landmarks.landmark)
                            angle_list = hand_angle(landmarks)
                            gesture = h_gesture(angle_list)
                            index_finger_tip = landmarks[8].tolist()

                        cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
                        cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                        draw_points(result_image, self.points[1:])
                        if self.state != State.RUNNING:
                            if gesture == self.last_gesture and gesture != 'none':
                                self.count += 1
                            else:
                                self.count = 0
                            if self.count > 20:
                                self.state = State.RUNNING
                                self.draw = False
                                threading.Thread(target=self.buzzer_task).start()
                                threading.Thread(target=self.do_act, args=(gesture,)).start()
                        else:
                            self.count = 0
                        self.last_gesture = gesture
                    else:
                        if self.state != State.NULL and time.time() - self.no_finger_timestamp > 2:
                            self.one_count = 0
                            self.points = [[0, 0]]
                            self.state = State.NULL
                except Exception as e:
                    self.get_logger().error(str(e))

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, 'bgr8'))
            if result_image is not None and self.get_parameter('display').value:
                cv2.imshow('result_image', result_image)
                cv2.waitKey(1)



def main():
    rclpy.init()
    node = HandGestureNode('finger_trace')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
