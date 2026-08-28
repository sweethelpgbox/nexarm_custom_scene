#!/usr/bin/env python3
# coding: utf8
# 手势交互
import cv2
import time
import enum
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
import sdk.buzzer as buzzer
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from sdk.common import vector_2d_angle
from ros_robot_controller_msgs.msg import ArmCoords
from rclpy.callback_groups import ReentrantCallbackGroup

TRACE_COLOR = (80, 255, 180)
TEXT_COLOR = (255, 220, 0)
TEXT_SHADOW = (30, 30, 30)
HAND_LANDMARK_COLOR = (255, 220, 0)
HAND_CONNECTION_COLOR = (80, 255, 180)
INIT_POSE = (200.0, 0.0, 200.0, 0.0, 0.0, -55.0, 1500)
GESTURE_ACTIONS = {
    'fist': [(200, 0, 90, -90, 0, 30, 800)],
    'five': [(200, 0, 260, -45, 0, -55, 900)],
    'one': [(200, 0, 200, -90, 0, -55, 700), (200, 0, 110, -90, 0, -55, 700), (200, 0, 200, -90, 0, -55, 700)],
    'two': [(200, 80, 200, -90, 0, -55, 900)],
    'three': [(200, -80, 200, -90, 0, -55, 900)],
    'gun': [(250, 0, 150, -90, 0, -55, 700), (150, 0, 150, -90, 0, -55, 700)],
    'six': [(200, 0, 200, -90, 90, -55, 900)],
    'OK': [(200, 0, 150, -90, 0, 0, 900)],
    'four': [(200, 0, 200, -45, 0, -55, 900)],
    'hand_heart': [(200, 0, 110, -90, 0, 30, 700), (200, 0, 110, -90, 0, -55, 700)],
}


def get_hand_landmarks(img_size, landmarks):
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    angle_list = []
    angle_list.append(vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20]))
    return [abs(a) for a in angle_list]


def h_gesture(angle_list):
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        return 'fist'
    if (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        return 'gun'
    if (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        return 'hand_heart'
    if (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        return 'one'
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        return 'two'
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        return 'three'
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        return 'OK'
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        return 'four'
    if (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        return 'five'
    if (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        return 'six'
    return 'none'


class State(enum.Enum):
    NULL = 0
    RUNNING = 1


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int64)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


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
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.buzzer = buzzer.BuzzerController()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.start = False
        self.draw = True
        self.state = State.NULL
        self.points = [[0, 0]]
        self.no_finger_timestamp = time.time()
        self.count = 0
        self.direction = ''
        self.last_gesture = 'none'

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.image_sub = self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        if self.get_bool_param('start', False):
            self.start_srv_callback(Trigger.Request(), Trigger.Response())
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'hand_gesture ready')

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return bool(value)
        except Exception:
            return default

    def should_display(self):
        return self.get_bool_param('display', True)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def wait_for_motion_ready(self):
        self.get_logger().info('等待底层控制初始化...')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            self.get_logger().info('等待机械臂坐标控制订阅...')
            time.sleep(0.2)
        self.init_action()

    def init_action(self):
        self.publish_arm(*INIT_POSE)
        time.sleep(1.8)

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

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start hand gesture')
        self.start = True
        response.success = True
        response.message = 'start'
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop hand gesture')
        self.start = False
        self.state = State.NULL
        self.draw = True
        self.last_gesture = 'none'
        self.init_action()
        response.success = True
        response.message = 'stop'
        return response

    def do_act(self, gesture):
        actions = GESTURE_ACTIONS.get(gesture)
        if actions:
            for act in actions:
                self.publish_arm(*act)
                time.sleep(act[6] / 1000.0 + 0.3)
            time.sleep(0.5)
            self.publish_arm(*INIT_POSE)
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
        landmark_spec = self.drawing.DrawingSpec(color=HAND_LANDMARK_COLOR, thickness=2, circle_radius=2)
        connection_spec = self.drawing.DrawingSpec(color=HAND_CONNECTION_COLOR, thickness=2, circle_radius=1)

        while rclpy.ok():
            bgr_image = cv2.flip(self.image_queue.get(), 1)
            result_image = np.copy(bgr_image)

            if time.time() - self.no_finger_timestamp > 2:
                self.direction = ''
            elif self.direction != '':
                cv2.putText(result_image, self.direction, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOR, 2)

            if self.start:
                try:
                    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                    results = self.hand_detector.process(rgb_image)
                    if results.multi_hand_landmarks and self.draw:
                        gesture = 'none'
                        for hand_landmarks in results.multi_hand_landmarks:
                            self.no_finger_timestamp = time.time()
                            self.drawing.draw_landmarks(
                                result_image,
                                hand_landmarks,
                                mp.solutions.hands.HAND_CONNECTIONS,
                                landmark_spec,
                                connection_spec,
                            )
                            h, w = bgr_image.shape[:2]
                            landmarks = get_hand_landmarks((w, h), hand_landmarks.landmark)
                            angle_list = hand_angle(landmarks)
                            gesture = h_gesture(angle_list)

                        cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TEXT_SHADOW, 5)
                        cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TEXT_COLOR, 2)
                        draw_points(result_image, self.points[1:], color=TRACE_COLOR)

                        if self.state != State.RUNNING:
                            if gesture == self.last_gesture and gesture != 'none':
                                self.count += 1
                            else:
                                self.count = 0
                            if self.count > 20:
                                self.state = State.RUNNING
                                self.draw = False
                                threading.Thread(target=self.buzzer_task, daemon=True).start()
                                threading.Thread(target=self.do_act, args=(gesture,), daemon=True).start()
                        else:
                            self.count = 0
                        self.last_gesture = gesture
                    else:
                        if self.state != State.NULL and time.time() - self.no_finger_timestamp > 2:
                            self.points = [[0, 0]]
                            self.state = State.NULL
                except Exception as e:
                    self.get_logger().error(str(e))

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, 'bgr8'))
            self.fps.update()
            self.fps.show_fps(result_image)
            if self.should_display():
                cv2.imshow('result_image', result_image)
                cv2.waitKey(1)


def main():
    rclpy.init()
    node = HandGestureNode('hand_gesture')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
