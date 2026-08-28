#!/usr/bin/env python3
# encoding: utf-8
# 标签追踪 — 全舵机直控，acc=0 speed=0
import cv2
import time
import rclpy
import queue
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
from rclpy.node import Node
from cv_bridge import CvBridge
from dt_apriltags import Detector
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState, ArmServoSingle
from ros_robot_controller_msgs.srv import BusServoCtrl
from example.scene_pose import get_use_scene_pose, load_scene_context

# joint1: 底座旋转(左右/Y), id=1, init=2048
# joint2: 大臂(上下/Z), id=2, init=2048, 方向反
JOINT1_ID = 1
JOINT1_INIT = 2048
JOINT1_MIN = 600
JOINT1_MAX = 3500

JOINT2_ID = 2
JOINT2_INIT = 2048
JOINT2_MIN = 1000
JOINT2_MAX = 3200
JOINT1_MAX_STEP = 10.0
JOINT2_MAX_STEP = 8.0
TRACK_DEADZONE_PX = 14.0
SERVO_MIN_DELTA = 1.5
SERVO_PUBLISH_INTERVAL = 0.08


class TagTrackNode(Node):
    INIT_X = 200
    INIT_Y = 0
    INIT_Z = 200
    INIT_PITCH = 0
    INIT_ROLL = 0
    GRAB_CLAW = 0

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.running = True
        self.start = False
        self.image_queue = queue.Queue(maxsize=2)
        self.display = self._get_bool_param('display', False)
        self.target_tag_id = self._get_int_param('tag_id', -1)
        self.image_topic = str(self._get_param_value('image_topic', '/depth_cam/rgb/image_raw'))
        self.scene_name = str(self._get_param_value('scene', 'scene_0'))
        self.last_image_log_time = 0.0
        self.last_detect_log_time = 0.0
        self.last_track_log_time = 0.0
        self.arm_state_ready = threading.Event()
        self.scene_context = self.load_current_scene_context()
        self.home_pose = self.scene_context['home_pose']

        self.pid_y = pid.PID(0.55, 0.0, 0.02)
        self.pid_z = pid.PID(0.40, 0.0, 0.02)
        self.joint1_pos = float(JOINT1_INIT)
        self.joint2_pos = float(JOINT2_INIT)
        self.last_joint1_publish = self.joint1_pos
        self.last_joint2_publish = self.joint2_pos
        self.last_servo_publish_time = 0.0

        self.fps = fps.FPS()
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.arm_state_sub = self.create_subscription(
            ArmFullState,
            '/ros_robot_controller/arm/full_state',
            self.arm_state_callback,
            5,
        )
        self.servo_pub = self.create_publisher(ArmServoSingle, '/ros_robot_controller/arm/servo_single', 5)
        self.servo_ctrl_client = self.create_client(BusServoCtrl, '/ros_robot_controller/bus_servo/ctrl')
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def _get_param_value(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
            return default
        return self.get_parameter(name).value

    def _get_bool_param(self, name, default):
        value = self._get_param_value(name, default)
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _get_int_param(self, name, default):
        try:
            return int(self._get_param_value(name, default))
        except (TypeError, ValueError):
            return int(default)

    def load_current_scene_context(self):
        context = load_scene_context({
            'x': float(self.INIT_X),
            'y': float(self.INIT_Y),
            'z': float(self.INIT_Z),
            'pitch': float(self.INIT_PITCH),
            'roll': float(self.INIT_ROLL),
            'claw': float(self.GRAB_CLAW),
            'time_ms': 1500,
        }, scene_name=self.scene_name)
        if not get_use_scene_pose(self):
            context['home_pose'].update({
                'x': float(self.INIT_X),
                'y': float(self.INIT_Y),
                'z': float(self.INIT_Z),
                'pitch': float(self.INIT_PITCH),
                'roll': float(self.INIT_ROLL),
                'claw': float(self.GRAB_CLAW),
            })
        return context

    def arm_state_callback(self, msg):
        self.arm_state_ready.set()

    def wait_for_arm_state(self, timeout_sec=10.0):
        self.get_logger().info('Waiting for ros_robot_controller arm state...')
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            if self.arm_state_ready.wait(timeout=0.1):
                self.get_logger().info('ros_robot_controller arm state ready')
                return True
        self.get_logger().warn('Timed out waiting for /ros_robot_controller/arm/full_state')
        return False

    @staticmethod
    def clamp_delta(value, limit):
        return max(-float(limit), min(float(limit), float(value)))

    def init_process(self):
        self.timer.cancel()

        # 等底层就绪
        client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.get_logger().info('Waiting for controller_manager...')
        client.wait_for_service()
        self.get_logger().info('Controller ready')
        self.destroy_client(client)
        self.wait_for_arm_state()

        # 1. 坐标控制到初始姿态
        msg = ArmCoords()
        self.scene_context = self.load_current_scene_context()
        self.home_pose = self.scene_context['home_pose']
        msg.x = float(self.home_pose['x']); msg.y = float(self.home_pose['y']); msg.z = float(self.home_pose['z'])
        msg.pitch = float(self.home_pose['pitch']); msg.roll = float(self.home_pose['roll']); msg.claw = float(self.home_pose['claw'])
        msg.time_ms = int(self.home_pose.get('time_ms', 1500))
        for _ in range(3):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(2.0)

        self.get_logger().info(
            f"[SCENE] active={self.scene_context['active_scene_name']}, "
            f"calibration={self.scene_context['calibration_scene_name']}, "
            f"map=({self.scene_context['map_length_m']}m x {self.scene_context['map_width_m']}m), "
            f"calibration_tag_id={self.scene_context['calibration_tag_id']}"
        )
        self.get_logger().info(
            f"Init pose sent: x={self.home_pose['x']}, y={self.home_pose['y']}, z={self.home_pose['z']}, "
            f"pitch={self.home_pose['pitch']}, roll={self.home_pose['roll']}, claw={self.home_pose['claw']}"
        )

        if self._get_bool_param('start', True):
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        if self.target_tag_id < 0:
            self.get_logger().info('AprilTag target: first detected tag')
        else:
            self.get_logger().info(f'AprilTag target id: {self.target_tag_id}')
        self.get_logger().info(f'Camera image topic: {self.image_topic}')
        self.get_logger().info('Result image topic: /tag_track/image_result')
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def set_servo_params(self, servo_id, position, acc, speed):
        if not self.servo_ctrl_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('BusServoCtrl service not available')
            return
        req = BusServoCtrl.Request()
        req.id = servo_id
        req.set_position = True
        req.position = int(position)
        req.acc = acc
        req.speed = speed
        req.set_torque = False
        req.set_mode = False
        future = self.servo_ctrl_client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f'Servo {servo_id} acc={acc} speed={speed}')
        else:
            self.get_logger().warn(f'Servo {servo_id} acc/speed set timeout or failed')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def publish_servo(self, servo_id, pos, time_ms):
        msg = ArmServoSingle()
        msg.id = servo_id
        msg.pos = int(pos)
        msg.time_ms = time_ms
        self.servo_pub.publish(msg)

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start tag track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop tag track")
        self.start = False
        response.success = True
        response.message = "stop"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)
        now = time.time()
        if now - self.last_image_log_time > 3.0:
            self.last_image_log_time = now
            self.get_logger().info(f'Camera frames received: {bgr_image.shape[1]}x{bgr_image.shape[0]}')

    def main(self):
        while self.running:
            try:
                rgb_image = self.image_queue.get(timeout=1.0)
            except queue.Empty:
                now = time.time()
                if now - self.last_image_log_time > 3.0:
                    self.last_image_log_time = now
                    self.get_logger().warn(f'No camera frame on {self.image_topic}')
                continue

            result_image = np.copy(rgb_image)
            status_text = 'AprilTag tracking: stopped'

            if self.start:
                t1 = time.time()
                gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
                tags = self.at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

                target_tag = None
                for tag in tags:
                    corners = tag.corners.astype(int)
                    color = (0, 255, 255)
                    cv2.polylines(result_image, [corners], True, color, 2)
                    cv2.putText(
                        result_image,
                        f'id:{tag.tag_id}',
                        (int(tag.center[0]) + 8, int(tag.center[1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                    if self.target_tag_id < 0 or tag.tag_id == self.target_tag_id:
                        target_tag = tag
                        break

                if target_tag is not None:
                    center_x = target_tag.center[0]
                    center_y = target_tag.center[1]
                    h, w = result_image.shape[:2]

                    corners = target_tag.corners.astype(int)
                    cv2.polylines(result_image, [corners], True, (0, 255, 0), 2)
                    cv2.circle(result_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    status_text = f'tracking tag {target_tag.tag_id}'

                    # joint1: 左右，像素右偏→舵机值增大
                    dx = center_x - (w / 2)
                    if abs(dx) > TRACK_DEADZONE_PX:
                        self.pid_y.SetPoint = w / 2
                        self.pid_y.update(center_x)
                        joint1_delta = self.clamp_delta(-self.pid_y.output, JOINT1_MAX_STEP)
                    else:
                        self.pid_y.clear()
                        joint1_delta = 0.0
                    self.joint1_pos += joint1_delta
                    self.joint1_pos = max(JOINT1_MIN, min(JOINT1_MAX, self.joint1_pos))

                    # joint2: 上下，方向反(min=4096,max=0)，像素下偏→舵机值增大(抬高)
                    dy = center_y - (h / 2)
                    if abs(dy) > TRACK_DEADZONE_PX:
                        self.pid_z.SetPoint = h / 2
                        self.pid_z.update(center_y)
                        joint2_delta = self.clamp_delta(self.pid_z.output, JOINT2_MAX_STEP)
                    else:
                        self.pid_z.clear()
                        joint2_delta = 0.0
                    self.joint2_pos += joint2_delta
                    self.joint2_pos = max(JOINT2_MIN, min(JOINT2_MAX, self.joint2_pos))

                    horizontal = '右' if dx > 0 else ('左' if dx < 0 else '中')
                    vertical = '下' if dy > 0 else ('上' if dy < 0 else '中')
                    now = time.time()
                    if now - self.last_track_log_time > 0.5:
                        self.last_track_log_time = now
                        self.get_logger().info(
                            f'Tag偏差: 左右={horizontal} dx={dx:.1f}px, 上下={vertical} dy={dy:.1f}px, '
                            f'joint1_delta={joint1_delta:.2f}, joint1_pos={self.joint1_pos:.1f}, '
                            f'joint2_delta={joint2_delta:.2f}, joint2_pos={self.joint2_pos:.1f}'
                        )

                    if now - self.last_servo_publish_time >= SERVO_PUBLISH_INTERVAL:
                        publish_1 = abs(self.joint1_pos - self.last_joint1_publish) >= SERVO_MIN_DELTA
                        publish_2 = abs(self.joint2_pos - self.last_joint2_publish) >= SERVO_MIN_DELTA
                        if publish_1:
                            self.publish_servo(JOINT1_ID, self.joint1_pos, 80)
                            self.last_joint1_publish = self.joint1_pos
                        if publish_2:
                            self.publish_servo(JOINT2_ID, self.joint2_pos, 80)
                            self.last_joint2_publish = self.joint2_pos
                        if publish_1 or publish_2:
                            self.last_servo_publish_time = now
                else:
                    ids = [int(tag.tag_id) for tag in tags]
                    if self.target_tag_id < 0:
                        status_text = 'no AprilTag detected'
                    else:
                        status_text = f'tag {self.target_tag_id} not found, detected: {ids}'
                    now = time.time()
                    if now - self.last_detect_log_time > 2.0:
                        self.last_detect_log_time = now
                        self.get_logger().warn(status_text)

                t2 = time.time()
                if t2 - t1 < 0.02:
                    time.sleep(0.02 - (t2 - t1))

            self.fps.update()
            self.fps.show_fps(result_image)
            cv2.putText(
                result_image,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255) if self.start and 'tracking' not in status_text else (0, 255, 0),
                2,
            )
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, 'bgr8'))
            if self.display:
                cv2.imshow("tag_track", result_image)
                cv2.waitKey(1)


def main():
    rclpy.init()
    node = TagTrackNode('tag_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
