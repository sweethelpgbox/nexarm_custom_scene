#!/usr/bin/env python3
# encoding: utf-8
# 人脸追踪 — 基于 ros_robot_controller 坐标控制协议
import cv2
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import mediapipe as mp
from sdk import fps
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import ArmCoords
from example.scene_pose import load_scene_home_pose
from sdk.common import show_faces, mp_face_location, box_center, distance

# 机械臂初始姿态 (mm / 度)
INIT_HOME = load_scene_home_pose()
INIT_X = INIT_HOME['x']
INIT_Y = INIT_HOME['y']
INIT_Z = INIT_HOME['z']
INIT_PITCH = 0.0
INIT_ROLL = INIT_HOME['roll']
INIT_CLAW = INIT_HOME['claw']

# 追踪范围限制 (mm)
Y_MIN, Y_MAX = -150.0, 150.0
Z_MIN, Z_MAX = 80.0, 350.0


class FaceTrackingNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.3,
        )
        self.running = True
        self.start = False
        self.bridge = CvBridge()
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)

        self.y_pos = INIT_Y
        self.z_pos = INIT_Z
        self.pid_y = pid.PID(0.05, 0.0, 0.007)
        self.pid_z = pid.PID(0.05, 0.0, 0.007)
        self.detected_face = 0

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.image_sub = self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.init_action()
        try:
            if self.get_parameter('start').value:
                self.start_srv_callback(Trigger.Request(), Trigger.Response())
        except Exception:
            pass
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'face_tracking ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.y_pos = INIT_Y
        self.z_pos = INIT_Z
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            self.get_logger().info('等待 ros_robot_controller 订阅...')
            time.sleep(0.5)
        self.publish_arm(INIT_X, INIT_Y, INIT_Z, INIT_PITCH, INIT_ROLL, INIT_CLAW, 1500)
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
        self.get_logger().info('\033[1;32m%s\033[0m' % "start face track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop face track")
        self.start = False
        self.init_action()
        response.success = True
        response.message = "stop"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    def main(self):
        while self.running:
            bgr_image = self.image_queue.get()
            result_image = np.copy(bgr_image)

            if not self.start:
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
                continue

            results = self.face_detector.process(bgr_image)
            boxes, keypoints = mp_face_location(results, bgr_image)
            o_h, o_w = bgr_image.shape[:2]

            if len(boxes) > 0:
                self.detected_face += 1
                self.detected_face = min(self.detected_face, 20)

                if self.detected_face >= 5:
                    center = [box_center(box) for box in boxes]
                    dist = [distance(c, (o_w / 2, o_h / 2)) for c in center]
                    face = min(zip(boxes, center, dist), key=lambda k: k[2])
                    center_x, center_y = face[1]

                    t1 = time.time()

                    # 画面中 center_x > 中心 → 人脸偏右 → 机械臂 Y 增大（往左追）
                    self.pid_y.SetPoint = o_w / 2
                    self.pid_y.update(center_x)
                    self.y_pos += self.pid_y.output
                    self.y_pos = max(Y_MIN, min(Y_MAX, self.y_pos))

                    # 画面中 center_y > 中心 → 人脸偏下 → 机械臂 Z 减小（往下追）
                    self.pid_z.SetPoint = o_h / 2
                    self.pid_z.update(center_y)
                    self.z_pos += self.pid_z.output
                    self.z_pos = max(Z_MIN, min(Z_MAX, self.z_pos))

                    self.publish_arm(INIT_X, self.y_pos, self.z_pos,
                                     INIT_PITCH, INIT_ROLL, INIT_CLAW, 20)

                    t2 = time.time()
                    dt = t2 - t1
                    if dt < 0.02:
                        time.sleep(0.02 - dt)

                result_image = show_faces(result_image, bgr_image, boxes, keypoints)
            else:
                if self.detected_face > 0:
                    self.detected_face -= 1
                else:
                    self.pid_y.clear()
                    self.pid_z.clear()

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
            self.fps.update()
            self.fps.show_fps(result_image)
            cv2.imshow("result", result_image)
            cv2.waitKey(1)

        self.init_action()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = FaceTrackingNode('face_tracking')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
