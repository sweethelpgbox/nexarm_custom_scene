#!/usr/bin/env python3
# encoding: utf-8
#肢体骨骼检测
"""
这个程序实现了人体骨架识别(this program implements human skeleton recognition)
运行现象：桌面显示识别结果画面， 显示人体骨架连线(runtime behavior: the desktop displays the recognition result screen, showing the human skeleton lines)
"""
import cv2
import time
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import ArmCoords

POSE_LANDMARK_COLOR = (255, 220, 0)
POSE_CONNECTION_COLOR = (80, 255, 180)
INIT_POSE = (200.0, 0.0, 200.0, 0.0, 0.0, 0.0, 1500)


class MankindPoseNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # 实例化一个肢体识别器(instantiate a limb recognizer)
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.2
        )
        self.drawing = mp.solutions.drawing_utils # 结果绘制工具(result drawing tool)
        self.fps = fps.FPS() # 帧率计数器(frame rate calculator)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换

        self.running = True
        self.display = self.get_bool_param('display', True)
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return bool(value)
        except Exception:
            return default

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('mankind_pose ready')

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
        self.publish_arm(*INIT_POSE)
        time.sleep(1.8)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)

    def main(self):
        while self.running:
            try:
                bgr_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            bgr_image = cv2.flip(bgr_image, 1)  # 镜像画面, 这样可以正对屏幕和相机看效果(mirror image, aligned with the screen and camera for better visualization)
            result_image = np.copy(bgr_image) # 将画面复制一份作为结果，结果绘制在这上面(duplicate the image as the result canvas, and draw the results on it)
            # bgr_image = cv2.resize(bgr_image, (int(ros_image.width / 2), int(ros_image.height / 2))) 
            results = self.pose.process(bgr_image)  # 进行识别(perform recognition)
            if results.pose_landmarks is not None:
                self.drawing.draw_landmarks(
                    result_image,
                    results.pose_landmarks,
                    mp.solutions.pose.POSE_CONNECTIONS,
                    self.drawing.DrawingSpec(color=POSE_LANDMARK_COLOR, thickness=2, circle_radius=2),
                    self.drawing.DrawingSpec(color=POSE_CONNECTION_COLOR, thickness=2, circle_radius=1),
                ) # 画出各关节及连线(draw the joints and lines connecting them)
            self.fps.update() # 计算帧率(calculate frame rate)
            self.fps.show_fps(result_image)
            if self.display:
                cv2.imshow("result", result_image)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    self.running = False

        cv2.destroyAllWindows()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MankindPoseNode('mankind_pose')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
    
