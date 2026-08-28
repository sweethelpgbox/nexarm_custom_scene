#!/usr/bin/env python3
# encoding: utf-8
#三维人脸检测
import os
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
from ros_robot_controller_msgs.msg import ArmCoords

mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh
LANDMARK_COLOR = (255, 220, 0)
CONNECTION_COLOR = (80, 255, 180)
drawing_spec = mp_drawing.DrawingSpec(color=LANDMARK_COLOR, thickness=1, circle_radius=1)
connection_spec = mp_drawing.DrawingSpec(color=CONNECTION_COLOR, thickness=1, circle_radius=1)
INIT_POSE = (200.0, 0.0, 200.0, 0.0, 0.0, 0.0, 1500)

class FaceMeshNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.running = True
        self.bridge = CvBridge()
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            min_detection_confidence=0.5,
        )
        self.drawing = mp.solutions.drawing_utils

        self.fps = fps.FPS()

        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')
        self.wait_for_motion_ready()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()

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
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(rgb_image)

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            black_image = np.zeros_like(image)

            resize_image = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)), cv2.INTER_NEAREST) # 缩放图片(resize the image)
            results = self.face_mesh.process(resize_image) # 调用人脸检测(call human face detection)
            if results.multi_face_landmarks is not None:
                for face_landmarks in results.multi_face_landmarks:
                    mp_drawing.draw_landmarks(
                            image=black_image,
                            landmark_list=face_landmarks,
                            connections = mp_face_mesh.FACEMESH_CONTOURS,
                            landmark_drawing_spec=drawing_spec,
                            connection_drawing_spec=connection_spec)
            result_image = np.concatenate([image, black_image], axis=1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow('face_mech', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出
                break
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = FaceMeshNode('face_landmarker')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()
