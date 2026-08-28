#!/usr/bin/env python3
# encoding: utf-8
# Locate and Grasp 定位夹取
import os
import cv2
import yaml
import time
import rclpy
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from sensor_msgs.msg import Image as RosImage, CameraInfo
from app import calibrated_pose
from example.scene_pose import load_scene_home_pose


class PositioningClamp(Node):
    # Constants (mm coords)
    INIT_HOME = load_scene_home_pose()
    INIT_X = INIT_HOME['x']
    INIT_Y = INIT_HOME['y']
    INIT_Z = INIT_HOME['z']
    INIT_PITCH = INIT_HOME['pitch']
    INIT_ROLL = INIT_HOME['roll']
    INIT_CLAW = INIT_HOME['claw']
    GRAB_CLAW = 30.0
    OPEN_CLAW = -90.0

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.K = None
        self.count = 0
        self.pick_pitch = -80  # degrees (negative for ArmCoords convention)
        self.result_image = None
        self.camera_type = os.environ['CAMERA_TYPE']
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"

        self.previous_pose = None
        self.start = True

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)

        # Subscribe to image topic. 订阅图像话题
        self.image_sub = self.create_subscription(RosImage, '/color_detection/result_image', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)

        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.OPEN_CLAW, 1000)

        threading.Thread(target=self.run, daemon=True).start()

    def get_yaml(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # Convert to numpy array. 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            self.white_area_center = np.array(config['white_area_pose_world'])

        tvec = extristric[:1]  # Take the first row. 取第一行
        rmat = extristric[1:]  # Take the last three rows. 取后面三行

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat

    def camera_info_callback(self, msg):
        self.K = np.matrix(msg.k).reshape(1, -1, 3)

    # Process ROS node data. 处理ROS节点数据
    def image_callback(self, result_image):
        # Convert ROS Image message to OpenCV image. 将 ROS Image 消息转换为 OpenCV 图像
        self.result_image = self.bridge.imgmsg_to_cv2(result_image, "mono8")

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

    def start_sorting(self, pose_t, pose_R):
        self.get_logger().info("开始搬运堆叠...")
        position = [float(pose_t[0]), float(pose_t[1]), float(pose_t[2])]
        yaw = float(pose_R)
        self.pick_with_reversed_claw(position, 80, yaw, 0.02)
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.OPEN_CLAW, 1000)
        time.sleep(1.0)

    def pick_with_reversed_claw(self, position, pitch, yaw, gripper_depth):
        x_mm = float(position[0]) * 1000.0
        y_mm = float(position[1]) * 1000.0
        z_stage = (float(position[2]) + 0.02) * 1000.0
        z_pre = (float(position[2]) + 0.01) * 1000.0
        z_down = (float(position[2]) + 0.01 - float(gripper_depth)) * 1000.0
        arm_pitch = -abs(float(pitch))
        roll = float(yaw)
        claw_open = self.GRAB_CLAW
        claw_grab = -25.5

        self.publish_arm(x_mm, y_mm, z_stage, arm_pitch, 0.0, claw_open, 1500)
        time.sleep(1.8)
        self.publish_arm(x_mm, y_mm, z_pre, arm_pitch, roll, claw_open, 1800)
        time.sleep(2.1)
        self.publish_arm(x_mm, y_mm, z_down, arm_pitch, roll, claw_open, 1000)
        time.sleep(1.2)
        self.publish_arm(x_mm, y_mm, z_down, arm_pitch, roll, claw_grab, 500)
        time.sleep(0.8)
        self.publish_arm(x_mm, y_mm, z_stage, arm_pitch, roll, claw_grab, 1000)
        time.sleep(1.2)
        self.publish_arm(x_mm, y_mm, z_stage + 10.0, arm_pitch, 0.0, claw_grab, 1000)
        time.sleep(1.3)

    def run(self):
        while True:
            try:
                if self.result_image is not None and self.K is not None:
                    # Calculate the detected contours. 计算识别到的轮廓
                    contours = cv2.findContours(self.result_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

                    if contours:
                        # Find the largest contour. 找出最大轮廓
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        x, y, yaw = rect[0][0], rect[0][1], rect[2]

                        self.get_yaml()
                        config_data = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
                        position, projection_matrix = calibrated_pose.pixel_to_calibrated_world(
                            (x, y),
                            self.K,
                            self.extristric,
                            self.white_area_center,
                            config_data,
                            height=0.015,
                        )

                        # If previous_pose is None, it means this is the first calculation.
                        if self.previous_pose is None:
                            self.previous_pose = position
                            continue

                        position_difference = np.linalg.norm(np.array(position) - np.array(self.previous_pose))

                        if position_difference < 0.01:
                            self.count += 1
                        else:
                            self.count = 0
                            self.previous_pose = position

                        if self.count >= 60:
                            config_data = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
                            position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics')
                            self.get_logger().info(f"像素坐标为: x: {x}, y: {y}")
                            self.get_logger().info(f"实际坐标为： {position}")
                            self.start_sorting(position, yaw)
                            self.count = 0
                            break
                    else:
                        time.sleep(0.01)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"发生错误: {e}")


def main():
    rclpy.init()
    node = PositioningClamp('positioning_clamp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
