import time
import math
import rclpy
from rclpy.node import Node
import numpy as np
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import BuzzerState, ArmCoords
from example.scene_pose import get_use_scene_pose, load_scene_home_pose


class PathPlanning(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.start = True

        # Constants (mm coords)
        self.INIT_X = 200; self.INIT_Y = 0; self.INIT_Z = 200
        self.INIT_PITCH = 0; self.INIT_ROLL = 0; self.INIT_CLAW = 0
        if get_use_scene_pose(self):
            home = load_scene_home_pose()
            self.INIT_X = home['x']; self.INIT_Y = home['y']; self.INIT_Z = home['z']
            self.INIT_PITCH = home['pitch']; self.INIT_ROLL = home['roll']; self.INIT_CLAW = home['claw']

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        self.wait_for_motion_ready()
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.INIT_CLAW, 1000)  # 设置机械臂初始位置
        time.sleep(1.5)
        self.run()

    def wait_for_motion_ready(self):
        self.get_logger().info('等待底层初始化完成...')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.05)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def move(self, x, y, z, pitch, t=1000):
        # x, y, z in mm; pitch in degrees
        self.publish_arm(x, y, z, pitch, self.INIT_ROLL, self.INIT_CLAW, t)

    def run(self):
        while self.start:
            # 运行路径规划程序
            self.get_logger().info("云台运动")
            # Old: 0.2m, -0.1m, 0.05m → 200mm, -100mm, 50mm
            self.move(200, -100, 50, self.INIT_PITCH)
            time.sleep(2)
            self.get_logger().info("动作1")
            # Old: 0.2m, -0.05m, 0.07m → 200mm, -50mm, 70mm
            self.move(200, -50, 70, self.INIT_PITCH)
            time.sleep(2)
            self.get_logger().info("动作2")
            # Old: 0.2m, -0.05m, 0.005m → 200mm, -50mm, 5mm
            self.move(200, -50, 5, self.INIT_PITCH)
            time.sleep(3)
            self.start = False  # 停止循环
            self.get_logger().info("停止运动")

        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.INIT_CLAW, 1000)  # 设置机械臂初始位置
        time.sleep(1)


def main():
    rclpy.init()
    node = PathPlanning('path_planning')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
