#!/usr/bin/env python3
# encoding: utf-8
# 逆运动学演示 — 发送末端坐标，底层返回舵机脉宽
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.srv import GetArmIK


class IkDemo(Node):
    def __init__(self):
        super().__init__('ik_demo')
        self.ik_client = self.create_client(GetArmIK, '/ros_robot_controller/arm/get_ik')
        self.ik_client.wait_for_service()
        self.max_attempts = 5
        self.attempt = 0
        self.retry_timer = None
        self.timer = self.create_timer(8.0, self.run_demo)

    def run_demo(self):
        self.timer.cancel()
        self.send_ik_request()

    def send_ik_request(self):
        self.attempt += 1
        # 示例: 发送目标末端坐标(mm) + 姿态(度)
        req = GetArmIK.Request()
        req.x = 200.0       # X 坐标(mm)
        req.y = 0.0         # Y 坐标(mm)
        req.z = 100.0       # Z 坐标(mm)
        req.pitch = -90.0   # 俯仰角(度), -90=朝下
        req.roll = 0.0      # 末端旋转(度)
        req.claw = 0.0      # 夹爪角度(度)

        self.get_logger().info(
            f'发送目标坐标({self.attempt}/{self.max_attempts}): '
            f'X={req.x}, Y={req.y}, Z={req.z}, '
            f'Pitch={req.pitch}, Roll={req.roll}, Claw={req.claw}'
        )

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_result_callback)

    def ik_result_callback(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(f'逆运动学结果 — 舵机脉宽: {list(res.servos)}')
        elif self.attempt < self.max_attempts:
            self.get_logger().warn('逆运动学计算失败，等待控制板运动学参数就绪后重试')
            self.retry_timer = self.create_timer(2.0, self.retry_ik_request)
        else:
            self.get_logger().error('逆运动学计算失败(目标位置可能超出工作空间)')

    def retry_ik_request(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.send_ik_request()


def main(args=None):
    rclpy.init(args=args)
    node = IkDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
