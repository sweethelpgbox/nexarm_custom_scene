#!/usr/bin/env python3
# encoding: utf-8
# 正运动学演示 — 发送关节角度，底层返回末端坐标
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.srv import GetArmFK


class FkDemo(Node):
    def __init__(self):
        super().__init__('fk_demo')
        self.fk_client = self.create_client(GetArmFK, '/ros_robot_controller/arm/get_fk')
        self.fk_client.wait_for_service()
        self.max_attempts = 5
        self.attempt = 0
        self.retry_timer = None
        self.timer = self.create_timer(8.0, self.run_demo)

    def run_demo(self):
        self.timer.cancel()
        self.send_fk_request()

    def send_fk_request(self):
        self.attempt += 1
        # 示例: 发送 4 个关节角度 + roll + claw
        req = GetArmFK.Request()
        req.j1 = 0.0      # 底座旋转角(度)
        req.j2 = -30.0     # 关节2角度(度)
        req.j3 = 60.0      # 关节3角度(度)
        req.j4 = -20.0     # 关节4角度(度)
        req.roll = 0.0     # 末端旋转(度)
        req.claw = 0.0     # 夹爪角度(度)

        self.get_logger().info(
            f'发送关节角度({self.attempt}/{self.max_attempts}): '
            f'j1={req.j1}, j2={req.j2}, j3={req.j3}, j4={req.j4}, '
            f'roll={req.roll}, claw={req.claw}'
        )

        future = self.fk_client.call_async(req)       
        future.add_done_callback(self.fk_result_callback)

    def fk_result_callback(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(
                f'正运动学结果 — X={res.x:.1f}mm, Y={res.y:.1f}mm, Z={res.z:.1f}mm, '
                f'Pitch={res.pitch:.1f}°, Roll={res.roll:.1f}°, Claw={res.claw:.1f}°'
            )
        elif self.attempt < self.max_attempts:
            self.get_logger().warn('正运动学计算失败，等待控制板运动学参数就绪后重试')
            self.retry_timer = self.create_timer(2.0, self.retry_fk_request)
        else:
            self.get_logger().error('正运动学计算失败')

    def retry_fk_request(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.send_fk_request()


def main(args=None):
    rclpy.init(args=args)
    node = FkDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
