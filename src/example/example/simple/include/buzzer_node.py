#!/usr/bin/env python3
# encoding: utf-8
# 蜂鸣器控制例程

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import BuzzerState

class BuzzerController(Node):
    def __init__(self):
        super().__init__('buzzer_controller')
        self.pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.ready_client = self.create_client(Trigger, '/controller_manager/init_finish')

    def wait_for_controller(self, ready_timeout_sec=30.0, discovery_timeout_sec=3.0):
        self.get_logger().info('Waiting for /controller_manager/init_finish...')
        if self.ready_client.wait_for_service(timeout_sec=ready_timeout_sec):
            future = self.ready_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                self.get_logger().info(f'Controller ready: {future.result().message}')
            else:
                self.get_logger().warning('Timed out waiting for controller ready response')
        else:
            self.get_logger().warning('Controller ready service not available; publishing may race startup')

        deadline = time.monotonic() + discovery_timeout_sec
        while rclpy.ok() and self.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            self.get_logger().warning('No subscriber on /ros_robot_controller/set_buzzer yet; publishing anyway')

    def set_buzzer(self, freq, on_time, off_time, repeat):
        msg = BuzzerState()
        msg.freq = freq
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        
        # 发布消息
        self.pub.publish(msg)
        self.get_logger().info(f'Published BuzzerState: freq={msg.freq}, on_time={msg.on_time}, off_time={msg.off_time}, repeat={msg.repeat}')

def main(args=None):
    rclpy.init(args=args)
    controller = BuzzerController()
    controller.wait_for_controller()

    # 发送蜂鸣器状态
    controller.set_buzzer(freq=1900, on_time=0.1, off_time=0.2, repeat=10)
    time.sleep(3)
    controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()
