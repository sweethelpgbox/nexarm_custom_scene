#!/usr/bin/env python3
# encoding: utf-8
# 舵机控制例程
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_control_demo')
        self.pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 1)
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
            self.get_logger().warning('No subscriber on /ros_robot_controller/bus_servo/set_position yet; publishing anyway')

    def set_servo_position(self, positions):
        msg = ServosPosition()
        position_list = []
        for i in positions:
            position = ServoPosition()
            position.id = i[0]
            position.position = int(i[1])
            position_list.append(position)
        msg.position = position_list
        self.pub.publish(msg)
        for pos in position_list:
            self.get_logger().info(f'id={pos.id}, position={pos.position}')

def main(args=None):
    rclpy.init(args=args)
    controller = ServoController()
    controller.wait_for_controller()

    try:
        while rclpy.ok():
            controller.set_servo_position(((3, 2000),))  # 设置舵机 ID 3 到位置 2000
            time.sleep(0.8)
            controller.set_servo_position(((3, 2500),))  # 设置舵机 ID 3 到位置 2500
            time.sleep(0.8)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()  # 清理节点
        rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()
