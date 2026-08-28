#!/usr/bin/env python3
# encoding: utf-8
import rclpy
import time
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState, ArmCoords
from rclpy.callback_groups import ReentrantCallbackGroup

class CheckServoConnection(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=timer_cb_group)
        self.client.wait_for_service()

        self.timer = self.create_timer(0.0, self.check_arm_controller, callback_group=timer_cb_group)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def check_arm_controller(self):
        self.timer.cancel()
        try:
            # Publish a test command (init pose) to verify the arm controller is responding
            self.publish_arm(200.0, 0.0, 200.0, -90.0, 0.0, 0.0, 1000)
            # Check that the topic has at least one subscriber (AT32 controller)
            time.sleep(1.0)
            sub_count = self.arm_pub.get_subscription_count()
            if sub_count > 0:
                self.get_logger().info(f"Arm controller is responding ({sub_count} subscriber(s) on /ros_robot_controller/arm/set_coords)")
            else:
                self.handle_arm_not_found()
        except Exception as e:
            self.get_logger().error(str(e))

    def handle_arm_not_found(self):
        while True:
            try:
                msg = BuzzerState()
                msg.freq = 1900
                msg.on_time = 0.1
                msg.off_time = 0.15
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                time.sleep(0.5)
                self.get_logger().error("No subscriber on /ros_robot_controller/arm/set_coords — arm controller not detected!")
                # Re-check
                if self.arm_pub.get_subscription_count() > 0:
                    self.get_logger().info("Arm controller recovered.")
                    break
            except Exception as e:
                self.get_logger().error(str(e))
                break  
 
 

def main():
    node = CheckServoConnection('check_servo_connection')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
if __name__ == '__main__':
    main()

    
