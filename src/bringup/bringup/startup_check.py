#!/usr/bin/env python3
# encoding: utf-8
# 自检程序(self-test program)
import os
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import BuzzerState, OLEDState, ServoPosition, ServosPosition

SCENE5_ID = 'scene_5'


def _active_scene():
    return (
        os.environ.get('CALIBRATION_CURRENT_SCENE')
        or os.environ.get('CALIBRATION_DEFAULT_SCENE')
        or os.environ.get('SCENE')
        or ''
    )


def _is_scene5():
    return _active_scene() == SCENE5_ID


def _scene5_arm_role():
    return os.environ.get('SCENE5_ARM_ROLE', 'A').strip().upper()


class BringUpNotifyNode(Node):
    def __init__(self):
        super().__init__('bringup_notify')
        self.ret = False
        self.lang = os.environ.get('ASR_LANGUAGE', 'English')

        if _is_scene5():
            role = _scene5_arm_role()
            buzzer_topic = f'/arm_{role.lower()}/ros_robot_controller/set_buzzer'
        else:
            buzzer_topic = '/ros_robot_controller/set_buzzer'
        self.buzzer_pub = self.create_publisher(BuzzerState, buzzer_topic, 1)
        self.servo_position_pub = self.create_publisher(ServosPosition, 'ros_robot_controller/bus_servo/set_position', 1)
        self.wait_for_services()

    def wait_for_services(self):
        self.get_logger().info("Waiting for services...")
        try:
            if _is_scene5():
                role = _scene5_arm_role()
                ns = f'/arm_{role.lower()}'
                if role == 'A':
                    self.wait_for_service(f'{ns}/scene5/arm_a/start')
                else:
                    self.wait_for_service(f'{ns}/scene5_waste_classification/enter')
            else:
                self.wait_for_service('/finger_trace/enter')
                self.wait_for_service('/object_sorting/enter')
                self.wait_for_service('/object_tracking/enter')
                self.wait_for_service('/waste_classification/enter')
                time.sleep(16)
            self.ret = True
        except Exception as e:
            self.get_logger().error(f"Error waiting for services: {e}")

    def wait_for_service(self, service_name):
        client = self.create_client(Trigger, service_name)

    def run(self):
        if self.ret:
            self.get_logger().info('\033[1;32m%s\033[0m' % "All services are ready!")
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.1
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

            self.set_position([
                                (1, 2048),
                                (2, 2048),
                                (3, 1198),
                                (4, 1200),
                                (5, 2048),
                                (6, 1000),
                            ])
        else:
            while rclpy.ok():
                msg = BuzzerState()
                msg.freq = 2500
                msg.on_time = 0.2
                msg.off_time = 0.01
                msg.repeat = 5
                self.buzzer_pub.publish(msg)
                time.sleep(0.5)

    def set_position(self, positions):
        msg = ServosPosition()

        for servo_id, pulse in positions:
            item = ServoPosition()
            item.id = int(servo_id)
            item.position = int(pulse)
            msg.position.append(item)

        self.servo_position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BringUpNotifyNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
