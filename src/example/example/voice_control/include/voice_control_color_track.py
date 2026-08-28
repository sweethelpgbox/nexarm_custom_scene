#!/usr/bin/env python3
# encoding: utf-8

# 语音控制颜色追踪
import os
import json
import rclpy
from . import voice_play
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.srv import SetFloat64
from std_srvs.srv import Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup

class VoiceControlColorTrackNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.language = os.environ['ASR_LANGUAGE']
        timer_cb_group = ReentrantCallbackGroup()
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.create_subscription(String, 'asr_node/voice_words', self.words_callback, 1, callback_group=timer_cb_group)
        self.client = self.create_client(Trigger, 'asr_node/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, 'object_tracking/init_finish')
        self.client.wait_for_service()
        self.enter_client = self.create_client(Trigger, 'object_tracking/enter')
        self.enter_client.wait_for_service()
        self.start_client = self.create_client(SetBool, 'object_tracking/enable_color_tracking')
        self.start_client.wait_for_service()
        self.set_color_client = self.create_client(SetFloat64, 'object_tracking/set_target_color', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.play('running')
        self.get_logger().info('\033[1;32m%s\033[0m' % '唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('\033[1;32m%s\033[0m' % '唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('\033[1;32m%s\033[0m' % '控制指令: 追踪红色 追踪绿色 追踪蓝色 停止追踪(Voice command: track red/green/blue object)')

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        voice_play.play(name, language=self.language)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_tracking_color(self, color_index):
        self.send_request(self.enter_client, Trigger.Request())

        req = SetFloat64.Request()
        req.data = float(color_index)
        res = self.send_request(self.set_color_client, req)
        if not res.success:
            self.get_logger().info(f'set target color failed: {res.message}')
            return False

        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.start_client, req)
        if not res.success:
            self.get_logger().info(f'enable color tracking failed: {res.message}')
            return False
        return True

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                               '失败10次(Fail-10-times']:
            if words == '追踪红色' or words == 'track red object':
                if self.start_tracking_color(1.0):
                    self.play('start_track_red')
                else:
                    self.play('track_fail')
            elif words == '追踪绿色' or words == 'track green object':
                if self.start_tracking_color(2.0):
                    self.play('start_track_green')
                else:
                    self.play('track_fail')
            elif words == '追踪蓝色' or words == 'track blue object':
                if self.start_tracking_color(3.0):
                    self.play('start_track_blue')
                else:
                    self.play('track_fail')
            elif words == '停止追踪' or words == 'stop tracking':
                req = SetBool.Request()
                req.data = False
                res = self.send_request(self.start_client, req)
                if res.success:
                    self.play('stop_track')
                else:
                    self.play('stop_fail')
        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

def main():
    rclpy.init()
    node = VoiceControlColorTrackNode('voice_control_color_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
