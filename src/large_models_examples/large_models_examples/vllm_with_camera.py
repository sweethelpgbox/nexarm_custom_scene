#!/usr/bin/env python3
# encoding: utf-8

import cv2
import time
import queue
import rclpy
import threading
import os

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Empty, Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState

PROMPT = '''
'''

display_size = [640, 480]


class VLLMWithCamera(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.image_queue = queue.Queue(maxsize=2)
        self.vllm_result = ''
        self.running = True
        self.current_pose = None
        self.known_pose = dict(DEFAULT_ARM_POSE)

        timer_cb_group = ReentrantCallbackGroup()
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def arm_state_callback(self, msg):
        self.current_pose = arm_pose_dict(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.claw, msg.yaw)

    def init_process(self):
        self.timer.cancel()
        wait_for_arm_runtime_ready(
            self,
            self.arm_pub,
            self.controller_init_client,
            self.kinematics_init_client,
        )

        msg = SetModel.Request()
        msg.model_type = 'vllm'
        if os.environ['ASR_LANGUAGE'] == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.model = vllm_model
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)

        init = load_scene_home_pose()
        self.publish_arm(init['x'], init['y'], init['z'], init['pitch'], init['roll'], init['claw'], 1000)
        time.sleep(1.0)
        pose = get_pose_snapshot(self)
        self.known_pose = dict(pose)

        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

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
        update_known_pose(self, x, y, z, pitch, roll, claw)

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data

    def process(self):
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result:
                msg = String()
                msg.data = self.vllm_result
                self.tts_text_pub.publish(msg)
                self.vllm_result = ''
            cv2.imshow('image', image)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def play_audio_callback(self, msg):
        if msg.data:
            req = SetBool.Request()
            req.data = True
            self.send_request(self.awake_client, req)

    def image_callback(self, ros_image):
        try:
            _, bgr_image = decode_color_image(ros_image)
        except Exception as e:
            self.get_logger().error(f"图像转换出错：{e}")
            return
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)


def main():
    node = VLLMWithCamera('vllm_with_camera')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
