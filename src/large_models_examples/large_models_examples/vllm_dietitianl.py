#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import math
import json
import time
import queue
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetBox

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    PROMPT = '''
你作为智能管家机器人，可以抓取各种东西， 你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出。
## 1. 理解用户指令
我会给你一句话，你需要根据我的话做出最佳决策，从做出的决策中提取「物体名称」
## 2. 理解图片
我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标
## 图片的尺寸为640*400，请输出像素坐标。
【特别注意】： 你每次只能抓取一个物体, object对应的name要用英文表示
## 输出格式（请仅输出以下内容，不要说任何多余的话
{
 "object": name,
 "xyxy": [xmin, ymin, xmax, ymax],
 "response": "人性化的回复，回复可以简洁点"
}
    '''
else:
    PROMPT = '''
**Role:
You are an intelligent housekeeper robot that can detect and grab objects based on user instructions and images. Your goal is to identify and locate a single object and respond in a friendly, human-like manner.

**Task:
1. Instruction Understanding
You will receive a sentence from the user.
→ Extract the object name based on the user's intent.
→ The object name must be in English.
→ You can only handle one object at a time.
→ Do not include any unmentioned objects.

2. Image Analysis
You will receive an image.
→ Find the bounding box (top-left and bottom-right pixel coordinates) for the object mentioned.
→ The image size is 640*400, so return pixel coordinates in [xmin, ymin, xmax, ymax].
→ Focus on accurate detection and orientation.

**Output Format (output only this JSON structure — no explanation or extra text):
{
  "object": "object_name_in_English",
  "xyxy": [xmin, ymin, xmax, ymax],
  "response": "Concise, friendly human-like reply"
}

**Output Example:
{
  "object": "apple",
  "xyxy": [160, 128, 307, 240],
  "response": "Got it! I found the apple."
}
    '''

class VLLMDietitianl(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.vllm_result = ''
        # self.vllm_result = '''json{"object":"香蕉","xyxy":[243, 441, 842, 552], "response": "香蕉"}'''
        self.running = True
        self.action_finish = False
        self.play_audio_finish = False
        self.lock = threading.RLock()
        self.language = os.environ["ASR_LANGUAGE"]
        self.client = speech.OpenAIAPI(api_key, base_url)
        
        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        
        self.enter_client = self.create_client(Trigger, 'intelligent_grasp/enter')
        self.enter_client.wait_for_service()

        self.start_client = self.create_client(SetBool, 'intelligent_grasp/enable_transport')
        self.start_client.wait_for_service()

        self.set_target_client = self.create_client(SetBox, 'intelligent_grasp/set_target')
        self.set_target_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()

        msg = SetModel.Request()
        msg.model_type = 'vllm'
        if self.language == 'Chinese':
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
        
        init_finish = self.create_client(Empty, 'intelligent_grasp/init_finish')
        self.get_logger().info('等待 intelligent_grasp 初始化完成...')
        wait_for_init_service(init_finish)
        self.get_logger().info('intelligent_grasp 初始化完成，进入抓取流程')
        self.send_request(self.enter_client, Trigger.Request())
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def normalize_box(self, box, image_width=640, image_height=400):
        try:
            box_values = list(box)
        except TypeError:
            self.get_logger().info(f'invalid vllm box format: {box}')
            return []
        if len(box_values) != 4:
            self.get_logger().info(f'invalid vllm box format: {box}')
            return []

        try:
            values = [float(i) for i in box_values]
        except (TypeError, ValueError):
            self.get_logger().info(f'invalid vllm box value: {box}')
            return []

        if all(0.0 <= i <= 1.0 for i in values):
            processed = [
                values[0] * image_width,
                values[1] * image_height,
                values[2] * image_width,
                values[3] * image_height,
            ]
        elif 0 <= values[0] <= image_width and 0 <= values[2] <= image_width and 0 <= values[1] <= image_height and 0 <= values[3] <= image_height:
            processed = values
        else:
            processed = self.client.data_process(values, image_width, image_height)

        x1, y1, x2, y2 = [int(round(i)) for i in processed]
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        x1 = max(0, min(image_width - 1, x1))
        x2 = max(0, min(image_width - 1, x2))
        y1 = max(0, min(image_height - 1, y1))
        y2 = max(0, min(image_height - 1, y2))
        if x2 - x1 < 4 or y2 - y1 < 4:
            self.get_logger().info(f'vllm box too small after normalize: raw={box} processed={[x1, y1, x2, y2]}')
            return []
        return [x1, y1, x2, y2]

    def llm_result_callback(self, msg):
        self.vllm_result = msg.data

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            if self.vllm_result:
                try:
                    result_text = self.vllm_result
                    if result_text.startswith("```") and result_text.endswith("```"):
                        result_text = result_text.strip("```").replace("json\n", "").strip()
                    result = json.loads(result_text)
                    raw_box = result['xyxy']
                    box = self.normalize_box(raw_box, 640, 400)
                    label = result['object']
                    response = result['response']
                    self.get_logger().info(f'vllm target label={label} raw_box={raw_box} box={box}')
                    if not box:
                        self.get_logger().info(f'invalid target box: {raw_box}')
                        msg = String()
                        msg.data = response
                        self.tts_text_pub.publish(msg)
                        self.action_finish = True
                        self.vllm_result = ''
                        continue

                    msg = SetBox.Request()
                    msg.label = label
                    msg.box = box
                    res = self.send_request(self.set_target_client, msg)
                    self.get_logger().info(f'set_target response success={res.success} message={res.message}')
                    # Enable sorting. 开启分拣
                    msg = SetBool.Request()
                    msg.data = True
                    res = self.send_request(self.start_client, msg)
                    self.get_logger().info(f'enable_transport response success={res.success} message={res.message}')
                    time.sleep(2)
                    msg = String()
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                except BaseException as e:
                    msg = String()
                    msg.data = self.vllm_result
                    self.tts_text_pub.publish(msg)
                    self.get_logger().info(f'error: {e}')
                self.action_finish = True
                self.vllm_result = ''
            else:
                time.sleep(0.01)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
        rclpy.shutdown()

def main():
    node = VLLMDietitianl('vllm_dietitianl')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
