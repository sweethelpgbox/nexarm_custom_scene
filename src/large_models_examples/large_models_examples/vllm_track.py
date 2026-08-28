#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import cv2
import json
import time
import queue
import rclpy
import threading
import sdk.fps as fps
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Empty, Trigger
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel
from large_models_examples.track_anything import ObjectTracker
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    PROMPT = '''
你作为图像识别专家，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出。
## 1. 理解用户指令
我会给你一句话，你需要根据我的话做出最佳决策，从做出的决策中提取「物体名称」, **object对应的name要用英文表示**, **不要输出没有提及到的物体**
## 2. 理解图片
我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标, **不要输出没有提及到的物体**
【特别注意】： 要深刻理解物体的方位关系
## 输出格式（请仅输出以下内容，不要说任何多余的话)
{
    "object": name, 
    "xyxy": [xmin, ymin, xmax, ymax]
}
    '''
else:
    PROMPT = '''
**Role:
You are an expert in image recognition who precisely detects and locates objects in images.

**Task:
1.Instruction Parsing:
You will receive a sentence from the user.
Extract only the relevant "object name" mentioned in the sentence.
The object name must be in English.
Note: Do not output any object that is not mentioned.

2.Image Analysis:
You will be provided with an image.
Locate the object corresponding to the extracted "object name" in the image.
Determine its bounding box by finding the pixel coordinates of the object's top-left and bottom-right corners.
Note: Only output the bounding box for the mentioned object.

3.Orientation Awareness:
Ensure you fully understand the spatial orientation of the object in the image.

**Output Format(Your final output must be a single JSON object with the following structure. The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1]):
{
  "object": "object_name_in_English",
  "xyxy": [xmin, ymin, xmax, ymax]
}

**Output Example:
{
  "object": "red",
  "xyxy": [0.1, 0.3, 0.4, 0.6]
}

**Important Instructions:
Do not include any additional text, explanations, or thought processes.
Output only the final JSON result as specified above.
Do not output any extra keys or comments.
    '''


class VLLMTrack(Node):
    INIT_X = DEFAULT_ARM_POSE['x']
    INIT_Y = DEFAULT_ARM_POSE['y']
    INIT_Z = DEFAULT_ARM_POSE['z']
    INIT_PITCH = DEFAULT_ARM_POSE['pitch']
    TRACK_PITCH = 0.0
    INIT_ROLL = DEFAULT_ARM_POSE['roll']
    INIT_CLAW = DEFAULT_ARM_POSE['claw']

    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.vllm_result = ''
        self.running = True
        self.start_track = False
        self.action_finish = False
        self.play_audio_finish = False
        self.stop = False
        self.language = os.environ["ASR_LANGUAGE"]
        self.bridge = CvBridge()
        self.track = ObjectTracker()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.current_pose = None
        self.known_pose = dict(DEFAULT_ARM_POSE)
        self.track_result = None

        timer_cb_group = ReentrantCallbackGroup()
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.result_image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        self.pid_params = {
            'kp1': 0.065, 'ki1': 0.0, 'kd1': 0.001,
            'kp2': 0.00004, 'ki2': 0.0, 'kd2': 0.0,
            'kp3': 0.05, 'ki3': 0.0, 'kd3': 0.0,
        }
        for param_name, default_value in self.pid_params.items():
            self.declare_parameter(param_name, default_value)
            self.pid_params[param_name] = self.get_parameter(param_name).value
        self.track.update_pid(
            [self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
            [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']],
            [self.pid_params['kp3'], self.pid_params['ki3'], self.pid_params['kd3']],
        )
        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def on_parameter_update(self, params):
        for param in params:
            if param.name in self.pid_params:
                self.pid_params[param.name] = param.value
        self.get_logger().info(f'PID parameters updated: {self.pid_params}')
        self.track.update_pid(
            [self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
            [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']],
            [self.pid_params['kp3'], self.pid_params['ki3'], self.pid_params['kd3']],
        )
        return SetParametersResult(successful=True)

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
        if self.language == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
            msg.model = vllm_model
        self.send_request(self.set_model_client, msg, timeout=15.0)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg, timeout=15.0)

        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.TRACK_PITCH, self.INIT_ROLL, self.INIT_CLAW, 1500)
        time.sleep(1.5)
        pose = get_pose_snapshot(self)
        self.track.set_init_param(pose['x'], pose['y'], pose['z'])

        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        threading.Thread(target=self.track_thread, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg, timeout=3.0):
        future = client.call_async(msg)
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if future.done() and future.result():
                return future.result()
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f'服务调用超时: {client.srv_name}')
        return future.result() if future.done() else None

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

    def wakeup_callback(self, msg):
        if msg.data:
            self.get_logger().info('wakeup interrupt')
            self.track.stop()
            self.start_track = False
            self.stop = True
            self.track_result = None

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result:
                try:
                    result = json.loads(self.vllm_result)
                    box = result['xyxy']
                    box = self.client.data_process(box, 640, 400)
                    box = [box[0], box[1], box[2] - box[0], box[3] - box[1]]
                    self.get_logger().info('box: %s' % str(box))
                    self.track.set_track_target(box, image)
                    self.start_track = True
                    self.stop = False
                    speech.play_audio(start_track_audio_path, block=False)
                except (ValueError, TypeError) as e:
                    self.start_track = False
                    msg = String()
                    msg.data = self.vllm_result
                    self.tts_text_pub.publish(msg)
                    self.get_logger().info(f'track parse failed: {e}')
                self.vllm_result = ''
                # 非阻塞唤醒，避免卡住追踪循环
                threading.Thread(target=self._enable_wakeup, daemon=True).start()
            if self.start_track and not self.stop:
                self.track_result = self.track.track(image)
            self.fps.update()
            self.fps.show_fps(image)
            cv2.imshow('image', image)
            cv2.waitKey(1)
            # self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

    def _enable_wakeup(self):
        try:
            wake = SetBool.Request()
            wake.data = True
            self.send_request(self.awake_client, wake, timeout=3.0)
        except Exception as e:
            self.get_logger().warn(f'唤醒服务调用失败: {e}')

    def track_thread(self):
        while self.running:
            if self.track_result is not None and not self.stop:
                y_mm, z_mm, _ = self.track_result
                self.publish_arm(self.INIT_X, y_mm, z_mm, self.TRACK_PITCH, self.INIT_ROLL, self.INIT_CLAW, 80)
                self.track_result = None
                time.sleep(0.05)
            else:
                time.sleep(0.01)

    def image_callback(self, ros_image):
        try:
            _, cv_image = decode_color_image(ros_image)
        except Exception as e:
            self.get_logger().error(f"图像转换出错：{e}")
            return
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(cv_image)


def main():
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
