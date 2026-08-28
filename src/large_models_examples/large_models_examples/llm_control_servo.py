#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/12/03
import ast
import time
import rclpy
import threading
from speech import speech
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Empty, Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from large_models.config import *
from large_models_msgs.srv import SetModel, SetString
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    PROMPT = '''
#角色任务
你是一款智能机械臂，需要根据输入内容生成基于统一坐标系的机械臂动作指令。

##要求与限制
1. 所有机械臂动作统一使用末端坐标控制，不要使用舵机ID、脉宽、脉冲值。
2. 坐标系定义：前方为x轴正方向，左侧为y轴正方向，上方为z轴正方向，单位为米。
3. pitch、roll、claw 的单位都是角度；向下抓取常用 pitch 为 -90。
4. 如果没有指定运行时间，固定为1秒。
5. 如果两个动作之间没有停顿，就尽量合并成一个动作。
6. 直接输出json结果，不要分析，不要输出多余内容。
7. 格式：{{'action':['xx', 'xx'], 'response':'xx'}}

##默认待机姿态（home）
- x=0.20米, y=0.00米, z=0.20米, pitch=0°, roll=0°, claw=0°

##当前机械臂姿态
{current_pose_text}

##工作空间限制（超出范围的坐标必须自动钳位到边界值）
- x范围：0.10～0.48米（前后方向）
- y范围：-0.45～0.45米（左右方向，左正右负）
- z范围：0.02～0.50米（高度方向）
- 如果用户要求的位置超出以上范围，将坐标限制在最近的边界值，并在response中提醒用户已调整

##结构要求
- action 是按执行顺序排列的函数字符串数组；找不到对应动作时返回 []。
- response 是 10 到 30 个字的简短回复。

##动作函数库
- 移动到指定坐标姿态：set_pose(x, y, z, pitch, roll, claw, duration)
- 在当前位置基础上增量移动：move(dx, dy, dz, duration)  — 请根据「当前机械臂姿态」计算增量
- 单独控制夹爪开合：set_claw(claw, duration)
- 回到默认待机位：home(duration)
- 延时指定时间：time.sleep(1)

##重要提示
- 用户说"往前/往后/往左/往右/往上/往下"等相对方向时，请使用 move() 函数，根据当前姿态做增量移动。
- 用户说"移动到某个位置"时，请使用 set_pose() 函数。
- 当前姿态会在每次指令前自动更新，请以此为基准计算。

##任务示例
输入：移动到前方20厘米、左边5厘米、高18厘米的位置
输出：{{'action':['set_pose(0.20, 0.05, 0.18, -90, 0, 0, 1)'], 'response':'好的，移动到指定位置'}}
输入：向右移动3厘米
输出：{{'action':['move(0, -0.03, 0, 1)'], 'response':'收到，向右微调一下'}}
输入：往前20厘米
输出：{{'action':['move(0.20, 0, 0, 1)'], 'response':'好的，往前移动20厘米'}}
输入：夹爪闭合
输出：{{'action':['set_claw(30, 0.6)'], 'response':'好嘞，准备夹住'}}
输入：回到初始位置
输出：{{'action':['home(1.5)'], 'response':'收到，这就回待机位'}}
'''
else:
    PROMPT = '''
#Role
You are an intelligent robot arm that must generate motion commands using a unified Cartesian coordinate interface.

##Requirements and Restrictions
1. Use Cartesian end-effector control only. Do not output servo IDs, pulse widths, or raw servo values.
2. Coordinate system: forward is +x, left is +y, up is +z, and the unit is meters.
3. pitch, roll, and claw are all in degrees. A common top-down grasp uses pitch = -90.
4. If duration is not specified, use 1 second.
5. If two actions have no pause between them, merge them when possible.
6. Output JSON only. Do not include analysis or extra text.
7. Format: {{"action": ["xx", "xx"], "response": "xx"}}

##Default Home Pose
- x=0.20m, y=0.00m, z=0.20m, pitch=0°, roll=0°, claw=0°

##Current Arm Pose
{current_pose_text}

##Workspace Limits (coordinates exceeding the range must be clamped to boundary values)
- x range: 0.10 to 0.48 meters (forward/backward)
- y range: -0.45 to 0.45 meters (left is positive, right is negative)
- z range: 0.02 to 0.50 meters (height)
- If the requested position exceeds these limits, clamp coordinates to the nearest boundary and inform the user in the response

##Structure
- action is an ordered list of function-call strings. If no matching action exists, return [].
- response is a short 10-30 word reply.

##Action Function Library
- Move to a target pose: set_pose(x, y, z, pitch, roll, claw, duration)
- Move incrementally from the current pose: move(dx, dy, dz, duration) — calculate delta based on "Current Arm Pose"
- Control gripper only: set_claw(claw, duration)
- Return to default home pose: home(duration)
- Delay: time.sleep(1)

##Important Notes
- When the user says "forward/backward/left/right/up/down", use move() for incremental motion based on the current pose.
- When the user says "move to a position", use set_pose() for absolute positioning.
- The current pose is automatically updated before each command.

##Examples
Input: Move to a point 20 cm forward, 5 cm left, and 18 cm high
Output: {{"action": ["set_pose(0.20, 0.05, 0.18, -90, 0, 0, 1)"], "response": "Got it, moving to the target pose."}}
Input: Move 3 cm to the right
Output: {{"action": ["move(0, -0.03, 0, 1)"], "response": "Sure, making a slight right adjustment."}}
Input: Move forward 20 cm
Output: {{"action": ["move(0.20, 0, 0, 1)"], "response": "Got it, moving forward 20 cm."}}
Input: Close the gripper
Output: {{"action": ["set_claw(30, 0.6)"], "response": "Okay, closing the gripper now."}}
Input: Return to the home pose
Output: {{"action": ["home(1.5)"], "response": "On it, returning to the standby pose."}}
'''


class LLMControlServo(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.action = []
        self.interrupt = False
        self.llm_result = ''
        self.action_finish = False
        self.play_audio_finish = False
        self.running = True
        self.current_pose = None
        self.known_pose = dict(DEFAULT_ARM_POSE)
        self.current_position = [self.known_pose['x'], self.known_pose['y'], self.known_pose['z']]
        self.current_pitch = self.known_pose['pitch']
        self.current_roll = self.known_pose['roll']
        self.current_claw = self.known_pose['claw']
        self.wakeup_enabled = False
        self.language = os.environ.get("ASR_LANGUAGE", "Chinese")

        timer_cb_group = ReentrantCallbackGroup()      
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')     
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)  

        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()     
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
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
        if os.environ["ASR_MODE"] == 'offline':
            msg.model_type = 'llm'
            msg.model = 'qwen3:1.7b'
            msg.base_url = ollama_host
        else:
            msg.model = llm_model
            msg.model_type = 'llm'
            msg.api_key = api_key
            msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        self.home(1.5)        
        self.sync_internal_pose_from_snapshot()
        self.update_prompt_with_pose()
        # self.enable_wakeup(True)

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

    def sync_internal_pose_from_snapshot(self):
        pose = get_pose_snapshot(self)
        self.current_position = [float(pose['x']), float(pose['y']), float(pose['z'])]
        self.current_pitch = float(pose['pitch'])
        self.current_roll = float(pose['roll'])
        self.current_claw = float(pose['claw'])

    def get_current_pose_text(self):
        x_m = self.current_position[0] / 1000.0
        y_m = self.current_position[1] / 1000.0
        z_m = self.current_position[2] / 1000.0
        if self.language == 'Chinese':
            return (f'- x={x_m:.3f}米, y={y_m:.3f}米, z={z_m:.3f}米, '
                    f'pitch={self.current_pitch:.1f}°, roll={self.current_roll:.1f}°, '
                    f'claw={self.current_claw:.1f}°')
        else:
            return (f'- x={x_m:.3f}m, y={y_m:.3f}m, z={z_m:.3f}m, '
                    f'pitch={self.current_pitch:.1f}°, roll={self.current_roll:.1f}°, '
                    f'claw={self.current_claw:.1f}°')

    def update_prompt_with_pose(self):
        pose_text = self.get_current_pose_text()
        filled_prompt = PROMPT.format(current_pose_text=pose_text)
        msg = SetString.Request()
        msg.data = filled_prompt
        self.send_request(self.set_prompt_client, msg)

    def wakeup_callback(self, msg):
        if not msg.data:
            return
        if self.action_finish or self.llm_result:
            self.get_logger().info('wakeup interrupt')
            self.interrupt = True

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def home(self, duration=1.5):
        home = load_scene_home_pose()
        self.publish_arm(home['x'], home['y'], home['z'], home['pitch'], home['roll'], home['claw'], int(float(duration) * 1000.0))
        time.sleep(max(float(duration), 0.1) + 0.3)
        self.sync_internal_pose_from_snapshot()

    def set_pose(self, x, y, z, pitch=-90.0, roll=0.0, claw=None, duration=1.0):
        if claw is None:
            claw = self.current_claw
        x_mm = float(x) * 1000.0
        y_mm = float(y) * 1000.0
        z_mm = float(z) * 1000.0
        self.current_position = [x_mm, y_mm, z_mm]
        self.current_pitch = float(pitch)
        self.current_roll = float(roll)
        self.current_claw = float(claw)
        self.publish_arm(x_mm, y_mm, z_mm, self.current_pitch, self.current_roll, self.current_claw, int(float(duration) * 1000.0))
        time.sleep(max(float(duration), 0.05) + 0.05)

    def set_position(self, x, y, z, pitch=-90.0, roll=0.0, claw=None, duration=1.0):
        self.set_pose(x, y, z, pitch, roll, claw, duration)

    def set_claw(self, claw, duration=0.6):
        self.current_claw = float(claw)
        self.publish_arm(
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            self.current_pitch,
            self.current_roll,
            self.current_claw,
            int(float(duration) * 1000.0),
        )
        time.sleep(max(float(duration), 0.05) + 0.05)

    def move(self, x, y, z, duration=1.0):
        self.current_position[0] += float(x) * 1000.0
        self.current_position[1] += float(y) * 1000.0
        self.current_position[2] += float(z) * 1000.0
        self.publish_arm(
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            self.current_pitch,
            self.current_roll,
            self.current_claw,
            int(float(duration) * 1000.0),
        )
        time.sleep(max(float(duration), 0.05) + 0.05)

    def play_audio_finish_callback(self, msg):
        msg = SetBool.Request()
        msg.data = True
        self.send_request(self.awake_client, msg)
        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            if self.llm_result:
                raw_result = self.llm_result
                msg = String()
                try:
                    if 'action' in raw_result and '{' in raw_result and '}' in raw_result:
                        payload = raw_result[raw_result.find('{'):raw_result.rfind('}') + 1]
                        try:
                            result = ast.literal_eval(payload)
                        except Exception:
                            result = eval(payload)
                        action_list = result.get('action', [])
                        response = result.get('response', '')
                        if isinstance(action_list, str):
                            action_list = [action_list]
                        elif isinstance(action_list, tuple):
                            action_list = list(action_list)
                        elif action_list is None:
                            action_list = []
                        elif not isinstance(action_list, list):
                            action_list = [str(action_list)]
                        msg.data = response
                        self.tts_text_pub.publish(msg)
                        for i in action_list:
                            if self.interrupt:
                                self.get_logger().info('interrupt')
                                break
                            command = str(i).strip()
                            if not command:
                                continue
                            if any(command.startswith(prefix) for prefix in ('set_pose', 'set_position', 'move', 'set_claw', 'home')):
                                eval('self.' + command)
                            else:
                                eval(command)
                    else:
                        msg.data = raw_result
                        self.tts_text_pub.publish(msg)
                except BaseException as e:
                    self.get_logger().error(f'llm_control_servo process error: {e}')
                    msg.data = raw_result
                    self.tts_text_pub.publish(msg)
                self.action_finish = True
                self.llm_result = ''
                # 动作执行完毕，同步当前姿态并更新 prompt
                self.sync_internal_pose_from_snapshot()
                self.update_prompt_with_pose()
                self.get_logger().info(f'姿态已更新: {self.get_current_pose_text()}')
            else:
                time.sleep(0.01)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
                self.interrupt = False
                # self.enable_wakeup(True)
        rclpy.shutdown()


def main():
    node = LLMControlServo('llm_control_servo')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
