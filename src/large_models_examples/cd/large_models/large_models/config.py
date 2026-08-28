#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import math
import yaml
import cv2
import rclpy
import dashscope
import numpy as np
from sdk import common
from pathlib import Path
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.srv import GetArmFullState


###Offline###
offline_llm = 'qwen3:1.7b'
if os.environ["ASR_LANGUAGE"] == 'Chinese':
    offline_asr = 'sherpa-onnx-streaming-zipformer-zh-xlarge-int8-2025-06-30'
    offline_tts = 'matcha-icefall-zh-baker'
else:
    offline_asr = 'sherpa-onnx-streaming-zipformer-en-2023-06-21'
    offline_tts = 'vits-ljs'
offline_tts_speaker = 100
offline_punct_model = 'sherpa-onnx-punct-ct-transformer-zh-en-vocab272727-2024-04-12'
ollama_host = 'http://localhost:11434'
sherpa_onnx_path = os.path.join(Path.home(), 'third_party/sherpa-onnx')

stepfun_api_key = 'zrM1ZxMP76rNbqedK1xA1CHPqwDv2wHzuk870PD5CpgypP53MI5FQVqrFUqNSngs'
stepfun_base_url = 'https://api.stepfun.com/v1'
stepfun_llm_model = ''
#'step-1v-8k'/'step-1o-vision-32k'/'step-1.5v-mini'
stepfun_vllm_model = 'step-1o-vision-32k'

# 阿里云key
aliyun_api_key = 'sk-f4c807239f204aeea7e3dc34e84f0d65'
aliyun_base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
aliyun_llm_model = 'qwen3.7-max'#'qwen-turbo'#'qwen-max-latest'
aliyun_vllm_model = 'qwen-vl-max-latest'
aliyun_tts_model = 'sambert-zhinan-v1'
aliyun_asr_model = 'paraformer-realtime-v2'
aliyun_voice_model = ''

###Internationally###
vllm_api_key = ''
vllm_base_url = 'https://openrouter.ai/api/v1'
vllm_model = 'qwen/qwen2.5-vl-72b-instruct:free'

llm_api_key = ''
llm_base_url = 'https://api.openai.com/v1'
llm_model = 'gpt-4o-mini'
openai_vllm_model = 'gpt-4o'
openai_tts_model = 'tts-1'
openai_asr_model = 'whisper-1'
openai_voice_model = 'onyx'
######

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    # The actual key used for invocation(实际调用的key)
    api_key = aliyun_api_key
    dashscope.api_key = aliyun_api_key
    base_url = aliyun_base_url
    asr_model = aliyun_asr_model
    tts_model = aliyun_tts_model
    voice_model = aliyun_voice_model
    llm_model = aliyun_llm_model
    vllm_model = aliyun_vllm_model
else:
    api_key = llm_api_key
    os.environ["OPENAI_API_KEY"] = api_key
    base_url = llm_base_url
    asr_model = openai_asr_model
    tts_model = openai_tts_model
    voice_model = openai_voice_model

# Get the path of the current program(获取程序所在路径)
code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline')
    else:
        audio_path = os.path.join(code_path, 'resources/audio')
else:
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline/en')
    else:
        audio_path = os.path.join(code_path, 'resources/audio/en')

# Path to the recorded audio(录音音频的路径)
recording_audio_path = os.path.join(audio_path, 'recording.wav')

# Path to the synthesized (TTS) audio(语音合成音频的路径)
tts_audio_path = os.path.join(audio_path, "tts_audio.wav")

# Path to the startup audio(启动音频的路径)
start_audio_path = os.path.join(audio_path, "start_audio.wav")

# Path to the wake-up response audio(唤醒回答音频的路径)
wakeup_audio_path = os.path.join(audio_path, "wakeup.wav")

# Path to the error audio(出错音频的路径)
error_audio_path = os.path.join(audio_path, "error.wav")

# Path to the audio played when no sound is detected(没有检测到声音时音频的路径)
no_voice_audio_path = os.path.join(audio_path, "no_voice.wav")

# Path to the audio played when recording is complete(录音完成时音频的路径)
dong_audio_path = os.path.join(audio_path, "dong.wav")

record_finish_audio_path = os.path.join(audio_path, "record_finish.wav")

start_track_audio_path = os.path.join(audio_path, "start_track.wav")

track_fail_audio_path = os.path.join(audio_path, "track_fail.wav")

APP_CONFIG_PATH = "/home/ubuntu/ros2_ws/src/app/config"
PERIPHERALS_CONFIG_PATH = "/home/ubuntu/ros2_ws/src/peripherals/config"
CAMERA_INFO_PATH = os.path.join(PERIPHERALS_CONFIG_PATH, 'camera_info.yaml')
TRANSFORM_PATH = os.path.join(APP_CONFIG_PATH, 'transform.yaml')
CALIBRATION_PATH = os.path.join(APP_CONFIG_PATH, 'calibration.yaml')
SCENE_CONFIG_PATH_APP = os.path.join(APP_CONFIG_PATH, 'calibration_scene.yaml')
SCENE_CONFIG_PATH_STEPPER = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"


def _get_scene_config_path():
    chassis_type = os.environ.get('CHASSIS_TYPE', '')
    if chassis_type == 'Slide_Rails':
        return SCENE_CONFIG_PATH_STEPPER
    return SCENE_CONFIG_PATH_APP


def load_scene_home_pose():
    home = {
        'x': 105.0,
        'y': 0.0,
        'z': 200.0,
        'pitch': -90.0,
        'roll': 0.0,
        'claw': 0.0,
    }
    scene_path = _get_scene_config_path()
    try:
        cfg = common.get_yaml_data(scene_path) or {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if isinstance(scenes, dict) and scenes:
            scene_name = str(cfg.get('current_scene', 'scene_1'))
            if scene_name not in scenes:
                scene_name = next(iter(scenes.keys()))
            scene_cfg = scenes.get(scene_name, {}) if isinstance(scenes.get(scene_name), dict) else {}
            hp = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
            home['x'] = float(hp.get('x', home['x']))
            home['y'] = float(hp.get('y', home['y']))
            home['z'] = float(hp.get('z', home['z']))
            home['pitch'] = float(hp.get('pitch', home['pitch']))
            home['roll'] = float(hp.get('roll', home['roll']))
            home['claw'] = float(hp.get('claw', home['claw']))
    except Exception:
        pass
    return home

HOME_POSE = load_scene_home_pose()
DEFAULT_ARM_POSE = {
    'x': HOME_POSE['x'],
    'y': HOME_POSE['y'],
    'z': HOME_POSE['z'],
    'pitch': HOME_POSE['pitch'],
    'roll': HOME_POSE['roll'],
    'claw': HOME_POSE['claw'],
    'yaw': 0.0,
}


def arm_pose_dict(x, y, z, pitch, roll, claw, yaw=None, joint_angles=None):
    if yaw is None:
        yaw = math.degrees(math.atan2(float(y), float(x))) if (float(x) != 0.0 or float(y) != 0.0) else 0.0
    pose = {
        'x': float(x),
        'y': float(y),
        'z': float(z),
        'pitch': float(pitch),
        'roll': float(roll),
        'claw': float(claw),
        'yaw': float(yaw),
    }
    if joint_angles is not None:
        pose['joint_angles'] = [float(v) for v in joint_angles]
    return pose


def update_known_pose(node, x, y, z, pitch, roll, claw):
    pose = arm_pose_dict(x, y, z, pitch, roll, claw)
    node.known_pose = dict(pose)
    return pose


def request_real_pose_snapshot(node, arm_state_client, timeout_sec=0.6):
    if arm_state_client is None:
        return None
    if not arm_state_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
        return None
    future = arm_state_client.call_async(GetArmFullState.Request())
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        if future.done():
            break
        time.sleep(0.01)
    if not future.done():
        return None
    try:
        response = future.result()
    except Exception as exc:
        if node is not None:
            node.get_logger().warn(f'获取真实末端状态失败: {exc}')
        return None
    if response is None or not response.success:
        return None
    pose = arm_pose_dict(
        response.x,
        response.y,
        response.z,
        response.pitch,
        response.roll,
        response.claw,
        response.yaw,
        response.joint_angles,
    )
    if node is not None:
        node.current_pose = dict(pose)
    return pose


def get_pose_snapshot(node, timeout_sec=0.6):
    return (
        getattr(node, 'current_pose', None)
        or request_real_pose_snapshot(node, getattr(node, 'arm_state_client', None), timeout_sec)
        or getattr(node, 'known_pose', None)
        or dict(DEFAULT_ARM_POSE)
    )


def get_endpoint_matrix(pose=None):
    p = pose if pose is not None else DEFAULT_ARM_POSE
    yaw_deg = p.get('yaw')
    if yaw_deg is None:
        yaw_deg = math.degrees(math.atan2(p['y'], p['x'])) if p['x'] or p['y'] else 0.0
    return common.xyz_euler_to_mat(
        [float(p['x']) / 1000.0, float(p['y']) / 1000.0, float(p['z']) / 1000.0],
        [float(p.get('roll', 0.0)), -float(p.get('pitch', 0.0)), float(yaw_deg)],
        degrees=True,
    )


def wait_for_arm_runtime_ready(node, arm_pub, controller_init_client=None, kinematics_init_client=None, timeout_sec=10.0):
    if controller_init_client is not None:
        if node is not None:
            node.get_logger().info('等待 controller_manager 初始化服务...')
        controller_init_client.wait_for_service()
    if kinematics_init_client is not None:
        if node is not None:
            node.get_logger().info('等待 kinematics 初始化服务...')
        kinematics_init_client.wait_for_service()

    if arm_pub is None:
        return True

    deadline = time.time() + timeout_sec
    while arm_pub.get_subscription_count() == 0 and time.time() < deadline:
        if node is not None:
            node.get_logger().info('等待机械臂坐标控制订阅...')
        time.sleep(0.2)
    return arm_pub.get_subscription_count() > 0


def wait_for_init_service(client, timeout_sec=10.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            return True
    return False


def load_hand2cam_tf_matrix(camera_info_path=CAMERA_INFO_PATH):
    try:
        with open(camera_info_path, 'r') as f:
            config = yaml.safe_load(f) or {}
        matrix = config.get('hand2cam_tf_matrix')
        if matrix is None:
            return None
        return np.array(matrix, dtype=np.float64)
    except Exception:
        return None


def load_transform_config(transform_path=TRANSFORM_PATH):
    config = common.get_yaml_data(transform_path)
    return {
        'plane': config.get('plane'),
        'corners': np.array(config['corners']) if 'corners' in config else None,
        'extristric': np.array(config['extristric']) if 'extristric' in config else None,
        'white_area_center': np.array(config['white_area_pose_world']) if 'white_area_pose_world' in config else None,
    }


def load_calibration_config(calibration_path=CALIBRATION_PATH):
    config = common.get_yaml_data(calibration_path)
    return {
        'depth_offset': tuple(float(v) for v in config['depth']['offset']),
        'depth_scale': tuple(float(v) for v in config['depth']['scale']),
        'kinematics_offset': tuple(float(v) for v in config['kinematics']['offset']),
        'kinematics_scale': tuple(float(v) for v in config['kinematics']['scale']),
    }


def apply_axis_calibration(position, offset, scale):
    result = [float(v) for v in position]
    for i in range(min(3, len(result))):
        result[i] = result[i] * float(scale[i]) + float(offset[i])
    return result


def decode_color_image(ros_image):
    height = int(ros_image.height)
    width = int(ros_image.width)
    encoding = str(getattr(ros_image, 'encoding', '') or '').lower()
    step = int(getattr(ros_image, 'step', 0) or 0)
    if step > 0 and width > 0:
        channels = max(1, step // width)
    else:
        channels = 4 if ('rgba' in encoding or 'bgra' in encoding) else 3

    shape = (height, width) if channels == 1 else (height, width, channels)
    raw_image = np.ndarray(shape=shape, dtype=np.uint8, buffer=ros_image.data)
    raw_image = np.copy(raw_image)

    if channels == 1:
        rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_GRAY2RGB)
    elif channels == 4:
        if encoding == 'rgba8':
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_RGBA2RGB)
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGRA2RGB)
    else:
        if encoding == 'rgb8':
            rgb_image = raw_image
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)

    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    return rgb_image, bgr_image


def normalize_gripper_roll_deg(angle_deg, limit_deg=120.0):
    normalized = ((float(angle_deg) + 90.0) % 180.0) - 90.0
    return float(np.clip(normalized, -limit_deg, limit_deg))
