#!/usr/bin/python3
# coding=utf8
# @Author: Aiden
# @Date: 2024/12/31
# pick_and_place — 新底层 ArmCoords 版本
# 参考旧版 jetarm 的分步流程和时序
import os
import math
import time
import rclpy
import yaml
import numpy as np
from ros_robot_controller_msgs.msg import ArmCoords

stop = False
chassis_type = os.environ.get('CHASSIS_TYPE', '')
APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
DEFAULT_HOME_POSE = {
    'x': 110.0,
    'y': 0.0,
    'z': 220.0,
    'pitch': -90.0,
    'roll': 0.0,
    'claw': 0.0,
}

def interrupt(status=False):
    global stop
    stop = status


def publish_arm(arm_pub, x, y, z, pitch, roll, claw, time_ms):
    msg = ArmCoords()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    msg.pitch = float(pitch)
    msg.roll = float(roll)
    msg.claw = float(claw)
    msg.time_ms = int(time_ms)
    arm_pub.publish(msg)


def _scene_config_path():
    if os.environ.get('CHASSIS_TYPE', '') == 'Slide_Rails':
        return STEPPER_SCENE_CONFIG
    return APP_SCENE_CONFIG


def load_scene_home_pose():
    home = dict(DEFAULT_HOME_POSE)
    home['time_ms'] = 2000
    try:
        with open(_scene_config_path(), 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if isinstance(scenes, dict) and scenes:
            scene_name = str(cfg.get('current_scene', 'scene_1'))
            if scene_name not in scenes:
                scene_name = next(iter(scenes.keys()))
            scene_cfg = scenes.get(scene_name, {}) if isinstance(scenes.get(scene_name), dict) else {}
            if scene_name == "scene_4" and isinstance(scene_cfg.get('calibration_pose'), dict):
                hp = scene_cfg.get('calibration_pose', {})
            else:
                hp = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
            for key, default_value in DEFAULT_HOME_POSE.items():
                home[key] = float(hp.get(key, default_value))
            try:
                home['time_ms'] = int(float(hp.get('time_ms', 2000)))
            except Exception:
                home['time_ms'] = 2000
    except Exception:
        pass
    return home


CLAW_OPEN = -100.0
CLAW_GRAB = 10.0
CLAW_FULL_CLOSE = 60.0


def pulse_to_claw(gripper_pulse):
    """旧版夹爪脉冲值 → ArmCoords.claw 角度"""
    pulse = float(gripper_pulse)
    if pulse <= 220.0:
        return CLAW_OPEN
    pulse = max(470.0, min(700.0, pulse))
    return CLAW_GRAB + (pulse - 470.0) * (CLAW_FULL_CLOSE - CLAW_GRAB) / (700.0 - 470.0)


dt = 0.1
d = 0.015
PICK_STAGE_CLEARANCE = 0.06
PICK_PRE_GRASP_CLEARANCE = 0.01
PICK_LIFT_CLEARANCE = 0.03
PICK_RESET_CLEARANCE = 0.01
PICK_FINAL_DESCENT_MARGIN = 0.005
PLACE_STAGE_CLEARANCE = 0.03
PLACE_PRE_RELEASE_CLEARANCE = 0.012
PLACE_RESET_CLEARANCE = 0.01
PLACE_EXTRA_DESCENT = 0.01


def pick_without_back(
    position,
    pitch,
    yaw,
    gripper_angle,
    gripper_depth,
    arm_pub,
    kinematics_client=None,
    interpolation=False,
    *,
    claw_grab_angle=None,
    dry_run=False,
):
    """
    抓取物体但不回 home
    position: [x, y, z] 米
    pitch: 俯仰角(度)
    yaw: 夹爪旋转角(度), 传给 ArmCoords.roll
    gripper_angle: 夹爪开合控制量（旧版脉冲值）
    gripper_depth: 下探深度(米)
    """
    global stop
    if not stop:
        x_mm = position[0] * 1000.0
        y_mm = position[1] * 1000.0
        z_stage = (position[2] + PICK_STAGE_CLEARANCE) * 1000.0
        z_pre_grasp = (position[2] + PICK_PRE_GRASP_CLEARANCE) * 1000.0
        z_lift = (position[2] + PICK_LIFT_CLEARANCE) * 1000.0
        roll = float(yaw)
        base_yaw_deg = math.degrees(math.atan2(y_mm, x_mm))
        claw_open = CLAW_OPEN
        claw_grab = float(claw_grab_angle) if claw_grab_angle is not None else pulse_to_claw(gripper_angle)
        arm_pitch = -abs(float(pitch))
        effective_depth = max(0.0, float(gripper_depth) - PICK_FINAL_DESCENT_MARGIN)
        z_down = (position[2] - effective_depth) * 1000.0

        # Step 1: 移到目标上方，腕部反向抵消底座旋转保持世界坐标0°
        publish_arm(arm_pub, x_mm, y_mm, z_stage, arm_pitch, -base_yaw_deg, claw_open, 1200)
        time.sleep(1.5)

        # Step 2: 下到预夹取高度，同时转腕部；此时仍离物体约 10mm
        publish_arm(arm_pub, x_mm, y_mm, z_pre_grasp, arm_pitch, roll, claw_open, 1500)
        time.sleep(1.8)

        if dry_run:
            return True

        if not stop:
            # Step 3: 腕部已到目标角度后，只改 z 慢速下探
            publish_arm(arm_pub, x_mm, y_mm, z_down, arm_pitch, roll, claw_open, 800)
            time.sleep(1.0)

            # Step 4: 夹取
            publish_arm(arm_pub, x_mm, y_mm, z_down, arm_pitch, roll, claw_grab, 800)
            time.sleep(1.0)

            # Step 5: 抬起到上方 (保持夹紧)
            publish_arm(arm_pub, x_mm, y_mm, z_lift, arm_pitch, roll, claw_grab, 800)
            time.sleep(1.0)
            return True
    return False


def pick(
    position,
    pitch,
    yaw,
    gripper_angle,
    gripper_depth,
    arm_pub,
    kinematics_client=None,
    interpolation=False,
    *,
    claw_grab_angle=None,
):
    """抓取物体并回到安全位"""
    if pick_without_back(
        position,
        pitch,
        yaw,
        gripper_angle,
        gripper_depth,
        arm_pub,
        kinematics_client,
        interpolation,
        claw_grab_angle=claw_grab_angle,
    ):
        if not stop:
            x_mm = position[0] * 1000.0
            y_mm = position[1] * 1000.0
            z_mm = (position[2] + PICK_LIFT_CLEARANCE) * 1000.0
            z_reset = (position[2] + PICK_LIFT_CLEARANCE + PICK_RESET_CLEARANCE) * 1000.0
            arm_pitch = -abs(float(pitch))
            claw_grab = float(claw_grab_angle) if claw_grab_angle is not None else pulse_to_claw(gripper_angle)

            # Step 6: 抬高一点并恢复 roll=0；ArmCoords 需要坐标也变化才稳定执行 roll
            publish_arm(arm_pub, x_mm, y_mm, z_reset, arm_pitch, 0.0, claw_grab, 500)
            time.sleep(0.8)

            # Step 7: 回当前场景默认位，保持夹紧
            home = load_scene_home_pose()
            move_ms = int(home.get('time_ms', 2000))
            publish_arm(arm_pub, home['x'], home['y'], home['z'], home['pitch'], home['roll'], claw_grab, move_ms)
            time.sleep(max(0.0, move_ms / 1000.0) + 0.3)
            return True
    return False


def place(
    position,
    pitch,
    yaw,
    gripper_angle,
    arm_pub,
    kinematics_client=None,
    interpolation=False,
    *,
    claw_hold_angle=None,
):
    """
    放置物体
    position: [x, y, z] 米
    pitch: 俯仰角(度)
    yaw: 夹爪旋转角(度)
    gripper_angle: 夹爪开合控制量
    """
    global stop
    if not stop:
        x_mm = position[0] * 1000.0
        y_mm = position[1] * 1000.0
        z_above = (position[2] + PLACE_STAGE_CLEARANCE) * 1000.0
        z_pre_release = (position[2] + PLACE_PRE_RELEASE_CLEARANCE) * 1000.0
        z_reset = (position[2] + PLACE_STAGE_CLEARANCE + PLACE_RESET_CLEARANCE) * 1000.0
        z_mm = (position[2] - PLACE_EXTRA_DESCENT) * 1000.0
        roll = float(yaw)
        arm_pitch = -abs(float(pitch))
        claw_hold = float(claw_hold_angle) if claw_hold_angle is not None else CLAW_GRAB

        # Step 1: 移到放置点上方 (roll=0, 保持夹紧)
        publish_arm(arm_pub, x_mm, y_mm, z_above, arm_pitch, 0.0, claw_hold, 1500)
        time.sleep(1.7)

        # Step 2: 下到预放置高度，同时转腕部；仍离放置面约 12mm
        publish_arm(arm_pub, x_mm, y_mm, z_pre_release, arm_pitch, roll, claw_hold, 1500)
        time.sleep(1.8)

        if not stop:
            # Step 3: 下降到放置高度 (保持夹紧)
            publish_arm(arm_pub, x_mm, y_mm, z_mm, arm_pitch, roll, claw_hold, 800)
            time.sleep(1.0)

            # Step 4: 张开夹爪放下物体
            publish_arm(arm_pub, x_mm, y_mm, z_mm, arm_pitch, roll, CLAW_OPEN, 400)
            time.sleep(0.6)

            # Step 5: 抬起到上方
            publish_arm(arm_pub, x_mm, y_mm, z_above, arm_pitch, roll, CLAW_OPEN, 1000)
            time.sleep(1.2)

            # Step 6: 抬高一点并恢复 roll=0；避免只改 roll 的 ArmCoords 被底层忽略
            publish_arm(arm_pub, x_mm, y_mm, z_reset, arm_pitch, 0.0, CLAW_OPEN, 800)
            time.sleep(1.0)

            # Step 7: 回当前场景默认位
            home = load_scene_home_pose()
            move_ms = int(home.get('time_ms', 2000))
            publish_arm(arm_pub, home['x'], home['y'], home['z'], home['pitch'], home['roll'], CLAW_OPEN, move_ms)
            time.sleep(max(0.0, move_ms / 1000.0) + 0.3)
            return True
    return False
