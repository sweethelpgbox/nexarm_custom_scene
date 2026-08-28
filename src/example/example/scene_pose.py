#!/usr/bin/env python3
# coding: utf-8

import os
from sdk import common

APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
SCENE0_ID = 'scene_0'
SCENE0_HOME_POSE = {
    'x': 200.0,
    'y': 0.0,
    'z': 200.0,
    'pitch': -90.0,
    'roll': 0.0,
    'claw': 0.0,
    'time_ms': 2000,
}


def get_scene_config_path():
    if os.environ.get('CHASSIS_TYPE', '') == 'Slide_Rails':
        return STEPPER_SCENE_CONFIG
    return APP_SCENE_CONFIG


def _float_from_dict(data, key, default_value):
    try:
        return float(data.get(key, default_value))
    except Exception:
        return float(default_value)


def _int_from_dict(data, key, default_value):
    try:
        return int(float(data.get(key, default_value)))
    except Exception:
        return int(default_value)


def _resolve_active_scene(data, default_scene=None, scene_name=None):
    scenes = data.get('scenes') if isinstance(data, dict) else None
    if not isinstance(scenes, dict) or not scenes:
        return scene_name or os.environ.get('CALIBRATION_CURRENT_SCENE') or os.environ.get('CALIBRATION_DEFAULT_SCENE') or os.environ.get('SCENE') or default_scene or SCENE0_ID, {}, {}

    scenes.setdefault(SCENE0_ID, {
        'name': 'Scene 0',
        'length_m': 0.167,
        'width_m': 0.13,
        'home_pose': dict(SCENE0_HOME_POSE),
    })

    default_scene = default_scene or SCENE0_ID
    scene_name = (
        scene_name
        or os.environ.get('CALIBRATION_CURRENT_SCENE')
        or os.environ.get('CALIBRATION_DEFAULT_SCENE')
        or os.environ.get('SCENE')
        or data.get('current_scene')
        or default_scene
    )
    scene_name = str(scene_name)
    if scene_name not in scenes:
        scene_name = next(iter(scenes.keys()))
    scene_cfg = scenes.get(scene_name, {}) if isinstance(scenes.get(scene_name), dict) else {}
    return scene_name, scene_cfg, scenes


def load_scene_context(default_home=None, scene_name=None):
    default_home = default_home or {
        'x': 110.0,
        'y': 0.0,
        'z': 220.0,
        'pitch': -90.0,
        'roll': 0.0,
        'claw': 0.0,
        'time_ms': 1500,
    }
    context = {
        'scene_path': get_scene_config_path(),
        'active_scene_name': scene_name or os.environ.get('CALIBRATION_CURRENT_SCENE') or os.environ.get('CALIBRATION_DEFAULT_SCENE') or os.environ.get('SCENE') or SCENE0_ID,
        'calibration_scene_name': scene_name or os.environ.get('CALIBRATION_CURRENT_SCENE') or os.environ.get('CALIBRATION_DEFAULT_SCENE') or os.environ.get('SCENE') or SCENE0_ID,
        'map_length_m': None,
        'map_width_m': None,
        'calibration_tag_id': None,
        'home_pose': dict(default_home),
    }

    try:
        data = common.get_yaml_data(context['scene_path']) or {}
        active_scene_name, active_scene_cfg, scenes = _resolve_active_scene(data, scene_name=scene_name)

        calibration_scene_name = str(active_scene_cfg.get('use_calibration_scene', active_scene_name))
        if calibration_scene_name not in scenes:
            calibration_scene_name = active_scene_name
        calibration_scene_cfg = scenes.get(calibration_scene_name, {})
        if not isinstance(calibration_scene_cfg, dict):
            calibration_scene_cfg = {}

        home = active_scene_cfg.get('home_pose', {}) if isinstance(active_scene_cfg.get('home_pose'), dict) else {}
        home_pose = dict(default_home)
        if active_scene_name == SCENE0_ID:
            home_pose.update(SCENE0_HOME_POSE)
        for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw'):
            home_pose[key] = _float_from_dict(home, key, home_pose[key])
        home_pose['time_ms'] = _int_from_dict(home, 'time_ms', home_pose.get('time_ms', 1500))

        length_m = active_scene_cfg.get('length_m')
        width_m = active_scene_cfg.get('width_m')
        white_area = active_scene_cfg.get('white_area', {}) if isinstance(active_scene_cfg.get('white_area'), dict) else {}
        fallback_white_area = (
            calibration_scene_cfg.get('white_area', {})
            if isinstance(calibration_scene_cfg.get('white_area'), dict)
            else {}
        )
        if length_m is None:
            length_m = white_area.get(
                'length_m',
                calibration_scene_cfg.get('length_m', fallback_white_area.get('length_m')),
            )
        if width_m is None:
            width_m = white_area.get(
                'width_m',
                calibration_scene_cfg.get('width_m', fallback_white_area.get('width_m')),
            )

        tag_cfg = active_scene_cfg.get('calibration_tag', {}) if isinstance(active_scene_cfg.get('calibration_tag'), dict) else {}
        if not tag_cfg:
            tag_cfg = (
                calibration_scene_cfg.get('calibration_tag', {})
                if isinstance(calibration_scene_cfg.get('calibration_tag'), dict)
                else {}
            )

        context.update({
            'active_scene_name': str(active_scene_name),
            'calibration_scene_name': str(calibration_scene_name),
            'map_length_m': None if length_m is None else float(length_m),
            'map_width_m': None if width_m is None else float(width_m),
            'calibration_tag_id': tag_cfg.get('id') if isinstance(tag_cfg, dict) else None,
            'home_pose': home_pose,
        })
    except Exception:
        pass
    return context


def load_scene_home_pose():
    default_home = {
        'x': 110.0,
        'y': 0.0,
        'z': 220.0,
        'pitch': -90.0,
        'roll': 0.0,
        'claw': 0.0,
        'time_ms': 1500,
    }
    home = load_scene_context(default_home)['home_pose']
    return {key: home[key] for key in default_home}


def get_use_scene_pose(node, default=True):
    parameter_name = 'use_scene_pose'
    try:
        if hasattr(node, 'has_parameter') and not node.has_parameter(parameter_name):
            node.declare_parameter(parameter_name, bool(default))
        value = node.get_parameter(parameter_name).value
        if value is None:
            return bool(default)
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    except Exception:
        return bool(default)


def select_pose(node, program_pose, scene_pose=None, default_pose=None):
    pose = dict(default_pose or {})
    pose.update(program_pose or {})
    if get_use_scene_pose(node, True):
        scene = scene_pose or load_scene_home_pose()
        pose.update({key: scene[key] for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw', 'time_ms') if key in scene})
    return pose
