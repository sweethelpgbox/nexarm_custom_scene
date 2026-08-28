#!/usr/bin/env python3
# coding: utf8
import yaml

from app import scene_play_registry


def load_active_scene_config(scene_config_path, default_scene='scene_1'):
    try:
        with open(scene_config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        cfg = {}

    scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
    if not isinstance(scenes, dict) or not scenes:
        scenes = {default_scene: {}}

    if isinstance(cfg, dict):
        scene_name = str(cfg.get('current_scene', default_scene))
    else:
        scene_name = default_scene
    if scene_name not in scenes:
        scene_name = next(iter(scenes.keys()))
    scene_cfg = scenes.get(scene_name, {})
    scene_cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    return scene_play_registry.merge_play_into_scene(scene_name, scene_cfg)


def _apply_pose_values(pose, values):
    for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw'):
        if key in values:
            pose[key] = float(values[key])
    if 'time_ms' in values:
        pose['time_ms'] = int(float(values['time_ms']))


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


def load_play_home_pose(scene_config_path, defaults, pitch_override=None, default_scene='scene_1', use_scene=True):
    pose = {key: float(value) for key, value in defaults.items()}
    scene_cfg = load_active_scene_config(scene_config_path, default_scene)

    if use_scene:
        home = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
        _apply_pose_values(pose, home)
    if pitch_override is not None:
        pose['pitch'] = float(pitch_override)
    return pose


def select_pose(node, program_pose, scene_pose=None, default_pose=None):
    pose = dict(default_pose or {})
    pose.update(program_pose or {})
    if get_use_scene_pose(node, True):
        scene = scene_pose or default_pose or {}
        pose.update({key: scene[key] for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw', 'time_ms') if key in scene})
    return pose
