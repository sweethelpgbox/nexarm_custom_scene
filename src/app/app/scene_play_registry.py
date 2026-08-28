#!/usr/bin/env python3
# coding: utf-8

import os
import copy
import yaml


SCENE0_ID = "scene_0"
SCENE1_ID = "scene_1"
SCENE2_ID = "scene_2"
SCENE3_ID = "scene_3"
SCENE4_ID = "scene_4"
SCENE5_ID = "scene_5"
SCENE6_ID = "scene_6"

APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
APP_PLAY_CONFIG_DIR = "/home/ubuntu/ros2_ws/src/app/config/plays"
EXAMPLE_MOTOR_PLAY_CONFIG_DIR = "/home/ubuntu/ros2_ws/src/example/example/motor/plays"
DEFAULT_TYPERC_PATH = "/home/ubuntu/ros2_ws/.typerc"
PLACE_OFFSET_LIMIT_M = 0.010
DEFAULT_GLOBAL_PLACE_OFFSET = {"x": 0.0, "y": 0.0, "z": 0.0}

SCENE_PLAY_REGISTRY = {
    SCENE0_ID: {
        "play_id": "idle",
        "package": None,
        "launch": None,
        "play_config_package": "app",
        "play_config": "scene0_idle.yaml",
    },
    SCENE1_ID: {
        "play_id": "basic_sorting",
        "package": "app",
        "launch": "object_sorting.launch.py",
        "launches": [
            {"package": "app", "launch": "object_sorting.launch.py"},
            {
                "package": "app",
                "launch": "waste_classification.launch.py",
                "arguments": {"model_name": "best_garbage_11"},
            },
        ],
        "play_config_package": "app",
        "play_config": "scene1_calibration.yaml",
    },
    SCENE2_ID: {
        "play_id": "color_tag_sorting",
        "package": "app",
        "launch": "object_sorting.launch.py",
        "launches": [
            {"package": "app", "launch": "object_sorting.launch.py"},
            {"package": "app", "launch": "waste_classification.launch.py"},
        ],
        "play_config_package": "app",
        "play_config": "scene2_color_tag_sorting.yaml",
    },
    SCENE3_ID: {
        "play_id": "waste_classification",
        "package": "app",
        "launch": "waste_classification.launch.py",
        "launches": [
            {"package": "app", "launch": "object_sorting.launch.py"},
            {"package": "app", "launch": "waste_classification.launch.py"},
        ],
        "play_config_package": "app",
        "play_config": "scene3_waste_classification.yaml",
    },
    SCENE4_ID: {
        "play_id": "slide_rail_sorting",
        "package": "app",
        "launch": "waste_classification_scene4.launch.py",
        "launches": [
            {"package": "app", "launch": "object_sorting.launch.py"},
            {"package": "app", "launch": "waste_classification_scene4.launch.py"},
        ],
        "play_config_package": "app",
        "play_config": "scene4_slide_rail_sorting.yaml",
    },
    SCENE5_ID: {
        "play_id": "dual_arm_conveyor",
        "package": "example",
        "launch": "scene5_dual_arm.launch.py",
        "launches": [
            {"package": "example", "launch": "scene5_dual_arm.launch.py"},
        ],
        "play_config_package": "example",
        "play_config": "scene5_dual_arm.yaml",
    },
    SCENE6_ID: {
        "play_id": "custom_object_sorting",
        "package": "app",
        "launch": "custom_object_sorting.launch.py",
        "launches": [
            {"package": "app", "launch": "custom_object_sorting.launch.py"},
        ],
        "play_config_package": "app",
        "play_config": "scene6_custom_object_sorting.yaml",
    },
}

PLAY_CONFIG_KEYS = {
    SCENE0_ID: ("place_policy", "place_targets"),
    SCENE1_ID: ("place_policy", "place_targets", "place_pitch", "place_roll"),
    SCENE2_ID: ("place_policy", "place_targets", "color_grid"),
    SCENE3_ID: ("place_policy", "place_targets", "place_pitch", "place_roll", "scene3_grid"),
    SCENE4_ID: (
        "place_policy",
        "place_targets",
        "rail",
        "scene4_pick",
        "scene4_place",
        "scene4_absolute_positions",
        "scene4_grid",
        "scene4_shelf",
        "kinematics",
    ),
    SCENE5_ID: ("place_policy", "place_targets", "scene5_grid", "scene5_dual_arm"),
    SCENE6_ID: ("place_policy", "place_targets"),
}


def load_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _clamp_float(value, default=0.0, minimum=None, maximum=None):
    try:
        parsed = float(value)
    except Exception:
        parsed = float(default)
    if minimum is not None:
        parsed = max(float(minimum), parsed)
    if maximum is not None:
        parsed = min(float(maximum), parsed)
    return parsed


def normalize_global_place_offset(value):
    raw = value if isinstance(value, dict) else {}
    limit = PLACE_OFFSET_LIMIT_M
    return {
        "x": _clamp_float(raw.get("x", raw.get("offset_x", 0.0)), 0.0, -limit, limit),
        "y": _clamp_float(raw.get("y", raw.get("offset_y", 0.0)), 0.0, -limit, limit),
        "z": _clamp_float(raw.get("z", raw.get("offset_z", 0.0)), 0.0, -limit, limit),
    }


def global_place_offset(path=None):
    data = load_yaml(path or scene_config_path_for())
    if not isinstance(data, dict):
        return dict(DEFAULT_GLOBAL_PLACE_OFFSET)
    raw = data.get("global_place_offset")
    if raw is None:
        raw = data.get("place_offset")
    return normalize_global_place_offset(raw)


def apply_global_place_offset(position, path=None, offset=None):
    if position is None:
        return None
    try:
        result = [float(position[0]), float(position[1]), float(position[2])]
    except Exception:
        return None
    off = normalize_global_place_offset(offset) if offset is not None else global_place_offset(path)
    result[0] = round(result[0] + off["x"], 6)
    result[1] = round(result[1] + off["y"], 6)
    result[2] = round(result[2] + off.get("z", 0.0), 6)
    return result


def resolve_place_pitch(scene_cfg, target_key, default_pitch):
    cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    target_key = str(target_key)
    for field in ("place_pitch", "place_pitches"):
        raw = cfg.get(field)
        if isinstance(raw, dict) and target_key in raw:
            return _clamp_float(raw.get(target_key), default_pitch, -180.0, 180.0)
    targets = cfg.get("place_targets")
    if isinstance(targets, dict):
        target = targets.get(target_key)
        if isinstance(target, dict) and "pitch" in target:
            return _clamp_float(target.get("pitch"), default_pitch, -180.0, 180.0)
    return float(default_pitch)


def resolve_place_roll(scene_cfg, target_key, default_roll=None):
    cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    target_key = str(target_key)
    for field in ("place_roll", "place_rolls", "place_yaw", "place_yaws"):
        raw = cfg.get(field)
        if isinstance(raw, dict) and target_key in raw:
            return _clamp_float(raw.get(target_key), 0.0, -180.0, 180.0)
    targets = cfg.get("place_targets")
    if isinstance(targets, dict):
        target = targets.get(target_key)
        if isinstance(target, dict):
            if "roll" in target:
                return _clamp_float(target.get("roll"), 0.0, -180.0, 180.0)
            if "yaw" in target:
                return _clamp_float(target.get("yaw"), 0.0, -180.0, 180.0)
    return None if default_roll is None else float(default_roll)


def save_yaml(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data or {}, f, sort_keys=False, allow_unicode=True)


def deep_merge(base, overlay):
    result = copy.deepcopy(base) if isinstance(base, dict) else {}
    if not isinstance(overlay, dict):
        return result
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def active_scene_from_env():
    scene_id = (
        os.environ.get("CALIBRATION_CURRENT_SCENE")
        or os.environ.get("CALIBRATION_DEFAULT_SCENE")
        or os.environ.get("SCENE")
    )
    if scene_id:
        return scene_id
    return _active_scene_from_typerc()


def _clean_typerc_export_value(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def _active_scene_from_typerc(path=None):
    values = _typerc_export_values(
        ("CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", "SCENE"),
        path=path,
    )
    for key in ("CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", "SCENE"):
        if values.get(key):
            return values[key]
    return None


def _typerc_export_values(keys, path=None):
    keys = tuple(keys)
    path = path or os.environ.get("CALIBRATION_TYPERC_PATH", DEFAULT_TYPERC_PATH)
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.read().splitlines()
    except Exception:
        return {}

    values = {}
    for line in lines:
        line = line.strip()
        if not line.startswith("export ") or "=" not in line:
            continue
        key, value = line[len("export "):].split("=", 1)
        key = key.strip()
        if key in keys:
            values[key] = _clean_typerc_export_value(value)
    return values


def chassis_type_from_env():
    return os.environ.get("CHASSIS_TYPE", "") or _typerc_export_values(("CHASSIS_TYPE",)).get("CHASSIS_TYPE", "")


def scene_config_path_for(scene_id=None):
    scene_id = scene_id or active_scene_from_env()
    if scene_id == SCENE4_ID:
        return STEPPER_SCENE_CONFIG
    if chassis_type_from_env() == "Slide_Rails" and scene_id != SCENE5_ID:
        return STEPPER_SCENE_CONFIG
    return APP_SCENE_CONFIG


def active_scene_id(default_scene=SCENE1_ID):
    env_scene = active_scene_from_env()
    if env_scene:
        return str(env_scene)
    cfg = load_yaml(scene_config_path_for())
    return str(cfg.get("current_scene", default_scene))


def play_for_scene(scene_id=None):
    scene_id = str(scene_id or active_scene_id())
    return dict(SCENE_PLAY_REGISTRY.get(scene_id, SCENE_PLAY_REGISTRY[SCENE1_ID]))


def play_config_path_for(scene_id=None):
    scene_id = str(scene_id or active_scene_id())
    entry = play_for_scene(scene_id)
    filename = entry.get("play_config")
    if not filename:
        return ""
    if entry.get("play_config_package") == "example":
        return os.path.join(EXAMPLE_MOTOR_PLAY_CONFIG_DIR, filename)
    return os.path.join(APP_PLAY_CONFIG_DIR, filename)


def load_play_config(scene_id=None, path=None):
    scene_id = str(scene_id or active_scene_id())
    data = load_yaml(path or play_config_path_for(scene_id))
    if not data:
        return {}
    data.setdefault("play_id", play_for_scene(scene_id).get("play_id"))
    return data


def extract_play_config(scene_id, scene_cfg):
    scene_id = str(scene_id)
    play = {"play_id": play_for_scene(scene_id).get("play_id")}
    keys = PLAY_CONFIG_KEYS.get(scene_id, ())
    if not isinstance(scene_cfg, dict):
        return play
    for key in keys:
        if key in scene_cfg:
            play[key] = copy.deepcopy(scene_cfg[key])
    return play


def strip_play_config(scene_id, scene_cfg):
    scene_id = str(scene_id)
    clean = copy.deepcopy(scene_cfg) if isinstance(scene_cfg, dict) else {}
    for key in PLAY_CONFIG_KEYS.get(scene_id, ()):
        clean.pop(key, None)
    clean["play_id"] = play_for_scene(scene_id).get("play_id")
    return clean


def merge_play_into_scene(scene_id, scene_cfg, play_cfg=None):
    scene_id = str(scene_id)
    merged = copy.deepcopy(scene_cfg) if isinstance(scene_cfg, dict) else {}
    play = play_cfg if isinstance(play_cfg, dict) else load_play_config(scene_id)
    for key in PLAY_CONFIG_KEYS.get(scene_id, ()):
        if key in play:
            merged[key] = copy.deepcopy(play[key])
    merged["play_id"] = play.get("play_id", play_for_scene(scene_id).get("play_id"))
    return merged


def scene_and_play_from_data(data, default_scene=SCENE1_ID):
    cfg = data if isinstance(data, dict) else {}
    scenes = cfg.get("scenes")
    if not isinstance(scenes, dict) or not scenes:
        return default_scene, {}
    scene_name = str(cfg.get("current_scene", default_scene))
    if scene_name not in scenes:
        scene_name = next(iter(scenes.keys()))
    scene_cfg = scenes.get(scene_name, {})
    return scene_name, merge_play_into_scene(scene_name, scene_cfg)


def load_active_scene_config(path=None, default_scene=SCENE1_ID):
    data = load_yaml(path or scene_config_path_for())
    return scene_and_play_from_data(data, default_scene)
