import time

import yaml

from app import scene_play_registry


SCENE4_ID = "scene_4"
APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
STEPPER_SET_POSITION_SERVICE = "/ros_robot_controller/stepper/set_position"

DEFAULT_RAIL_CONFIG = {
    "enabled": False,
    "total_steps": 4200,
    "subdivision": 0x02,
    "calibration_abs_position": 4000,
    "place_abs_position": 700,
    "reset_wait_sec": 18.0,
    "speed_steps_per_sec": 1000.0,
}
DEFAULT_CALIBRATION_POSE = {
    "x": 145.0,
    "y": 0.0,
    "z": 290.0,
    "pitch": -90.0,
    "roll": 0.0,
    "claw": 0.0,
    "time_ms": 2000,
}
DEFAULT_TRANSFER_POSE = {
    "x": 181.0,
    "y": 0.0,
    "z": 275.0,
    "pitch": 0.0,
    "roll": 0.0,
    "time_ms": 2000,
}
SCENE4_PICK_ZONE_LOWER = "lower_board"
SCENE4_PLACE_FRAME = "frame"
SCENE4_SHELF_UPPER = "upper_shelf"
SCENE4_SHELF_LOWER = "lower_shelf"
SCENE4_SHELF_LEVELS = (SCENE4_SHELF_UPPER, SCENE4_SHELF_LOWER)
SCENE4_VALID_DESTINATIONS = (SCENE4_PLACE_FRAME, SCENE4_SHELF_UPPER, SCENE4_SHELF_LOWER)
SCENE4_SHELF_LEFT = "left"
SCENE4_SHELF_RIGHT = "right"
DEFAULT_SHELF_LENGTH_M = 0.600
DEFAULT_SHELF_WIDTH_M = 0.263
SCENE4_SUBDIVISION_CODES = {
    4: 0x02,
    8: 0x03,
    16: 0x07,
}
SCENE4_SUBDIVISION_FACTORS = {
    0x02: 4,
    0x03: 8,
    0x07: 16,
}
DEFAULT_RAIL_BY_SUBDIVISION = {
    4: {
        "total_steps": 4200,
        "calibration_abs_position": 4000,
        "place_abs_position": 700,
    },
    8: {
        "total_steps": 8400,
        "calibration_abs_position": 8000,
        "place_abs_position": 1400,
    },
}
DEFAULT_SHELF_RAIL_SLOTS = {
    4: {
        SCENE4_SHELF_LEFT: 3900,
        SCENE4_SHELF_RIGHT: 1100,
        "slot_1": 3900,
        "slot_2": 2967,
        "slot_3": 2033,
        "slot_4": 1100,
    },
    8: {
        SCENE4_SHELF_LEFT: 7800,
        SCENE4_SHELF_RIGHT: 2200,
        "slot_1": 7800,
        "slot_2": 5933,
        "slot_3": 4067,
        "slot_4": 2200,
    },
}
SCENE4_TARGET_SLOTS = {
    "red": (SCENE4_SHELF_UPPER, SCENE4_SHELF_LEFT),
    "green": (SCENE4_SHELF_UPPER, SCENE4_SHELF_RIGHT),
    "yellow": (SCENE4_SHELF_LOWER, SCENE4_SHELF_LEFT),
    "blue": (SCENE4_SHELF_LOWER, SCENE4_SHELF_RIGHT),
    "residual_waste": (SCENE4_SHELF_UPPER, SCENE4_SHELF_LEFT),
    "food_waste": (SCENE4_SHELF_UPPER, SCENE4_SHELF_RIGHT),
    "hazardous_waste": (SCENE4_SHELF_LOWER, SCENE4_SHELF_LEFT),
    "recyclable_waste": (SCENE4_SHELF_LOWER, SCENE4_SHELF_RIGHT),
}
SCENE4_TARGET_SIDES = {
    key: side for key, (_level, side) in SCENE4_TARGET_SLOTS.items()
}
SCENE4_COLOR_KEYS = ("red", "green", "yellow", "blue")
SCENE4_WASTE_KEYS = ("residual_waste", "food_waste", "hazardous_waste", "recyclable_waste")
SCENE4_ALL_KEYS = SCENE4_COLOR_KEYS + SCENE4_WASTE_KEYS
SCENE4_FRAME_SLOT_COUNT = 4
SCENE4_SHELF_SLOT_COUNT = 4
SCENE4_WASTE_TARGET_MAP = {
    "residual_waste": "red",
    "food_waste": "green",
    "hazardous_waste": "yellow",
    "recyclable_waste": "blue",
}
SCENE4_FRAME_SLOT_TARGETS = (
    [0.255, 0.12, 0.08],
    [0.255, -0.12, 0.08],
    [0.13, 0.03, 0.08],
    [0.13, -0.13, 0.08],
)
SCENE4_SHELF_SLOT_FIELDS = {
    SCENE4_SHELF_UPPER: {
        "color": "color_upper_slots",
        "waste": "waste_upper_slots",
    },
    SCENE4_SHELF_LOWER: {
        "color": "color_lower_slots",
        "waste": "waste_lower_slots",
    },
}
DEFAULT_SHELF_CONFIG = {
    "length_m": DEFAULT_SHELF_LENGTH_M,
    "width_m": DEFAULT_SHELF_WIDTH_M,
    "level_match_tolerance_m": 0.06,
    "rail_slots": DEFAULT_SHELF_RAIL_SLOTS,
    "levels": {
        SCENE4_SHELF_UPPER: {
            "target_z_m": 0.315,
            "approach_pose": {
                "x": 270.0,
                "y": 0.0,
                "z": 407.0,
                "pitch": 0.0,
                "roll": 0.0,
                "claw": 0.0,
                "time_ms": 1500,
            },
            "pose": {
                "x": 348.0,
                "y": 0.0,
                "z": 407.0,
                "pitch": 0.0,
                "roll": 0.0,
                "claw": 0.0,
                "time_ms": 1500,
            },
        },
        SCENE4_SHELF_LOWER: {
            "target_z_m": 0.190,
            "pose": {
                "x": 350.0,
                "y": 0.0,
                "z": 210.0,
                "pitch": 0.0,
                "roll": 0.0,
                "claw": 0.0,
                "time_ms": 2000,
            },
        },
    },
}
DEFAULT_PLACE_CONFIG = {
    "default_destination": SCENE4_PLACE_FRAME,
    "targets": {
        "red": SCENE4_PLACE_FRAME,
        "green": SCENE4_PLACE_FRAME,
        "yellow": SCENE4_PLACE_FRAME,
        "blue": SCENE4_PLACE_FRAME,
    },
}
DEFAULT_LOWER_VIEW_POSE = {
    "x": 145.0,
    "y": 0.0,
    "z": 290.0,
    "pitch": -90.0,
    "roll": 0.0,
    "claw": 0.0,
    "time_ms": 2000,
}
DEFAULT_ABSOLUTE_POSITIONS = {
    "frame_slots": [list(pos) for pos in SCENE4_FRAME_SLOT_TARGETS],
    "upper_shelf_slots": [
        [0.240, 0.060, 0.315],   # 上层左1
        [0.240, 0.020, 0.315],   # 上层左2
        [0.240, -0.020, 0.315],  # 上层右1
        [0.240, -0.060, 0.315],  # 上层右2
    ],
    "lower_shelf_slots": [
        [0.100, 0.060, 0.190],   # 下层左1
        [0.100, 0.020, 0.190],   # 下层左2
        [0.100, -0.020, 0.190],  # 下层右1
        [0.100, -0.060, 0.190],  # 下层右2
    ],
}
DEFAULT_PICK_CONFIG = {
    "active_zone": SCENE4_PICK_ZONE_LOWER,
    SCENE4_PICK_ZONE_LOWER: {
        "view_pose": dict(DEFAULT_LOWER_VIEW_POSE),
        "use_plane_calibration": True,
        "detection": {
            "min_v": 0,
            "max_v": 1080,
        },
    },
}


def scene_config_path():
    scene_id = scene_play_registry.active_scene_from_env()
    if scene_id:
        return scene_play_registry.scene_config_path_for(scene_id)
    if scene_play_registry.chassis_type_from_env() == "Slide_Rails":
        return scene_play_registry.scene_config_path_for(SCENE4_ID)
    return scene_play_registry.scene_config_path_for()


def active_scene_from_data(data, default_scene="scene_1"):
    cfg = data if isinstance(data, dict) else {}
    scenes = cfg.get("scenes")
    if not isinstance(scenes, dict) or not scenes:
        return default_scene, {}

    scene_name = str(cfg.get("current_scene", default_scene))
    if scene_name not in scenes:
        scene_name = next(iter(scenes.keys()))

    scene_cfg = scenes.get(scene_name, {})
    if not isinstance(scene_cfg, dict):
        scene_cfg = {}
    return scene_name, scene_play_registry.merge_play_into_scene(scene_name, scene_cfg)


def load_active_scene(path=None):
    cfg_path = path or scene_config_path()
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}
    return active_scene_from_data(data)


def scene4_rail_config(scene_cfg):
    raw = scene_cfg.get("rail", {}) if isinstance(scene_cfg, dict) else {}
    raw = raw if isinstance(raw, dict) else {}
    subdivision = scene4_subdivision_code(raw.get("subdivision", DEFAULT_RAIL_CONFIG["subdivision"]))
    rail = dict(DEFAULT_RAIL_CONFIG)
    rail.update(scene4_rail_defaults_for_subdivision(subdivision))
    rail.update(raw)
    rail["subdivision"] = scene4_subdivision_code(rail.get("subdivision", subdivision))

    factor_defaults = scene4_rail_defaults_for_subdivision(rail["subdivision"])
    rail["enabled"] = bool(rail.get("enabled", False))
    rail["total_steps"] = _normalized_int(rail.get("total_steps"), factor_defaults["total_steps"], 0)
    rail["calibration_abs_position"] = _normalized_int(
        rail.get("calibration_abs_position"),
        factor_defaults["calibration_abs_position"],
        0,
        rail["total_steps"],
    )
    rail["place_abs_position"] = _normalized_int(
        rail.get("place_abs_position"),
        factor_defaults["place_abs_position"],
        0,
        rail["total_steps"],
    )
    for key in ("reset_wait_sec", "speed_steps_per_sec"):
        rail[key] = float(rail.get(key, DEFAULT_RAIL_CONFIG[key]))
    return rail


def scene4_calibration_pose(scene_cfg):
    pose = dict(DEFAULT_CALIBRATION_POSE)
    raw = scene_cfg.get("calibration_pose", {}) if isinstance(scene_cfg, dict) else {}
    if not isinstance(raw, dict):
        raw = {}

    for key in ("x", "y", "z", "pitch", "roll", "claw"):
        try:
            pose[key] = float(raw.get(key, pose[key]))
        except Exception:
            pose[key] = DEFAULT_CALIBRATION_POSE[key]
    try:
        pose["time_ms"] = int(raw.get("time_ms", pose["time_ms"]))
    except Exception:
        pose["time_ms"] = DEFAULT_CALIBRATION_POSE["time_ms"]
    return pose


def scene4_transfer_pose(scene_cfg):
    return dict(DEFAULT_TRANSFER_POSE)


def publish_scene4_transfer_pose(publish_arm, claw, scene_cfg=None):
    pose = scene4_transfer_pose(scene_cfg or {})
    publish_arm(
        pose["x"],
        pose["y"],
        pose["z"],
        pose["pitch"],
        pose["roll"],
        claw,
        int(pose["time_ms"]),
    )
    time.sleep(max(0.0, float(pose["time_ms"]) / 1000.0))
    return pose


def _merge_missing_dict(target, default):
    if not isinstance(target, dict):
        target = {}
    for key, value in default.items():
        if isinstance(value, dict):
            target[key] = _merge_missing_dict(target.get(key), value)
        elif key not in target:
            target[key] = value
    return target


def _normalized_int(value, default, minimum=None, maximum=None):
    try:
        parsed = int(value)
    except Exception:
        parsed = int(default)
    if minimum is not None:
        parsed = max(int(minimum), parsed)
    if maximum is not None:
        parsed = min(int(maximum), parsed)
    return parsed


def scene4_subdivision_code(value, default=0x02):
    try:
        parsed = int(value)
    except Exception:
        return int(default)
    if parsed in SCENE4_SUBDIVISION_CODES:
        return SCENE4_SUBDIVISION_CODES[parsed]
    if parsed in SCENE4_SUBDIVISION_FACTORS:
        return parsed
    return int(default)


def scene4_subdivision_factor(value):
    code = scene4_subdivision_code(value)
    return int(SCENE4_SUBDIVISION_FACTORS.get(code, 4))


def scene4_rail_defaults_for_subdivision(subdivision):
    factor = scene4_subdivision_factor(subdivision)
    defaults = dict(DEFAULT_RAIL_BY_SUBDIVISION.get(factor, DEFAULT_RAIL_BY_SUBDIVISION[4]))
    defaults["subdivision"] = scene4_subdivision_code(subdivision)
    return defaults


def _normalized_float(value, default, minimum=None, maximum=None):
    try:
        parsed = float(value)
    except Exception:
        parsed = float(default)
    if minimum is not None:
        parsed = max(float(minimum), parsed)
    if maximum is not None:
        parsed = min(float(maximum), parsed)
    return parsed


def _normalized_pose(value, default):
    raw = value if isinstance(value, dict) else {}
    pose = {}
    for key in ("x", "y", "z", "pitch", "roll", "claw"):
        pose[key] = _normalized_float(raw.get(key), default[key])
    pose["time_ms"] = _normalized_int(raw.get("time_ms"), default.get("time_ms", 2000), 1)
    return pose


def scene4_pick_config(scene_cfg):
    raw = scene_cfg.get("scene4_pick", {}) if isinstance(scene_cfg, dict) else {}
    pick = _merge_missing_dict(raw, yaml.safe_load(yaml.safe_dump(DEFAULT_PICK_CONFIG)))
    pick["active_zone"] = SCENE4_PICK_ZONE_LOWER

    lower = pick.setdefault(SCENE4_PICK_ZONE_LOWER, {})
    lower["view_pose"] = _normalized_pose(
        lower.get("view_pose"),
        DEFAULT_PICK_CONFIG[SCENE4_PICK_ZONE_LOWER]["view_pose"],
    )
    lower["use_plane_calibration"] = bool(lower.get("use_plane_calibration", True))
    lower_default = DEFAULT_PICK_CONFIG[SCENE4_PICK_ZONE_LOWER]["detection"]
    lower_detection = lower.setdefault("detection", {})
    min_v = _normalized_int(lower_detection.get("min_v"), lower_default["min_v"], 0)
    max_v = _normalized_int(lower_detection.get("max_v"), lower_default["max_v"], 0)
    if min_v > max_v:
        min_v, max_v = max_v, min_v
    lower["detection"] = {
        "min_v": min_v,
        "max_v": max_v,
    }
    return {
        "active_zone": SCENE4_PICK_ZONE_LOWER,
        SCENE4_PICK_ZONE_LOWER: lower,
    }


def scene4_active_pick_zone(scene_cfg):
    return scene4_pick_config(scene_cfg).get("active_zone", SCENE4_PICK_ZONE_LOWER)


def scene4_observation_pose(scene_cfg):
    """场景4下层拣取的观察姿态 —— 优先读 play 配置里的 view_pose，兜底用 calibration_pose"""
    if isinstance(scene_cfg, dict):
        pick = scene_cfg.get("scene4_pick", {})
        if isinstance(pick, dict):
            lower = pick.get(SCENE4_PICK_ZONE_LOWER, {})
            if isinstance(lower, dict) and isinstance(lower.get("view_pose"), dict):
                return _normalized_pose(lower["view_pose"], DEFAULT_CALIBRATION_POSE)
    return scene4_calibration_pose(scene_cfg)


def scene4_shelf_config(scene_cfg):
    cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    raw = cfg.get("scene4_shelf", {}) if isinstance(cfg.get("scene4_shelf"), dict) else {}
    rail = scene4_rail_config(cfg)

    shelf = {
        "length_m": _normalized_float(
            raw.get("length_m"),
            DEFAULT_SHELF_CONFIG["length_m"],
            0.001,
        ),
        "width_m": _normalized_float(
            raw.get("width_m"),
            DEFAULT_SHELF_CONFIG["width_m"],
            0.001,
        ),
        "level_match_tolerance_m": _normalized_float(
            raw.get("level_match_tolerance_m"),
            DEFAULT_SHELF_CONFIG["level_match_tolerance_m"],
            0.0,
        ),
        "rail_slots": {},
        "levels": {},
    }
    raw_slots = raw.get("rail_slots", {}) if isinstance(raw.get("rail_slots"), dict) else {}
    for factor, defaults in DEFAULT_SHELF_RAIL_SLOTS.items():
        raw_factor = raw_slots.get(str(factor), raw_slots.get(factor, {}))
        raw_factor = raw_factor if isinstance(raw_factor, dict) else {}
        shelf["rail_slots"][str(factor)] = {}
        for slot_key, default_value in defaults.items():
            shelf["rail_slots"][str(factor)][slot_key] = _normalized_int(raw_factor.get(slot_key), default_value, 0)

    for level_key in SCENE4_SHELF_LEVELS:
        level_default = DEFAULT_SHELF_CONFIG["levels"][level_key]
        level = {
            "target_z_m": _normalized_float(
                level_default.get("target_z_m"),
                level_default["target_z_m"],
                0.0,
            ),
            "pose": _normalized_pose(level_default.get("pose"), level_default["pose"]),
        }
        default_approach = level_default.get("approach_pose")
        if default_approach is not None:
            level["approach_pose"] = _normalized_pose(default_approach, default_approach)
        shelf["levels"][level_key] = level

    shelf["upper_z_m"] = shelf["levels"][SCENE4_SHELF_UPPER]["target_z_m"]
    shelf["lower_z_m"] = shelf["levels"][SCENE4_SHELF_LOWER]["target_z_m"]
    shelf["active_subdivision"] = scene4_subdivision_factor(rail["subdivision"])
    return shelf


def scene4_place_config(scene_cfg):
    raw = scene_cfg.get("scene4_place", {}) if isinstance(scene_cfg, dict) and isinstance(scene_cfg.get("scene4_place"), dict) else {}
    valid = set(SCENE4_VALID_DESTINATIONS)
    default_destination = str(raw.get("default_destination", DEFAULT_PLACE_CONFIG["default_destination"]))
    if default_destination not in valid:
        default_destination = DEFAULT_PLACE_CONFIG["default_destination"]

    targets = dict(DEFAULT_PLACE_CONFIG["targets"])
    raw_targets = raw.get("targets", {}) if isinstance(raw.get("targets"), dict) else {}
    for key, value in raw_targets.items():
        if str(key) not in SCENE4_COLOR_KEYS:
            continue
        destination = str(value)
        if destination in valid:
            targets[str(key)] = destination
    return {
        "default_destination": default_destination,
        "targets": targets,
    }


def scene4_target_destination(scene_cfg, target_key, target_position=None):
    target_key = str(target_key)
    if target_key not in SCENE4_COLOR_KEYS:
        return None
    place = scene4_place_config(scene_cfg)
    destination = place["targets"].get(target_key, place["default_destination"])
    if destination in SCENE4_VALID_DESTINATIONS:
        return destination

    slot = SCENE4_TARGET_SLOTS.get(target_key)
    if slot is not None:
        return slot[0]

    if target_position is not None:
        shelf = scene4_shelf_config(scene_cfg)
        try:
            z_m = float(target_position[2])
            best_level = min(
                SCENE4_SHELF_LEVELS,
                key=lambda level: abs(z_m - shelf["levels"][level]["target_z_m"]),
            )
            if abs(z_m - shelf["levels"][best_level]["target_z_m"]) <= shelf["level_match_tolerance_m"]:
                return best_level
        except Exception:
            pass
    return SCENE4_PLACE_FRAME


def scene4_target_side(target_key):
    target_key = str(target_key)
    return SCENE4_TARGET_SIDES.get(target_key) or SCENE4_TARGET_SIDES.get(SCENE4_WASTE_TARGET_MAP.get(target_key, ""))


def _scene4_target_group(target_key):
    return "waste" if str(target_key) in SCENE4_WASTE_KEYS else "color"


def _normalized_frame_slots(slots, keys):
    keys = list(keys)
    slot_count = SCENE4_FRAME_SLOT_COUNT
    raw_slots = slots if isinstance(slots, (list, tuple)) else []
    clean = []
    used = set()
    for raw in raw_slots[:slot_count]:
        key = str(raw) if raw is not None else ""
        if key in keys and key not in used:
            clean.append(key)
            used.add(key)
        else:
            clean.append("")
    while len(clean) < slot_count:
        clean.append("")
    for key in keys:
        if key in used:
            continue
        try:
            empty_index = clean.index("")
            clean[empty_index] = key
        except ValueError:
            clean.append(key)
        used.add(key)
    while len(clean) < slot_count:
        clean.append("")
    return clean[:slot_count]


def _normalized_position(value, default):
    raw = value if isinstance(value, (list, tuple)) else []
    pos = []
    for idx, fallback in enumerate(default):
        try:
            pos.append(round(float(raw[idx]), 3))
        except Exception:
            pos.append(round(float(fallback), 3))
    return pos[:3]


def _expanded_legacy_shelf_list(raw_slots, middle_value):
    if SCENE4_SHELF_SLOT_COUNT == 4 and len(raw_slots) == 2:
        return [raw_slots[0], middle_value, middle_value, raw_slots[1]]
    return raw_slots


def _normalized_position_slots(value, defaults, expand_legacy_shelf=False):
    raw_slots = value if isinstance(value, (list, tuple)) else []
    if expand_legacy_shelf:
        raw_slots = _expanded_legacy_shelf_list(raw_slots, None)
    slots = []
    for index, default in enumerate(defaults):
        raw = raw_slots[index] if index < len(raw_slots) else default
        slots.append(_normalized_position(raw, default))
    return slots


def scene4_absolute_positions(scene_cfg):
    raw = scene_cfg.get("scene4_absolute_positions", {}) if isinstance(scene_cfg, dict) else {}
    raw = raw if isinstance(raw, dict) else {}
    return {
        "frame_slots": _normalized_position_slots(raw.get("frame_slots"), DEFAULT_ABSOLUTE_POSITIONS["frame_slots"]),
        "upper_shelf_slots": _normalized_position_slots(
            raw.get("upper_shelf_slots"),
            DEFAULT_ABSOLUTE_POSITIONS["upper_shelf_slots"],
            expand_legacy_shelf=True,
        ),
        "lower_shelf_slots": _normalized_position_slots(
            raw.get("lower_shelf_slots"),
            DEFAULT_ABSOLUTE_POSITIONS["lower_shelf_slots"],
            expand_legacy_shelf=True,
        ),
    }


def scene4_frame_slot_target(index, scene_cfg=None):
    positions = scene4_absolute_positions(scene_cfg or {})
    index = max(0, min(len(SCENE4_FRAME_SLOT_TARGETS) - 1, int(index)))
    return list(positions["frame_slots"][index])


def scene4_shelf_slot_target(scene_cfg, destination, slot_index):
    positions = scene4_absolute_positions(scene_cfg or {})
    field = "upper_shelf_slots" if destination == SCENE4_SHELF_UPPER else "lower_shelf_slots"
    slot_index = max(0, min(len(positions[field]) - 1, int(slot_index)))
    return list(positions[field][slot_index])


def scene4_frame_slot_index(scene_cfg, target_key):
    target_key = str(target_key)
    grid = scene_cfg.get("scene4_grid", {}) if isinstance(scene_cfg, dict) else {}
    if isinstance(grid, dict) and isinstance(grid.get("all_slots"), (list, tuple)):
        all_slots = _normalized_frame_slots(grid.get("all_slots"), SCENE4_ALL_KEYS)
        if target_key in all_slots:
            return all_slots.index(target_key)
    if target_key in SCENE4_WASTE_KEYS:
        keys = list(SCENE4_WASTE_KEYS)
        slot_field = "waste_slots"
    else:
        keys = list(SCENE4_COLOR_KEYS)
        slot_field = "color_slots"
    slots = _normalized_frame_slots(grid.get(slot_field), keys) if isinstance(grid, dict) else keys
    return slots.index(target_key) if target_key in slots else 0


def _default_shelf_slots(keys, destination):
    slots = [""] * SCENE4_SHELF_SLOT_COUNT
    for key in keys:
        slot = SCENE4_TARGET_SLOTS.get(key)
        if slot is None or slot[0] != destination:
            continue
        index = SCENE4_SHELF_SLOT_COUNT - 1 if slot[1] == SCENE4_SHELF_RIGHT else 0
        slots[index] = key
    return slots


def _normalized_shelf_slots(slots, keys, destination):
    keys = list(keys)
    raw_slots = slots if isinstance(slots, (list, tuple)) else _default_shelf_slots(keys, destination)
    raw_slots = _expanded_legacy_shelf_list(raw_slots, "")
    clean = []
    used = set()
    for raw in raw_slots[:SCENE4_SHELF_SLOT_COUNT]:
        key = str(raw) if raw is not None else ""
        if key in keys and key not in used:
            clean.append(key)
            used.add(key)
        else:
            clean.append("")
    while len(clean) < SCENE4_SHELF_SLOT_COUNT:
        clean.append("")
    return clean[:SCENE4_SHELF_SLOT_COUNT]


def scene4_shelf_slot_index(scene_cfg, target_key, destination):
    destination = destination if destination in SCENE4_SHELF_LEVELS else SCENE4_SHELF_UPPER
    target_key = str(target_key)
    group = _scene4_target_group(target_key)
    keys = list(SCENE4_WASTE_KEYS if group == "waste" else SCENE4_COLOR_KEYS)
    grid = scene_cfg.get("scene4_grid", {}) if isinstance(scene_cfg, dict) else {}
    all_field = "all_upper_slots" if destination == SCENE4_SHELF_UPPER else "all_lower_slots"
    if isinstance(grid, dict) and isinstance(grid.get(all_field), (list, tuple)):
        all_slots = _normalized_shelf_slots(grid.get(all_field), SCENE4_ALL_KEYS, destination)
        if target_key in all_slots:
            return all_slots.index(target_key)
    field = SCENE4_SHELF_SLOT_FIELDS[destination][group]
    slots = _normalized_shelf_slots(grid.get(field), keys, destination) if isinstance(grid, dict) else _default_shelf_slots(keys, destination)
    return slots.index(target_key) if target_key in slots else None


def scene4_shelf_slot_side(scene_cfg, target_key, destination):
    slot_index = scene4_shelf_slot_index(scene_cfg, target_key, destination)
    if slot_index is None:
        return scene4_target_side(target_key)
    return SCENE4_SHELF_RIGHT if int(slot_index) >= 2 else SCENE4_SHELF_LEFT


def scene4_shelf_fixed_position(scene_cfg, target_key, destination):
    destination = destination if destination in SCENE4_SHELF_LEVELS else SCENE4_SHELF_UPPER
    slot_index = scene4_shelf_slot_index(scene_cfg, target_key, destination)
    if slot_index is None:
        side = scene4_target_side(target_key) or SCENE4_SHELF_LEFT
        slot_index = SCENE4_SHELF_SLOT_COUNT - 1 if side == SCENE4_SHELF_RIGHT else 0
    return scene4_shelf_slot_target(scene_cfg, destination, slot_index)


def scene4_fixed_place_position(scene_cfg, target_key):
    destination = scene4_target_destination(scene_cfg, target_key)
    if destination in SCENE4_SHELF_LEVELS:
        return scene4_shelf_fixed_position(scene_cfg, target_key, destination)
    if destination == SCENE4_PLACE_FRAME:
        return scene4_frame_slot_target(scene4_frame_slot_index(scene_cfg, target_key), scene_cfg)
    return None


def scene4_shelf_rail_position(scene_cfg, target_key, target_position=None, destination=None):
    if destination is None:
        destination = scene4_target_destination(scene_cfg, target_key, target_position)
    rail = scene4_rail_config(scene_cfg)
    factor = scene4_subdivision_factor(rail["subdivision"])
    shelf = scene4_shelf_config(scene_cfg)
    slots = shelf["rail_slots"].get(str(factor), shelf["rail_slots"]["4"])
    slot_index = scene4_shelf_slot_index(scene_cfg, target_key, destination)
    if slot_index is not None:
        slot_key = f"slot_{int(slot_index) + 1}"
        if slot_key in slots:
            return int(slots[slot_key])
    side = scene4_shelf_slot_side(scene_cfg, target_key, destination)
    if side is not None:
        return int(slots[side])

    try:
        x_m = float(target_position[0])
    except Exception:
        x_m = 0.0
    ratio = max(0.0, min(1.0, x_m / max(float(shelf["length_m"]), 1e-6)))
    start = int(rail["calibration_abs_position"])
    end = int(rail["place_abs_position"])
    return int(round(start + (end - start) * ratio))


def scene4_shelf_place(scene_cfg, target_key, target_position):
    destination = scene4_target_destination(scene_cfg, target_key, target_position)
    shelf = scene4_shelf_config(scene_cfg)
    if destination not in shelf["levels"]:
        return None
    level = shelf["levels"][destination]
    result = {
        "destination": destination,
        "rail_position": scene4_shelf_rail_position(scene_cfg, target_key, target_position, destination),
        "pose": dict(level["pose"]),
        "target_position": [float(target_position[0]), float(target_position[1]), float(target_position[2])],
    }
    if "approach_pose" in level:
        result["approach_pose"] = dict(level["approach_pose"])
    return result


def create_stepper_position_client(node, callback_group=None):
    from ros_robot_controller_msgs.srv import SetStepperPosition

    kwargs = {}
    if callback_group is not None:
        kwargs["callback_group"] = callback_group
    return node.create_client(SetStepperPosition, STEPPER_SET_POSITION_SERVICE, **kwargs)


def call_stepper_position(
    node,
    client,
    position,
    reset_first=False,
    subdivision=0x07,
    reset_wait_sec=18.0,
    speed_steps_per_sec=1000.0,
    timeout_sec=30.0,
):
    from ros_robot_controller_msgs.srv import SetStepperPosition

    if client is None:
        return False, "stepper client is not initialized"
    if not client.wait_for_service(timeout_sec=1.0):
        return False, f"{STEPPER_SET_POSITION_SERVICE} not available"

    req = SetStepperPosition.Request()
    req.position = int(position)
    req.absolute = True
    req.reset_first = bool(reset_first)
    req.set_subdivision = True
    req.subdivision = int(subdivision) & 0xFF
    req.wait_for_motion = True
    req.reset_wait_sec = float(reset_wait_sec)
    req.speed_steps_per_sec = float(speed_steps_per_sec)

    future = client.call_async(req)
    deadline = time.time() + max(0.1, float(timeout_sec))
    while time.time() < deadline:
        if future.done():
            try:
                res = future.result()
            except Exception as exc:
                return False, f"stepper service error: {exc}"
            if res is None:
                return False, "stepper service returned no response"
            return bool(res.success), str(res.message)
        time.sleep(0.02)
    return False, "stepper service timeout"


def move_scene4_rail_to_position(
    node,
    client,
    position,
    scene_path=None,
    logger=None,
    reset_first=False,
):
    scene_name, scene_cfg = load_active_scene(scene_path)
    if scene_name != SCENE4_ID:
        return True

    rail = scene4_rail_config(scene_cfg)
    if not rail["enabled"]:
        return True

    position = int(max(0, min(int(position), rail["total_steps"])))
    timeout_sec = rail["reset_wait_sec"] + abs(position) / max(1.0, rail["speed_steps_per_sec"]) + 5.0
    ok, message = call_stepper_position(
        node,
        client,
        position=position,
        reset_first=reset_first,
        subdivision=rail["subdivision"],
        reset_wait_sec=rail["reset_wait_sec"],
        speed_steps_per_sec=rail["speed_steps_per_sec"],
        timeout_sec=timeout_sec,
    )
    if logger is not None:
        if ok:
            logger.info(f"scene_4 rail moved to {position}: {message}")
        else:
            logger.warn(f"scene_4 rail move failed: {message}")
    return ok


def move_scene4_rail(node, client, mode, scene_path=None, logger=None, reset_first=None):
    scene_name, scene_cfg = load_active_scene(scene_path)
    if scene_name != SCENE4_ID:
        return True

    rail = scene4_rail_config(scene_cfg)
    if not rail["enabled"]:
        return True

    if mode == "calibration":
        position = rail["calibration_abs_position"]
        if reset_first is None:
            reset_first = True
    elif mode == "place":
        position = rail["place_abs_position"]
        if reset_first is None:
            reset_first = False
    else:
        return False

    return move_scene4_rail_to_position(
        node,
        client,
        position,
        scene_path=scene_path,
        logger=logger,
        reset_first=reset_first,
    )


def return_scene4_to_calibration_pose(node, client, publish_arm, scene_path=None, logger=None):
    scene_name, scene_cfg = load_active_scene(scene_path)
    if scene_name != SCENE4_ID:
        return None

    pose = scene4_observation_pose(scene_cfg)
    move_ms = int(pose.get("time_ms", 2000))
    publish_arm(
        pose["x"],
        pose["y"],
        pose["z"],
        pose["pitch"],
        pose["roll"],
        pose["claw"],
        move_ms,
    )
    time.sleep(max(0.0, move_ms / 1000.0) + 0.5)
    if not move_scene4_rail(
        node,
        client,
        mode="calibration",
        scene_path=scene_path,
        logger=logger,
        reset_first=False,
    ):
        return False
    return True
