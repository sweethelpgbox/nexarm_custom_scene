KINEMATICS_PARAM_COUNT = 9
LINKAGE1_INDEX = 0
BASE_HIGH_INDEX = 8
SCENE4_ID = "scene_4"
DEFAULT_HOME_POSE = {
    "x": 115.0,
    "y": 0.0,
    "z": 302.0,
    "pitch": -35.0,
    "roll": 0.0,
    "claw": 0.0,
}
DEFAULT_SCENE4_RAIL_CONFIG = {
    "enabled": False,
    "total_steps": 4200,
    "subdivision": 0x02,
    "calibration_abs_position": 4000,
    "place_abs_position": 700,
    "reset_wait_sec": 18.0,
    "speed_steps_per_sec": 1000.0,
}


def _float_cfg(cfg, key, default_value):
    try:
        return float(cfg.get(key, default_value))
    except Exception:
        return float(default_value)


def _base_plus_delta(current_value, cfg, base_key, delta_key, default_delta):
    if base_key in cfg:
        base_value = _float_cfg(cfg, base_key, current_value)
        restore_value = base_value
    else:
        base_value = float(current_value)
        restore_value = float(current_value)
    delta_value = _float_cfg(cfg, delta_key, default_delta)
    return base_value + delta_value, restore_value


def scene4_target_and_restore_params(current_params, scene_cfg):
    if current_params is None or len(current_params) < KINEMATICS_PARAM_COUNT:
        raise ValueError("expected 9 kinematics parameters")

    target = [float(v) for v in current_params[:KINEMATICS_PARAM_COUNT]]
    restore = [float(v) for v in current_params[:KINEMATICS_PARAM_COUNT]]
    cfg = scene_cfg.get("kinematics", {}) if isinstance(scene_cfg.get("kinematics"), dict) else {}

    rail_height = _float_cfg(cfg, "rail_height_mm", 72.0)
    target[LINKAGE1_INDEX], restore[LINKAGE1_INDEX] = _base_plus_delta(
        current_params[LINKAGE1_INDEX],
        cfg,
        "linkage1_mm",
        "linkage1_delta_mm",
        rail_height,
    )
    target[BASE_HIGH_INDEX], restore[BASE_HIGH_INDEX] = _base_plus_delta(
        current_params[BASE_HIGH_INDEX],
        cfg,
        "base_high_mm",
        "base_high_delta_mm",
        rail_height,
    )
    return target, restore


def scene_home_pose(scene_cfg):
    home_cfg = scene_cfg.get("home_pose", {}) if isinstance(scene_cfg.get("home_pose"), dict) else {}
    home = dict(DEFAULT_HOME_POSE)
    for key in home:
        try:
            home[key] = float(home_cfg.get(key, home[key]))
        except Exception:
            pass
    return home


def scene_home_command(scene_cfg, roll_to_driver_roll=None):
    cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    home = scene_home_pose(cfg)
    if roll_to_driver_roll is None:
        roll_to_driver_roll = float

    return {
        "x": home["x"],
        "y": home["y"],
        "z": home["z"],
        "pitch": home["pitch"],
        "roll": home["roll"],
        "driver_roll": float(roll_to_driver_roll(home["roll"])),
        "claw": home["claw"],
        "time_ms": int(_float_cfg(cfg, "home_time_ms", 1500)),
    }


def _int_cfg(cfg, key, default_value):
    try:
        return int(cfg.get(key, default_value))
    except Exception:
        return int(default_value)


def scene4_rail_config(scene_cfg):
    raw = scene_cfg.get("rail", {}) if isinstance(scene_cfg, dict) else {}
    rail = dict(DEFAULT_SCENE4_RAIL_CONFIG)
    if isinstance(raw, dict):
        rail.update(raw)

    rail["enabled"] = bool(rail.get("enabled", False))
    for key in ("total_steps", "subdivision", "calibration_abs_position", "place_abs_position"):
        rail[key] = _int_cfg(rail, key, DEFAULT_SCENE4_RAIL_CONFIG[key])
    for key in ("reset_wait_sec", "speed_steps_per_sec"):
        rail[key] = _float_cfg(rail, key, DEFAULT_SCENE4_RAIL_CONFIG[key])
    return rail


def scene4_startup_rail_config(active_scene_name, scene_cfg, chassis_type):
    if str(active_scene_name) != SCENE4_ID and str(chassis_type) != "Slide_Rails":
        return None

    rail = scene4_rail_config(scene_cfg)
    if not rail["enabled"]:
        return None
    return rail
