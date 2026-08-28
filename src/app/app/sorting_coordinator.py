#!/usr/bin/env python3
# coding: utf-8

import copy
import fcntl
import json
import os
import time


COLOR_GROUP = "color"
WASTE_GROUP = "waste"
COLOR_KEYS = ("yellow", "red", "green", "blue")
WASTE_KEYS = ("residual_waste", "food_waste", "hazardous_waste", "recyclable_waste")
DEFAULT_PRIORITY = COLOR_KEYS + WASTE_KEYS
DETECTION_TTL_SEC = 1.2
ACTIVE_LEASE_SEC = 90.0


def state_path():
    return os.environ.get("SORTING_COORDINATOR_STATE", "/tmp/nexarm_sorting_coordinator.json")


def _empty_state():
    return {
        "session": {},
        "detections": {},
        "active": None,
    }


def _load_state_unlocked(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f) or {}
        if not isinstance(data, dict):
            return _empty_state()
    except Exception:
        return _empty_state()
    data.setdefault("session", {})
    data.setdefault("detections", {})
    data.setdefault("active", None)
    return data


def _save_state_unlocked(path, state):
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, sort_keys=True)
    os.replace(tmp_path, path)


def _with_state(fn):
    path = state_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    lock_path = f"{path}.lock"
    with open(lock_path, "a+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        state = _load_state_unlocked(path)
        result = fn(state)
        _save_state_unlocked(path, state)
        return result


def _unique_valid(keys):
    valid = set(DEFAULT_PRIORITY)
    clean = []
    for key in keys or ():
        key = str(key)
        if key in valid and key not in clean:
            clean.append(key)
    for key in DEFAULT_PRIORITY:
        if key not in clean:
            clean.append(key)
    return clean


def _unique_targets(keys):
    valid = set(DEFAULT_PRIORITY)
    clean = []
    for key in keys or ():
        key = str(key)
        if key in valid and key not in clean:
            clean.append(key)
    return clean


def _slots_from_grid(grid, field, allowed):
    if not isinstance(grid, dict):
        return []
    slots = []
    for key in grid.get(field, []):
        key = str(key)
        if key in allowed and key not in slots:
            slots.append(key)
    return slots


def priority_from_scene_config(scene_cfg):
    scene_cfg = scene_cfg if isinstance(scene_cfg, dict) else {}
    explicit = scene_cfg.get("combined_sort_priority") or scene_cfg.get("sort_priority")
    if isinstance(explicit, (list, tuple)):
        return _unique_valid(explicit)

    color_slots = []
    waste_slots = []
    color_slots.extend(_slots_from_grid(scene_cfg.get("color_grid"), "slots", COLOR_KEYS))
    scene3_grid = scene_cfg.get("scene3_grid")
    color_slots.extend(_slots_from_grid(scene3_grid, "color_slots", COLOR_KEYS))
    waste_slots.extend(_slots_from_grid(scene3_grid, "waste_slots", WASTE_KEYS))
    scene4_grid = scene_cfg.get("scene4_grid")
    scene4_all_slots = _slots_from_grid(scene4_grid, "all_slots", DEFAULT_PRIORITY)
    scene4_all_slots.extend(_slots_from_grid(scene4_grid, "all_upper_slots", DEFAULT_PRIORITY))
    scene4_all_slots.extend(_slots_from_grid(scene4_grid, "all_lower_slots", DEFAULT_PRIORITY))
    if scene4_all_slots:
        return _unique_valid(scene4_all_slots)
    color_slots.extend(_slots_from_grid(scene4_grid, "color_slots", COLOR_KEYS))
    waste_slots.extend(_slots_from_grid(scene4_grid, "waste_slots", WASTE_KEYS))
    scene5_grid = scene_cfg.get("scene5_grid")
    color_slots.extend(_slots_from_grid(scene5_grid, "color_slots", COLOR_KEYS))
    waste_slots.extend(_slots_from_grid(scene5_grid, "waste_slots", WASTE_KEYS))
    return _unique_valid(color_slots + waste_slots)


def group_for_target(target_key):
    if target_key in COLOR_KEYS:
        return COLOR_GROUP
    if target_key in WASTE_KEYS:
        return WASTE_GROUP
    return None


def sort_keys(keys, priority=None):
    priority = list(priority or DEFAULT_PRIORITY)
    order = {key: idx for idx, key in enumerate(priority)}
    return sorted(keys, key=lambda key: (order.get(key, len(order)), str(key)))


def sort_items(items, key_fn, priority=None):
    priority = list(priority or DEFAULT_PRIORITY)
    order = {key: idx for idx, key in enumerate(priority)}
    return sorted(items, key=lambda item: (order.get(key_fn(item), len(order)), str(key_fn(item))))


def reset_session(scene_id=None, priority=None):
    priority = _unique_valid(priority or DEFAULT_PRIORITY)

    def update(state):
        state["session"] = {
            "scene_id": str(scene_id or ""),
            "priority": list(priority),
            "started_at": time.time(),
        }
        state["detections"] = {}
        state["active"] = None
        return True

    return _with_state(update)


def start_session(scene_id=None, priority=None):
    return reset_session(scene_id, priority)


def report_detections(group, targets, scene_id=None, priority=None):
    now = time.time()
    priority = _unique_valid(priority or DEFAULT_PRIORITY)
    targets = sort_keys(_unique_targets(targets), priority)
    targets = [key for key in targets if group_for_target(key) == group]

    def update(state):
        session = state.setdefault("session", {})
        if priority:
            session["priority"] = list(priority)
        if scene_id is not None:
            session["scene_id"] = str(scene_id)
        state.setdefault("detections", {})[str(group)] = {
            "targets": targets,
            "ts": now,
        }
        _cleanup_unlocked(state, now)
        return True

    return _with_state(update)


def _cleanup_unlocked(state, now):
    detections = state.setdefault("detections", {})
    for group in list(detections.keys()):
        try:
            ts = float(detections[group].get("ts", 0.0))
        except Exception:
            ts = 0.0
        if now - ts > DETECTION_TTL_SEC:
            detections.pop(group, None)
    active = state.get("active")
    if isinstance(active, dict):
        try:
            lease_until = float(active.get("lease_until", 0.0))
        except Exception:
            lease_until = 0.0
        if now > lease_until:
            state["active"] = None


def _current_priority(state, fallback=None):
    session = state.get("session", {}) if isinstance(state, dict) else {}
    priority = session.get("priority") if isinstance(session, dict) else None
    return _unique_valid(priority or fallback or DEFAULT_PRIORITY)


def _detected_targets_unlocked(state, now, extra_target=None):
    _cleanup_unlocked(state, now)
    detected = []
    for info in state.get("detections", {}).values():
        if not isinstance(info, dict):
            continue
        detected.extend(info.get("targets", []))
    if extra_target:
        detected.append(extra_target)
    return _unique_targets(detected)


def selected_target(state, now=None, extra_target=None, priority=None):
    now = time.time() if now is None else now
    priority = _current_priority(state, priority)
    detected = _detected_targets_unlocked(state, now, extra_target)
    for key in priority:
        if key in detected:
            return key
    return None


def try_claim(group, target_key, scene_id=None, priority=None, lease_sec=ACTIVE_LEASE_SEC):
    now = time.time()
    group = str(group)
    target_key = str(target_key)

    def update(state):
        session = state.setdefault("session", {})
        if scene_id is not None:
            session["scene_id"] = str(scene_id)
        if priority:
            session["priority"] = _unique_valid(priority)
        _cleanup_unlocked(state, now)
        active = state.get("active")
        if isinstance(active, dict):
            return False, f"busy:{active.get('group')}:{active.get('target')}"

        selected = selected_target(state, now, target_key, priority)
        if selected is not None and selected != target_key:
            return False, f"wait:{selected}"

        state["active"] = {
            "group": group,
            "target": target_key,
            "ts": now,
            "lease_until": now + float(lease_sec),
        }
        return True, "claimed"

    return _with_state(update)


def release_claim(group=None, target_key=None):
    group = None if group is None else str(group)
    target_key = None if target_key is None else str(target_key)

    def update(state):
        active = state.get("active")
        if not isinstance(active, dict):
            return False
        if group is not None and active.get("group") != group:
            return False
        if target_key is not None and active.get("target") != target_key:
            return False
        state["active"] = None
        return True

    return _with_state(update)


def snapshot():
    return copy.deepcopy(_load_state_unlocked(state_path()))
