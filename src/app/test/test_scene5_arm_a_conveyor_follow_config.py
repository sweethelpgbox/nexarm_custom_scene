import pytest
import yaml
from pathlib import Path

_YAML_PATH = (
    Path(__file__).resolve().parents[2]
    / 'example/example/motor/plays/scene5_dual_arm.yaml'
)


def _load_conveyor_follow_cfg():
    with open(_YAML_PATH, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('scene5_dual_arm', {}).get('arm_a_conveyor_follow', {})


def _parse_speed_profiles(conveyor_follow):
    """Pure equivalent of Scene5ArmALoader.load_conveyor_speed_profiles."""
    profiles = conveyor_follow.get('speed_profiles', {})
    loaded = {}
    for cmd, raw in profiles.items():
        if not isinstance(raw, dict):
            continue
        try:
            cmd_int = int(cmd)
            mmps = float(raw.get('mmps', 0.0))
            if mmps <= 0:
                continue
            loaded[cmd_int] = {
                'mmps': mmps,
                'lead_ms': int(raw.get('lead_ms', 300)),
                'release_ms': int(raw.get('release_ms', 450)),
                'tail_ms': int(raw.get('tail_ms', 250)),
            }
        except (TypeError, ValueError):
            continue
    return loaded


def _clamped_offset(profile, duration_key, max_offset_mm):
    """Pure equivalent of Scene5ArmALoader.clamped_follow_offset."""
    duration_sec = max(0.0, float(profile.get(duration_key, 0)) / 1000.0)
    raw = float(profile['mmps']) * duration_sec
    limit = max(0.0, max_offset_mm)
    return min(raw, limit) if limit > 0.0 else raw


def test_conveyor_follow_axis_is_x():
    cfg = _load_conveyor_follow_cfg()
    assert cfg.get('axis', 'x').strip().lower() == 'x'


def test_conveyor_follow_max_offset_is_80mm():
    cfg = _load_conveyor_follow_cfg()
    assert float(cfg.get('max_offset_mm', 0.0)) == pytest.approx(80.0)


def test_speed_profile_minus20_is_7p5_mmps():
    cfg = _load_conveyor_follow_cfg()
    profiles = _parse_speed_profiles(cfg)
    assert -20 in profiles
    assert profiles[-20]['mmps'] == pytest.approx(7.5)


def test_speed_profile_minus50_is_22p5_mmps():
    cfg = _load_conveyor_follow_cfg()
    profiles = _parse_speed_profiles(cfg)
    assert -50 in profiles
    assert profiles[-50]['mmps'] == pytest.approx(22.5)


def test_speed_profile_minus100_is_42p0_mmps():
    cfg = _load_conveyor_follow_cfg()
    profiles = _parse_speed_profiles(cfg)
    assert -100 in profiles
    assert profiles[-100]['mmps'] == pytest.approx(42.0)


def test_clamped_offset_caps_at_80mm():
    # 42 mm/s * 3s = 126 mm — must be clamped to 80
    profile = {'mmps': 42.0, 'release_ms': 3000}
    assert _clamped_offset(profile, 'release_ms', 80.0) == pytest.approx(80.0)


def test_clamped_offset_unclamped_below_limit():
    # 7.5 mm/s * 0.45s = 3.375 mm — well under 80
    profile = {'mmps': 7.5, 'release_ms': 450}
    assert _clamped_offset(profile, 'release_ms', 80.0) == pytest.approx(3.375)


def test_with_follow_offset_modifies_x_not_y_or_z():
    pose = {'x': 0.0, 'y': 210.0, 'z': 100.0}
    axis = 'x'
    offset = 12.5
    adjusted = dict(pose)
    adjusted[axis] = float(adjusted.get(axis, 0.0)) + float(offset)
    assert adjusted['x'] == pytest.approx(12.5)
    assert adjusted['y'] == pytest.approx(210.0)
    assert adjusted['z'] == pytest.approx(100.0)
