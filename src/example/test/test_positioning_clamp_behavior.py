import importlib.util
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
EXAMPLE_SRC = ROOT / "src" / "example"

sys.path.insert(0, str(EXAMPLE_SRC))


def _load_positioning_clamp():
    path = EXAMPLE_SRC / "example" / "opencv" / "include" / "positioning_clamp.py"
    spec = importlib.util.spec_from_file_location("positioning_clamp_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_positioning_clamp_uses_shared_pick_flow(monkeypatch):
    module = _load_positioning_clamp()
    calls = []

    def fake_pick(position, pitch, yaw, gripper_angle, gripper_depth, arm_pub):
        calls.append((position, pitch, yaw, gripper_angle, gripper_depth, arm_pub))
        return True

    monkeypatch.setattr(module.pick_and_place, "pick", fake_pick)
    node = module.PositioningClamp.__new__(module.PositioningClamp)
    node.arm_pub = object()
    node.publish_arm = lambda *args: calls.append(("publish_arm", args))

    class Logger:
        def info(self, *_args, **_kwargs):
            pass

    node.get_logger = lambda: Logger()

    node.start_sorting(np.array([0.12, -0.02, 0.015], dtype=np.float64), 15.0)

    assert calls[0] == ([0.12, -0.02, 0.015], 80, 15.0, 540, 0.02, node.arm_pub)
    assert calls[1][0] == "publish_arm"


def test_color_sorting_launch_does_not_auto_enable_all_targets_by_default():
    launch_path = EXAMPLE_SRC / "example" / "opencv" / "color_sorting_node.launch.py"
    text = launch_path.read_text(encoding="utf-8")

    assert "broadcast = LaunchConfiguration('broadcast', default='true')" in text
