import importlib.util
import sys
from pathlib import Path

import yaml
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[3]
EXAMPLE_SRC = ROOT / "src" / "example"

sys.path.insert(0, str(EXAMPLE_SRC))


def _load_example_waste_launch():
    launch_path = (
        EXAMPLE_SRC
        / "example"
        / "yolo_detect"
        / "waste_classification.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "example_waste_classification_launch", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scene0_default_map_uses_167mm_by_130mm():
    from example.scene_pose import _resolve_active_scene

    _scene_name, scene_cfg, _scenes = _resolve_active_scene(
        {
            "current_scene": "scene_0",
            "scenes": {
                "scene_1": {"name": "Scene 1"},
            },
        }
    )

    assert scene_cfg["length_m"] == 0.167
    assert scene_cfg["width_m"] == 0.13


def test_scene0_saved_calibration_map_and_tag_center_match_167mm_by_130mm():
    scene_path = ROOT / "src" / "app" / "config" / "calibration_scene.yaml"
    cfg = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    scene0 = cfg["scenes"]["scene_0"]

    assert abs(scene0["length_m"] - 0.167) < 1e-9
    assert abs(scene0["width_m"] - 0.13) < 1e-9
    assert abs(scene0["calibration_tag"]["center_in_map_m"]["x"] - -0.0635) < 1e-9
    assert abs(scene0["calibration_tag"]["center_in_map_m"]["y"] - 0.045) < 1e-9


def test_example_waste_launch_defaults_when_need_compile_is_unset(monkeypatch, tmp_path):
    monkeypatch.delenv("need_compile", raising=False)
    monkeypatch.setenv("MACHINE_TYPE", "Pro")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    launch_module = _load_example_waste_launch()

    actions = launch_module.launch_setup(LaunchContext())

    assert actions


def test_yolo_node_subscribes_to_launch_image_topic_parameter():
    yolo_text = (
        EXAMPLE_SRC
        / "example"
        / "yolo_detect"
        / "yolo_node.py"
    ).read_text(encoding="utf-8")

    assert "self.image_topic = str(self.get_parameter('image_topic').value)" in yolo_text
    assert "self.create_subscription(Image, self.image_topic, self.image_callback, 1)" in yolo_text
    assert "self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)" not in yolo_text
