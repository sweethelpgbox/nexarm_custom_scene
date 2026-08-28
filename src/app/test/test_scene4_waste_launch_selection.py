from pathlib import Path


APP_SRC = Path(__file__).resolve().parents[1]


def test_scene4_waste_launch_uses_yolo11_model():
    launch_path = APP_SRC / "launch" / "waste_classification_scene4.launch.py"

    assert launch_path.exists()
    text = launch_path.read_text(encoding="utf-8")
    assert "waste_classification.launch.py" in text
    assert "'model_name': 'best_garbage_11'" in text


def test_scene3_waste_launch_default_uses_yolo11_model():
    text = (APP_SRC / "launch" / "waste_classification.launch.py").read_text(encoding="utf-8")

    assert "default='best_garbage_11'" in text
    assert "default='best_garbage_26'" not in text


def test_start_app_selects_scene4_waste_launch_for_scene4_only():
    text = (APP_SRC / "launch" / "start_app.launch.py").read_text(encoding="utf-8")

    assert "scene_play_registry.active_scene_id" in text
    assert "scene_play_registry.play_for_scene" in text
    assert "waste_classification_scene4.launch.py" not in text


def test_start_app_passes_registry_launch_arguments():
    text = (APP_SRC / "launch" / "start_app.launch.py").read_text(encoding="utf-8")

    assert "launch_entry.get('arguments', {})" in text
    assert "launch_arguments.update(extra_arguments)" in text


def test_scene1_waste_uses_direct_place_branch():
    text = (APP_SRC / "app" / "waste_classification.py").read_text(encoding="utf-8")

    assert "def scene1_direct_place" in text
    assert "scene_name == scene_play_registry.SCENE1_ID" in text
    assert "self.scene1_direct_place(place_pos, place_pitch, yaw, target)" in text


def test_scene1_waste_pick_holds_yaw_after_grab():
    text = (APP_SRC / "app" / "waste_classification.py").read_text(encoding="utf-8")

    assert "def scene1_pick_hold_yaw" in text
    assert "pick_and_place.pick_without_back" in text
    assert "finish = self.scene1_pick_hold_yaw(position, yaw, target)" in text
    assert "self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], roll_deg, -30.0, move_ms)" in text


def test_grasp_yaw_does_not_add_base_yaw_to_image_angle():
    text = (APP_SRC / "app" / "utils" / "calculate_grasp_yaw.py").read_text(encoding="utf-8")

    assert "yaw1 = yaw + angle1" not in text
    assert "yaw1 = angle1" in text
