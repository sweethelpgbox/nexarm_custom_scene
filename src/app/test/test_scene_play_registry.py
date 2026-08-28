import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_scene_registry_reads_scene_and_chassis_from_typerc(tmp_path, monkeypatch):
    from app import scene_play_registry

    typerc = tmp_path / ".typerc"
    typerc.write_text(
        "export CALIBRATION_CURRENT_SCENE=scene_4\n"
        "export CHASSIS_TYPE=Slide_Rails\n",
        encoding="utf-8",
    )
    for key in ("CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", "SCENE", "CHASSIS_TYPE"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("CALIBRATION_TYPERC_PATH", str(typerc))

    assert scene_play_registry.active_scene_from_env() == scene_play_registry.SCENE4_ID
    assert scene_play_registry.chassis_type_from_env() == "Slide_Rails"
    assert scene_play_registry.scene_config_path_for() == scene_play_registry.STEPPER_SCENE_CONFIG


def test_global_place_offset_is_limited_and_applied(tmp_path):
    from app import scene_play_registry

    scene_cfg = tmp_path / "calibration_scene.yaml"
    scene_cfg.write_text(
        "global_place_offset:\n"
        "  x: 0.02\n"
        "  y: -0.004\n",
        encoding="utf-8",
    )

    assert scene_play_registry.global_place_offset(str(scene_cfg)) == {"x": 0.01, "y": -0.004}
    assert scene_play_registry.apply_global_place_offset(
        [0.1, 0.2, 0.03],
        str(scene_cfg),
    ) == [0.11, 0.196, 0.03]


def test_scene1_launches_basic_sorting_nodes():
    from app import scene_play_registry

    entry = scene_play_registry.play_for_scene(scene_play_registry.SCENE1_ID)
    launches = {(item["package"], item["launch"]) for item in entry["launches"]}
    waste_launch = next(
        item for item in entry["launches"]
        if item["package"] == "app" and item["launch"] == "waste_classification.launch.py"
    )

    assert entry["play_id"] == "basic_sorting"
    assert ("app", "object_sorting.launch.py") in launches
    assert ("app", "waste_classification.launch.py") in launches
    assert waste_launch["arguments"]["model_name"] == "best_garbage_11"


def test_scene1_waste_place_pose_configured():
    from app import scene_play_registry

    cfg = scene_play_registry.load_play_config(scene_play_registry.SCENE1_ID)

    assert cfg["place_targets"]["residual_waste"] == [0.0, 0.2, 0.15]
    assert cfg["place_targets"]["hazardous_waste"] == [0.12, 0.26, 0.14]
    assert cfg["place_targets"]["recyclable_waste"] == [0.19, 0.31, 0.15]
    for target in ("residual_waste", "hazardous_waste", "recyclable_waste"):
        assert scene_play_registry.resolve_place_pitch(cfg, target, 80.0) == 80.0
        assert scene_play_registry.resolve_place_roll(cfg, target) == 0.0
