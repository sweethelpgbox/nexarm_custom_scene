import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_priority_uses_scene_grid_order():
    from app import sorting_coordinator

    scene_cfg = {
        "color_grid": {
            "slots": ["red", "green", "blue", "yellow"],
        },
        "scene3_grid": {
            "waste_slots": ["food_waste", "recyclable_waste", "hazardous_waste", "residual_waste"],
        },
    }

    priority = sorting_coordinator.priority_from_scene_config(scene_cfg)

    assert priority[:4] == ["red", "green", "blue", "yellow"]
    assert priority[4:8] == ["food_waste", "recyclable_waste", "hazardous_waste", "residual_waste"]


def test_priority_uses_scene4_all_frame_and_shelf_slots():
    from app import sorting_coordinator

    scene_cfg = {
        "scene4_grid": {
            "all_slots": ["red", "green", "yellow", "blue"],
            "all_upper_slots": ["residual_waste", "food_waste"],
            "all_lower_slots": ["hazardous_waste", "recyclable_waste"],
        },
    }

    priority = sorting_coordinator.priority_from_scene_config(scene_cfg)

    assert priority[:8] == [
        "red",
        "green",
        "yellow",
        "blue",
        "residual_waste",
        "food_waste",
        "hazardous_waste",
        "recyclable_waste",
    ]


def test_lower_priority_target_waits_when_higher_priority_is_detected(tmp_path, monkeypatch):
    from app import sorting_coordinator

    monkeypatch.setenv("SORTING_COORDINATOR_STATE", str(tmp_path / "sort_state.json"))
    priority = ["red", "food_waste"]

    sorting_coordinator.reset_session("scene_2", priority)
    sorting_coordinator.report_detections(sorting_coordinator.COLOR_GROUP, ["red"], "scene_2", priority)
    sorting_coordinator.report_detections(sorting_coordinator.WASTE_GROUP, ["food_waste"], "scene_2", priority)

    claimed, reason = sorting_coordinator.try_claim(
        sorting_coordinator.WASTE_GROUP,
        "food_waste",
        "scene_2",
        priority,
    )

    assert claimed is False
    assert "red" in reason

    claimed, _ = sorting_coordinator.try_claim(
        sorting_coordinator.COLOR_GROUP,
        "red",
        "scene_2",
        priority,
    )
    assert claimed is True

    sorting_coordinator.release_claim(sorting_coordinator.COLOR_GROUP, "red")
    sorting_coordinator.report_detections(sorting_coordinator.COLOR_GROUP, [], "scene_2", priority)

    claimed, _ = sorting_coordinator.try_claim(
        sorting_coordinator.WASTE_GROUP,
        "food_waste",
        "scene_2",
        priority,
    )
    assert claimed is True
