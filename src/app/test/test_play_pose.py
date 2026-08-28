import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_play_home_pose_can_force_pitch_without_losing_scene_position(tmp_path):
    from app.play_pose import load_play_home_pose

    scene_path = tmp_path / "calibration_scene.yaml"
    scene_path.write_text(
        "\n".join(
            [
                "current_scene: scene_0",
                "scenes:",
                "  scene_0:",
                "    home_pose:",
                "      x: 200.0",
                "      y: 10.0",
                "      z: 210.0",
                "      pitch: -90.0",
                "      roll: -2.0",
                "      claw: 0.0",
            ]
        ),
        encoding="utf-8",
    )

    pose = load_play_home_pose(
        str(scene_path),
        {"x": 105.0, "y": 0.0, "z": 200.0, "pitch": 0.0, "roll": 0.0, "claw": -60.0},
        pitch_override=0.0,
    )

    assert pose == {
        "x": 200.0,
        "y": 10.0,
        "z": 210.0,
        "pitch": 0.0,
        "roll": -2.0,
        "claw": 0.0,
    }
