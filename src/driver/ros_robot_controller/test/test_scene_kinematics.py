import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_scene4_adds_rail_height_to_linkage1_and_base_high():
    from ros_robot_controller.scene_kinematics import scene4_target_and_restore_params

    current = [110.45, 225.0, 36.97, 145.0, 0.0, 130.23, 0.0, 50.0, 70.5]
    scene_cfg = {
        "kinematics": {
            "linkage1_mm": 110.45,
            "linkage1_delta_mm": 72.0,
            "base_high_mm": 70.5,
            "base_high_delta_mm": 72.0,
        }
    }

    target, restore = scene4_target_and_restore_params(current, scene_cfg)

    assert target[0] == 182.45
    assert target[8] == 142.5
    assert restore[0] == 110.45
    assert restore[8] == 70.5


def test_scene4_does_not_cumulatively_add_rail_height_if_params_are_already_applied():
    from ros_robot_controller.scene_kinematics import scene4_target_and_restore_params

    current = [182.45, 225.0, 36.97, 145.0, 0.0, 130.23, 0.0, 50.0, 142.5]
    scene_cfg = {
        "kinematics": {
            "linkage1_mm": 110.45,
            "linkage1_delta_mm": 72.0,
            "base_high_mm": 70.5,
            "base_high_delta_mm": 72.0,
        }
    }

    target, restore = scene4_target_and_restore_params(current, scene_cfg)

    assert target[0] == 182.45
    assert target[8] == 142.5
    assert restore[0] == 110.45
    assert restore[8] == 70.5


def test_scene4_home_pose_defaults_for_low_level_startup():
    from ros_robot_controller.scene_kinematics import scene_home_pose

    pose = scene_home_pose(
        {
            "home_pose": {
                "x": 115.0,
                "y": 0.0,
                "z": 302.0,
                "pitch": -35.0,
                "roll": 0.0,
                "claw": 0.0,
            }
        }
    )

    assert pose == {
        "x": 115.0,
        "y": 0.0,
        "z": 302.0,
        "pitch": -35.0,
        "roll": 0.0,
        "claw": 0.0,
    }


def test_scene4_home_command_matches_low_level_startup_pose():
    from ros_robot_controller.scene_kinematics import scene_home_command

    command = scene_home_command(
        {
            "home_pose": {
                "x": 115.0,
                "y": 0.0,
                "z": 302.0,
                "pitch": -35.0,
                "roll": 0.0,
                "claw": 0.0,
            }
        },
        roll_to_driver_roll=lambda angle: -float(angle),
    )

    assert command == {
        "x": 115.0,
        "y": 0.0,
        "z": 302.0,
        "pitch": -35.0,
        "roll": 0.0,
        "driver_roll": 0.0,
        "claw": 0.0,
        "time_ms": 1500,
    }


def test_scene4_startup_rail_config_requires_scene4_or_slide_rails():
    from ros_robot_controller.scene_kinematics import scene4_startup_rail_config

    scene_cfg = {
        "rail": {
            "enabled": True,
            "subdivision": 7,
            "reset_wait_sec": 18.0,
        }
    }

    assert scene4_startup_rail_config("scene_1", scene_cfg, "None") is None

    active_scene_rail = scene4_startup_rail_config("scene_4", scene_cfg, "None")
    slide_rail = scene4_startup_rail_config("scene_1", scene_cfg, "Slide_Rails")

    assert active_scene_rail["subdivision"] == 7
    assert active_scene_rail["reset_wait_sec"] == 18.0
    assert slide_rail["subdivision"] == 7


def test_scene4_startup_rail_config_ignores_disabled_rail():
    from ros_robot_controller.scene_kinematics import scene4_startup_rail_config

    assert scene4_startup_rail_config(
        "scene_4",
        {"rail": {"enabled": False}},
        "Slide_Rails",
    ) is None
