import sys
import types
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_scene4_rail_defaults_use_steps_and_4_subdivision():
    from app.scene4_runtime import scene4_rail_config

    scene_cfg = {
        "rail": {
            "enabled": True,
            "total_steps": 4200,
            "calibration_abs_position": 4000,
            "place_abs_position": 700,
        }
    }

    rail = scene4_rail_config(scene_cfg)

    assert rail["enabled"] is True
    assert rail["total_steps"] == 4200
    assert rail["calibration_abs_position"] == 4000
    assert rail["place_abs_position"] == 700
    assert rail["subdivision"] == 0x02


def test_scene4_runtime_identifies_active_scene():
    from app.scene4_runtime import active_scene_from_data

    name, cfg = active_scene_from_data(
        {
            "current_scene": "scene_4",
            "scenes": {
                "scene_1": {"name": "Scene 1"},
                "scene_4": {"name": "Scene 4"},
            },
        }
    )

    assert name == "scene_4"
    assert cfg["name"] == "Scene 4"


def test_scene4_calibration_pose_defaults_to_user_requested_pose():
    from app.scene4_runtime import scene4_calibration_pose

    pose = scene4_calibration_pose({})

    assert pose == {
        "x": 145.0,
        "y": 0.0,
        "z": 290.0,
        "pitch": -90.0,
        "roll": 0.0,
        "claw": 0.0,
        "time_ms": 2000,
    }


def test_scene4_calibration_pose_allows_scene_override():
    from app.scene4_runtime import scene4_calibration_pose

    pose = scene4_calibration_pose(
        {
            "calibration_pose": {
                "x": "146",
                "time_ms": "2500",
            }
        }
    )

    assert pose["x"] == 146.0
    assert pose["y"] == 0.0
    assert pose["time_ms"] == 2500


def test_scene4_pick_config_defaults_to_lower_only():
    from app.scene4_runtime import scene4_pick_config

    pick = scene4_pick_config({})

    assert pick["active_zone"] == "lower_board"
    assert pick["lower_board"]["detection"]["min_v"] == 0
    assert pick["lower_board"]["detection"]["max_v"] == 1080
    assert "upper_shelf" not in pick


def test_scene4_absolute_positions_override_fixed_slot_coordinates():
    from app import scene4_runtime

    scene_cfg = {
        "scene4_place": {
            "targets": {
                "red": "frame",
                "green": "upper_shelf",
                "yellow": "lower_shelf",
            },
        },
        "scene4_grid": {
            "color_slots": ["green", "red", "blue", "yellow"],
            "color_upper_slots": ["red", "green"],
            "color_lower_slots": ["blue", "yellow"],
        },
        "scene4_absolute_positions": {
            "frame_slots": [
                [0.11, 0.21, 0.01],
                [0.12, 0.22, 0.02],
                [0.13, 0.23, 0.03],
                [0.14, 0.24, 0.04],
            ],
            "upper_shelf_slots": [
                [0.31, 0.01, 0.33],
                [0.32, -0.02, 0.34],
            ],
            "lower_shelf_slots": [
                [0.21, 0.03, 0.18],
                [0.22, -0.04, 0.19],
            ],
        }
    }

    positions = scene4_runtime.scene4_absolute_positions(scene_cfg)

    assert len(positions["frame_slots"]) == 4
    assert len(positions["upper_shelf_slots"]) == 2
    assert len(positions["lower_shelf_slots"]) == 2
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "red") == [0.12, 0.22, 0.02]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "green") == [0.32, -0.02, 0.34]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "yellow") == [0.22, -0.04, 0.19]


def test_scene4_shelf_defaults_use_fixed_layer_poses_and_slot_rail_positions():
    from app import scene4_runtime

    scene_cfg = {
        "rail": {
            "enabled": True,
            "total_steps": 4200,
            "subdivision": 0x02,
            "calibration_abs_position": 4000,
            "place_abs_position": 700,
        },
        "scene4_place": {
            "targets": {
                "red": "upper_shelf",
                "green": "upper_shelf",
                "yellow": "lower_shelf",
                "blue": "lower_shelf",
            },
        },
    }

    shelf = scene4_runtime.scene4_shelf_config(scene_cfg)
    red = scene4_runtime.scene4_shelf_place(scene_cfg, "red", [0.24, 0.06, 0.32])
    green = scene4_runtime.scene4_shelf_place(scene_cfg, "green", [0.24, -0.06, 0.32])
    yellow = scene4_runtime.scene4_shelf_place(scene_cfg, "yellow", [0.10, 0.06, 0.19])
    blue = scene4_runtime.scene4_shelf_place(scene_cfg, "blue", [0.10, -0.06, 0.19])

    assert shelf["length_m"] == 0.6
    assert shelf["levels"]["upper_shelf"]["approach_pose"]["x"] == 270.0
    assert shelf["levels"]["upper_shelf"]["approach_pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["x"] == 348.0
    assert shelf["levels"]["upper_shelf"]["pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["pitch"] == 0.0
    assert shelf["levels"]["lower_shelf"]["pose"]["x"] == 350.0
    assert shelf["levels"]["lower_shelf"]["pose"]["z"] == 210.0
    assert shelf["levels"]["lower_shelf"]["pose"]["pitch"] == 0.0
    assert red["destination"] == "upper_shelf"
    assert red["rail_position"] == 3900
    assert green["destination"] == "upper_shelf"
    assert green["rail_position"] == 1100
    assert yellow["destination"] == "lower_shelf"
    assert yellow["rail_position"] == 3900
    assert blue["destination"] == "lower_shelf"
    assert blue["rail_position"] == 1100


def test_scene4_shelf_layer_poses_are_fixed_even_if_config_has_old_values():
    from app import scene4_runtime

    scene_cfg = {
        "scene4_shelf": {
            "levels": {
                "upper_shelf": {
                    "approach_pose": {
                        "x": 240,
                        "z": 320,
                    },
                    "pose": {
                        "x": 325,
                        "z": 315,
                        "pitch": -30,
                    },
                },
                "lower_shelf": {
                    "pose": {
                        "x": 310,
                        "z": 170,
                        "pitch": -25,
                    },
                },
            },
        },
    }

    shelf = scene4_runtime.scene4_shelf_config(scene_cfg)

    assert shelf["levels"]["upper_shelf"]["approach_pose"]["x"] == 270.0
    assert shelf["levels"]["upper_shelf"]["approach_pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["x"] == 348.0
    assert shelf["levels"]["upper_shelf"]["pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["pitch"] == 0.0
    assert shelf["levels"]["lower_shelf"]["pose"]["x"] == 350.0
    assert shelf["levels"]["lower_shelf"]["pose"]["z"] == 210.0
    assert shelf["levels"]["lower_shelf"]["pose"]["pitch"] == 0.0


def test_scene4_place_config_preserves_frame_upper_and_lower_destinations():
    from app import scene4_runtime

    scene_cfg = {
        "scene4_place": {
            "default_destination": "frame",
            "targets": {
                "red": "frame",
                "green": "lower_shelf",
                "yellow": "upper_shelf",
                "blue": "frame",
                "residual_waste": "frame",
                "food_waste": "lower_shelf",
            },
        }
    }

    place = scene4_runtime.scene4_place_config(scene_cfg)

    assert place["default_destination"] == "frame"
    assert place["targets"]["red"] == "frame"
    assert place["targets"]["green"] == "lower_shelf"
    assert place["targets"]["yellow"] == "upper_shelf"
    assert place["targets"]["blue"] == "frame"
    assert place["targets"]["residual_waste"] == "frame"
    assert place["targets"]["food_waste"] == "lower_shelf"


def test_scene4_shelf_place_returns_none_for_frame_and_fixed_pose_for_selected_layers():
    from app import scene4_runtime

    scene_cfg = {
        "rail": {
            "enabled": True,
            "total_steps": 4200,
            "subdivision": 0x02,
            "calibration_abs_position": 4000,
            "place_abs_position": 700,
        },
        "scene4_place": {
            "targets": {
                "red": "frame",
                "green": "lower_shelf",
                "yellow": "upper_shelf",
                "recyclable_waste": "upper_shelf",
            },
        },
    }

    red = scene4_runtime.scene4_shelf_place(scene_cfg, "red", [0.285, 0.16, 0.015])
    green = scene4_runtime.scene4_shelf_place(scene_cfg, "green", [0.10, -0.06, 0.19])
    yellow = scene4_runtime.scene4_shelf_place(scene_cfg, "yellow", [0.24, 0.06, 0.315])
    recyclable = scene4_runtime.scene4_shelf_place(scene_cfg, "recyclable_waste", [0.24, -0.06, 0.315])

    assert red is None
    assert green["destination"] == "lower_shelf"
    assert green["rail_position"] == 1100
    assert green["pose"]["x"] == 350.0
    assert green["pose"]["z"] == 210.0
    assert green["pose"]["pitch"] == 0.0
    assert "approach_pose" not in green
    assert yellow["destination"] == "upper_shelf"
    assert yellow["rail_position"] == 3900
    assert yellow["approach_pose"]["x"] == 270.0
    assert yellow["approach_pose"]["y"] == 0.0
    assert yellow["approach_pose"]["z"] == 407.0
    assert yellow["approach_pose"]["pitch"] == 0.0
    assert yellow["pose"]["x"] == 348.0
    assert yellow["pose"]["z"] == 407.0
    assert yellow["pose"]["pitch"] == 0.0
    assert recyclable["destination"] == "upper_shelf"
    assert recyclable["rail_position"] == 1100


def test_scene4_fixed_place_position_snaps_to_destination_and_grid_slot():
    from app import scene4_runtime

    scene_cfg = {
        "scene4_place": {
            "targets": {
                "red": "frame",
                "green": "lower_shelf",
                "yellow": "upper_shelf",
                "residual_waste": "frame",
                "food_waste": "lower_shelf",
            },
        },
        "scene4_grid": {
            "color_slots": ["green", "red", "blue", "yellow"],
            "waste_slots": ["food_waste", "residual_waste", "hazardous_waste", "recyclable_waste"],
        },
    }

    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "red") == [0.285, -0.16, 0.015]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "green") == [0.100, -0.060, 0.190]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "yellow") == [0.240, 0.060, 0.315]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "residual_waste") == [0.285, -0.16, 0.015]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "food_waste") == [0.100, -0.060, 0.190]

    scene_cfg["scene4_grid"]["all_slots"] = [
        "yellow",
        "blue",
        "green",
        "red",
        "recyclable_waste",
        "hazardous_waste",
        "food_waste",
        "residual_waste",
    ]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "red") == [0.115, -0.16, 0.015]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "residual_waste") == [0.285, -0.16, 0.015]


def test_scene4_shelf_slots_are_fixed_positions_not_color_bindings():
    from app import scene4_runtime

    scene_cfg = {
        "scene4_place": {
            "targets": {
                "red": "upper_shelf",
                "green": "upper_shelf",
                "yellow": "lower_shelf",
                "blue": "lower_shelf",
            },
        },
        "scene4_grid": {
            "color_upper_slots": ["green", "red"],
            "color_lower_slots": ["blue", "yellow"],
        },
    }

    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "red") == [0.240, -0.060, 0.315]
    assert scene4_runtime.scene4_shelf_place(scene_cfg, "red", [0.240, -0.060, 0.315])["rail_position"] == 1100
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "green") == [0.240, 0.060, 0.315]
    assert scene4_runtime.scene4_shelf_place(scene_cfg, "green", [0.240, 0.060, 0.315])["rail_position"] == 3900
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "yellow") == [0.100, -0.060, 0.190]
    assert scene4_runtime.scene4_fixed_place_position(scene_cfg, "blue") == [0.100, 0.060, 0.190]


def test_scene4_transfer_pose_defaults_to_fixed_safe_pose_and_can_publish(monkeypatch):
    from app import scene4_runtime

    calls = []
    monkeypatch.setattr(scene4_runtime.time, "sleep", lambda seconds: calls.append(("sleep", seconds)))

    pose = scene4_runtime.scene4_transfer_pose({})
    scene4_runtime.publish_scene4_transfer_pose(lambda *args: calls.append(("arm", args)), claw=88)

    assert pose == {
        "x": 181.0,
        "y": 0.0,
        "z": 275.0,
        "pitch": 0.0,
        "roll": 0.0,
        "time_ms": 2000,
    }
    assert calls == [
        ("arm", (181.0, 0.0, 275.0, 0.0, 0.0, 88, 2000)),
        ("sleep", 2.0),
    ]


def test_scene4_transfer_pose_only_used_for_shelf_placement_paths():
    app_dir = Path(__file__).resolve().parents[1] / "app"
    sorting_text = (app_dir / "object_sorting.py").read_text(encoding="utf-8")
    waste_text = (app_dir / "waste_classification.py").read_text(encoding="utf-8")

    sorting_shelf = sorting_text.split("def _execute_scene4_shelf_place", 1)[1].split("    def init_process", 1)[0]
    waste_shelf = waste_text.split("def _execute_scene4_shelf_place", 1)[1].split("    def send_request", 1)[0]
    assert "publish_scene4_transfer_pose" in sorting_shelf
    assert "publish_scene4_transfer_pose" in waste_shelf
    assert "approach_pose = shelf_place.get('approach_pose')" in sorting_shelf
    assert "approach_pose = shelf_place.get('approach_pose')" in waste_shelf

    sorting_open_index = sorting_shelf.index(
        "self.publish_arm(pose['x'], pose['y'], pose['z'], pose['pitch'], pose['roll'], claw_open, 500)"
    )
    sorting_retract_index = sorting_shelf.index("retract_ms = int(approach_pose.get('time_ms', move_ms))")
    sorting_transfer_index = sorting_shelf.index("publish_scene4_transfer_pose(self.publish_arm, claw_open")
    assert sorting_open_index < sorting_retract_index < sorting_transfer_index

    waste_open_index = waste_shelf.index(
        "self.publish_arm(pose['x'], pose['y'], pose['z'], pose['pitch'], pose['roll'], claw_open, 500)"
    )
    waste_retract_index = waste_shelf.index("retract_ms = int(approach_pose.get('time_ms', move_ms))")
    waste_transfer_index = waste_shelf.index("publish_scene4_transfer_pose(self.publish_arm, claw_open")
    assert waste_open_index < waste_retract_index < waste_transfer_index

    sorting_frame = sorting_text.split("shelf_place = self._resolve_scene4_shelf_place", 1)[1].split(
        "if not scene4_runtime.move_scene4_rail",
        1,
    )[0]
    waste_frame = waste_text.split("if scene4_active:", 1)[1].split(
        "if not scene4_runtime.move_scene4_rail",
        1,
    )[0]
    assert "publish_scene4_transfer_pose" not in sorting_frame
    assert "publish_scene4_transfer_pose" not in waste_frame


def test_waste_classification_enters_scene4_by_returning_rail_to_calibration_first():
    app_dir = Path(__file__).resolve().parents[1] / "app"
    waste_text = (app_dir / "waste_classification.py").read_text(encoding="utf-8")

    assert "def _configure_scene4_observation_pose(self):" in waste_text
    helper = waste_text.split("def _configure_scene4_observation_pose(self):", 1)[1].split("\n    def ", 1)[0]
    assert "scene4_runtime.move_scene4_rail(" in helper
    assert '"calibration"' in helper
    assert "return dict(pose)" in helper

    enter = waste_text.split("def enter_srv_callback", 1)[1].split("\n    def exit_srv_callback", 1)[0]
    assert "self.home_pose = self._configure_scene4_observation_pose()" in enter
    assert enter.index("self._init_parameters()") < enter.index("self.home_pose = self._configure_scene4_observation_pose()")
    assert enter.index("self.home_pose = self._configure_scene4_observation_pose()") < enter.index("self.publish_arm(")


def test_scene4_shelf_8_subdivision_doubles_fixed_rail_positions():
    from app import scene4_runtime

    scene_cfg = {
        "rail": {
            "enabled": True,
            "total_steps": 8400,
            "subdivision": 0x03,
            "calibration_abs_position": 8000,
            "place_abs_position": 1400,
        },
        "scene4_place": {
            "targets": {
                "red": "upper_shelf",
                "green": "upper_shelf",
                "yellow": "lower_shelf",
                "blue": "lower_shelf",
            },
        },
    }

    assert scene4_runtime.scene4_shelf_place(scene_cfg, "red", [0.24, 0.06, 0.32])["rail_position"] == 7800
    assert scene4_runtime.scene4_shelf_place(scene_cfg, "green", [0.24, -0.06, 0.32])["rail_position"] == 2200
    assert scene4_runtime.scene4_shelf_place(scene_cfg, "yellow", [0.10, 0.06, 0.19])["rail_position"] == 7800
    assert scene4_runtime.scene4_shelf_place(scene_cfg, "blue", [0.10, -0.06, 0.19])["rail_position"] == 2200


def test_scene4_place_defaults_send_color_to_frame_and_waste_to_shelves():
    from app import scene4_runtime

    assert scene4_runtime.scene4_target_destination({}, "red", [0.24, 0.06, 0.32]) == "frame"
    assert scene4_runtime.scene4_target_destination({}, "yellow", [0.10, 0.06, 0.19]) == "frame"
    assert scene4_runtime.scene4_target_destination({}, "residual_waste", [0.095, 0.214, 0.02]) == "upper_shelf"
    shelf_place = scene4_runtime.scene4_shelf_place({}, "residual_waste", [0.095, 0.214, 0.02])
    assert shelf_place is not None
    assert shelf_place["destination"] == "upper_shelf"
    assert shelf_place["rail_position"] == 3900


def test_scene4_observation_pose_ignores_disabled_upper_pick_zone():
    from app.scene4_runtime import scene4_observation_pose

    pose = scene4_observation_pose(
        {
            "scene4_pick": {
                "active_zone": "upper_shelf",
                "upper_shelf": {
                    "view_pose": {
                        "x": 112,
                    },
                },
            }
        }
    )

    assert pose["x"] == 145.0
    assert pose["z"] == 290.0
    assert pose["pitch"] == -90.0


def test_scene4_calibration_rail_move_can_skip_forced_reset(monkeypatch):
    from app import scene4_runtime

    calls = []
    monkeypatch.setattr(
        scene4_runtime,
        "load_active_scene",
        lambda _path=None: (
            "scene_4",
            {
                "rail": {
                    "enabled": True,
                    "calibration_abs_position": 4000,
                    "subdivision": 0x02,
                    "reset_wait_sec": 18.0,
                    "speed_steps_per_sec": 1000.0,
                }
            },
        ),
    )

    def fake_call(*_args, **kwargs):
        calls.append(kwargs)
        return True, "ok"

    monkeypatch.setattr(scene4_runtime, "call_stepper_position", fake_call)

    ok = scene4_runtime.move_scene4_rail(
        node=object(),
        client=object(),
        mode="calibration",
        reset_first=False,
    )

    assert ok is True
    assert calls[0]["position"] == 4000
    assert calls[0]["reset_first"] is False


def test_create_stepper_position_client_uses_callback_group_when_provided(monkeypatch):
    from app import scene4_runtime

    created = {}

    class FakeClientType:
        pass

    class FakeNode:
        def create_client(self, srv_type, srv_name, **kwargs):
            created["srv_type"] = srv_type
            created["srv_name"] = srv_name
            created["kwargs"] = kwargs
            return object()

    controller_msgs = types.ModuleType("ros_robot_controller_msgs")
    srv_module = types.ModuleType("ros_robot_controller_msgs.srv")
    srv_module.SetStepperPosition = FakeClientType
    monkeypatch.setitem(sys.modules, "ros_robot_controller_msgs", controller_msgs)
    monkeypatch.setitem(sys.modules, "ros_robot_controller_msgs.srv", srv_module)
    callback_group = object()

    scene4_runtime.create_stepper_position_client(FakeNode(), callback_group=callback_group)

    assert created["srv_type"] is FakeClientType
    assert created["srv_name"] == "/ros_robot_controller/stepper/set_position"
    assert created["kwargs"]["callback_group"] is callback_group


def test_return_scene4_to_calibration_pose_publishes_pose_then_moves_rail(monkeypatch):
    from app import scene4_runtime

    calls = []
    monkeypatch.setattr(
        scene4_runtime,
        "load_active_scene",
        lambda _path=None: (
            "scene_4",
            {
                "calibration_pose": {
                    "x": 145.0,
                    "y": 0.0,
                    "z": 290.0,
                    "pitch": -90.0,
                    "roll": 0.0,
                    "claw": 0.0,
                    "time_ms": 2000,
                }
            },
        ),
    )
    monkeypatch.setattr(
        scene4_runtime,
        "move_scene4_rail",
        lambda *args, **kwargs: calls.append(("rail", kwargs)) or True,
    )
    monkeypatch.setattr(scene4_runtime.time, "sleep", lambda seconds: calls.append(("sleep", seconds)))

    def publish_arm(*pose):
        calls.append(("arm", pose))

    ok = scene4_runtime.return_scene4_to_calibration_pose(
        node=object(),
        client=object(),
        publish_arm=publish_arm,
        scene_path="/tmp/scene.yaml",
    )

    assert ok is True
    assert calls[0] == ("arm", (145.0, 0.0, 290.0, -90.0, 0.0, 0.0, 2000))
    assert calls[1] == ("sleep", 2.0)
    assert calls[2] == ("rail", {"mode": "calibration", "scene_path": "/tmp/scene.yaml", "logger": None, "reset_first": False})
