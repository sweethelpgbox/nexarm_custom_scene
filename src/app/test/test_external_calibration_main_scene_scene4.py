import importlib.util
import sys
import types
from pathlib import Path

import yaml


REPO_MAIN_SCENE_PATH = Path(__file__).resolve().parents[3] / "software/calibration/main_scene.py"
MAIN_SCENE_PATH = (
    REPO_MAIN_SCENE_PATH
    if REPO_MAIN_SCENE_PATH.exists()
    else Path("/home/ubuntu/software/calibration/main_scene.py")
)


class _Dummy:
    def __init__(self, *args, **kwargs):
        pass

    def __call__(self, *args, **kwargs):
        return _Dummy()

    def __getattr__(self, _name):
        return _Dummy()

    def __or__(self, _other):
        return 0

    def __ror__(self, _other):
        return 0

    def name(self):
        return "#000000"


class _DummySignal:
    def connect(self, *args, **kwargs):
        pass

    def emit(self, *args, **kwargs):
        pass


class _DummyQt:
    LeftButton = 1
    AlignCenter = 1
    AlignTop = 2
    AlignLeft = 4
    TextWordWrap = 8
    PointingHandCursor = 16
    ArrowCursor = 32
    NoPen = 64
    NoBrush = 128
    KeepAspectRatio = 256
    SmoothTransformation = 512


def _install_stubs(monkeypatch):
    pyqt5 = types.ModuleType("PyQt5")
    qtcore = types.ModuleType("PyQt5.QtCore")
    qtcore.Qt = _DummyQt
    qtcore.pyqtSignal = lambda *args, **kwargs: _DummySignal()
    qtcore.QPointF = _Dummy
    qtcore.QRectF = _Dummy
    qtcore.QSize = _Dummy

    qtgui = types.ModuleType("PyQt5.QtGui")
    for name in ("QColor", "QPainter", "QPen", "QBrush", "QImage", "QPixmap"):
        setattr(qtgui, name, _Dummy)

    qtwidgets = types.ModuleType("PyQt5.QtWidgets")
    for name in (
        "QApplication",
        "QMainWindow",
        "QPushButton",
        "QComboBox",
        "QCheckBox",
        "QLabel",
        "QDoubleSpinBox",
        "QMessageBox",
        "QWidget",
        "QGridLayout",
        "QHBoxLayout",
        "QVBoxLayout",
        "QGroupBox",
        "QTabWidget",
        "QTabBar",
        "QScrollArea",
        "QSizePolicy",
        "QDialog",
        "QSplitter",
        "QFrame",
    ):
        setattr(qtwidgets, name, _Dummy)

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.init = lambda: None
    rclpy.spin = lambda _node: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Dummy
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = _Dummy()
    rclpy_qos.QoSProfile = _Dummy
    rclpy_qos.ReliabilityPolicy = _Dummy()
    rclpy_qos.HistoryPolicy = _Dummy()
    rclpy_qos.DurabilityPolicy = _Dummy()
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = _Dummy

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    interfaces = types.ModuleType("interfaces")
    interfaces_srv = types.ModuleType("interfaces.srv")
    controller_msgs = types.ModuleType("ros_robot_controller_msgs")
    controller_msgs_msg = types.ModuleType("ros_robot_controller_msgs.msg")
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    for module, names in (
        (std_msgs_msg, ("Int8", "String")),
        (std_srvs_srv, ("SetBool", "Trigger")),
        (interfaces_srv, ("SetString", "SetStringBool", "SetStringList")),
        (controller_msgs_msg, ("ArmCoords",)),
        (sensor_msgs_msg, ("Image", "CompressedImage")),
    ):
        for name in names:
            setattr(module, name, type(name, (), {"Request": _Dummy, "Response": _Dummy}))

    for name, module in (
        ("PyQt5", pyqt5),
        ("PyQt5.QtCore", qtcore),
        ("PyQt5.QtGui", qtgui),
        ("PyQt5.QtWidgets", qtwidgets),
        ("rclpy", rclpy),
        ("rclpy.node", rclpy_node),
        ("rclpy.qos", rclpy_qos),
        ("rclpy.executors", rclpy_executors),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msgs_msg),
        ("std_srvs", std_srvs),
        ("std_srvs.srv", std_srvs_srv),
        ("interfaces", interfaces),
        ("interfaces.srv", interfaces_srv),
        ("ros_robot_controller_msgs", controller_msgs),
        ("ros_robot_controller_msgs.msg", controller_msgs_msg),
        ("sensor_msgs", sensor_msgs),
        ("sensor_msgs.msg", sensor_msgs_msg),
    ):
        monkeypatch.setitem(sys.modules, name, module)


def _load_main_scene(monkeypatch, default_scene="scene_4", scene=None, chassis_type=None):
    _install_stubs(monkeypatch)
    if default_scene is None:
        monkeypatch.delenv("CALIBRATION_DEFAULT_SCENE", raising=False)
    else:
        monkeypatch.setenv("CALIBRATION_DEFAULT_SCENE", default_scene)
    if scene is None:
        monkeypatch.delenv("SCENE", raising=False)
    else:
        monkeypatch.setenv("SCENE", scene)
    if chassis_type is None:
        monkeypatch.delenv("CHASSIS_TYPE", raising=False)
    else:
        monkeypatch.setenv("CHASSIS_TYPE", chassis_type)
    module_name = "external_calibration_main_scene_for_scene4_test"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, MAIN_SCENE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_scene_environment_selects_scene4_without_explicit_default(monkeypatch):
    main_scene = _load_main_scene(
        monkeypatch,
        default_scene=None,
        scene="scene_4",
        chassis_type="None",
    )

    assert main_scene.DEFAULT_CURRENT_SCENE == "scene_4"
    assert main_scene.SCENE_YAML_PATH.endswith("/app/config/calibration_scene.yaml")


def test_scene_environment_overrides_saved_yaml_current_scene(monkeypatch, tmp_path):
    main_scene = _load_main_scene(
        monkeypatch,
        default_scene=None,
        scene="scene_4",
        chassis_type="None",
    )
    scene_path = tmp_path / "calibration_scene.yaml"
    scene_path.write_text(
        yaml.safe_dump(
            {
                "current_scene": "scene_1",
                "scenes": {
                    "scene_1": {"name": "Scene 1"},
                    "scene_4": {"name": "Scene 4"},
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(main_scene, "SCENE_YAML_PATH", str(scene_path))

    class FakeWindow:
        def normalize_scene_cfg(self, cfg):
            return main_scene.MainWindow.normalize_scene_cfg(self, cfg)

        def apply_play_configs(self, cfg):
            return None

        def save_play_configs(self, cfg):
            return None

        def scene_cfg_for_save(self, cfg):
            return cfg

    cfg = main_scene.MainWindow.load_scene_cfg(FakeWindow())
    saved_cfg = yaml.safe_load(scene_path.read_text(encoding="utf-8"))

    assert cfg["current_scene"] == "scene_4"
    assert saved_cfg["current_scene"] == "scene_4"
    assert cfg["scenes"]["scene_4"]["length_m"] == 0.263
    assert cfg["scenes"]["scene_4"]["width_m"] == 0.263


def test_enter_calibration_waits_for_scene4_rail_motion(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    calls = []

    class FakeArmControl:
        enter_calibration_client = object()

        def send_request(self, client, msg, timeout_sec=5.0):
            calls.append((client, msg, timeout_sec))
            return object()

    main_scene.ArmControlNode.enter_calibration(FakeArmControl())

    assert calls[0][0] is FakeArmControl.enter_calibration_client
    assert calls[0][2] >= 90.0


def test_scene4_has_stop_play_button_and_handler(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    text = MAIN_SCENE_PATH.read_text(encoding="utf-8")

    assert "self.btn_scene4_stop = QPushButton('关闭玩法')" in text
    assert "self.btn_scene4_stop.pressed.connect(self.stop_scene4_tasks)" in text
    assert hasattr(main_scene.MainWindow, "stop_scene4_tasks")


def test_scene4_has_combined_color_and_waste_sorting(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    text = MAIN_SCENE_PATH.read_text(encoding="utf-8")

    assert "self.btn_scene4_start_play = QPushButton('开启玩法')" in text
    assert "self.btn_scene4_start_play.pressed.connect(self.start_scene4_all_sorting)" in text
    assert "self.btn_scene4_confirm_place = QPushButton('确认位置')" in text
    assert "self.btn_scene4_sort_all" not in text
    assert "def start_scene4_all_sorting(self):" in text
    assert "threading.Thread(target=self._scene4_all_sort_worker, daemon=True).start()" in text
    assert "def _scene4_all_sort_worker(self):" in text
    assert "ok, msg = self.node.start_color_and_waste_sorting()" in text
    assert len(main_scene.SCENE4_COLOR_KEYS) + len(main_scene.WASTE_KEYS) == 8
    assert "self.set_scene4_mode(SCENE4_MODE_ALL)" in text
    assert main_scene.SCENE4_MODE_ALL == "all"
    assert main_scene.scene4_keys_for_mode(main_scene.SCENE4_MODE_ALL) == (
        list(main_scene.SCENE4_COLOR_KEYS) + list(main_scene.WASTE_KEYS)
    )
    assert hasattr(main_scene.MainWindow, "start_scene4_all_sorting")


def test_scene4_color_actions_pass_single_target(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    text = MAIN_SCENE_PATH.read_text(encoding="utf-8")

    assert "def start_scene4_color(self, color_key=None):" in text
    assert "self.start_scene4_color(key)" not in text
    assert "threading.Thread(target=self._scene4_color_worker, args=(color_key,), daemon=True).start()" in text
    assert "def _scene4_color_worker(self, color_key=None):" in text
    assert "ok, msg = self.node.start_color_sorting(color_key, stop_all=True)" in text
    assert hasattr(main_scene.MainWindow, "start_scene4_color")


def test_scene_play_buttons_stop_calibration_before_starting_tasks(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    text = MAIN_SCENE_PATH.read_text(encoding="utf-8")

    assert hasattr(main_scene.MainWindow, "_stop_calibration_for_play")
    stop_helper = text.split("def _stop_calibration_for_play", 1)[1].split("\n    def ", 1)[0]
    assert "self.node.enable_calibration(False)" in stop_helper
    assert "self.node.exit_calibration()" in stop_helper

    scene4_waste = text.split("def start_scene4_waste", 1)[1].split("\n    def _scene4_waste_worker", 1)[0]
    scene4_color = text.split("def start_scene4_color", 1)[1].split("\n    def _scene4_color_worker", 1)[0]
    scene4_all = text.split("def start_scene4_all_sorting", 1)[1].split("\n    def _scene4_all_sort_worker", 1)[0]
    assert "self._stop_calibration_for_play()" in scene4_waste
    assert "self._stop_calibration_for_play()" in scene4_color
    assert "self._stop_calibration_for_play()" in scene4_all


def test_scene4_ui_supports_mode_switching_and_4_slot_frame(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    text = MAIN_SCENE_PATH.read_text(encoding="utf-8")
    scene4_board_class = text.split("class Scene4BoardWidget", 1)[1].split("class MainWindow", 1)[0]
    release_handler = scene4_board_class.split("def mouseReleaseEvent", 1)[1].split("def _draw_shelf_layer", 1)[0]
    shelf_layer = scene4_board_class.split("def _draw_shelf_layer", 1)[1].split("def _draw_frame_layer", 1)[0]
    frame_layer = scene4_board_class.split("def _draw_frame_layer", 1)[1].split("def _draw_card", 1)[0]
    destination_at = scene4_board_class.split("def _destination_at", 1)[1].split("def _destination_for_key", 1)[0]
    frame_layer = scene4_board_class.split("def _draw_frame_layer", 1)[1].split("def _draw_card", 1)[0]
    destination_at = scene4_board_class.split("def _destination_at", 1)[1].split("def _destination_for_key", 1)[0]
    scene = main_scene.DEFAULT_SCENE_CONFIG["scenes"]["scene_4"]

    assert main_scene.SCENE4_FRAME_SLOT_COUNT == 4
    assert len(main_scene.SCENE4_FRAME_SLOT_TARGETS) == 4
    assert main_scene.scene4_fixed_position("red", "frame", 1) == list(main_scene.SCENE4_FRAME_SLOT_TARGETS[1])
    assert main_scene.SCENE4_SHELF_SLOT_COUNT == 2
    assert main_scene.scene4_fixed_position("red", "upper_shelf", shelf_slot_index=1) == [0.24, -0.06, 0.315]
    assert main_scene.scene4_fixed_position("yellow", "lower_shelf", shelf_slot_index=1) == [0.10, -0.06, 0.190]
    assert main_scene.SCENE4_MODES == ("color", "waste", "all")
    assert main_scene.SCENE4_CONFIG_MODES == ("color", "waste", "all")
    all_keys = main_scene.scene4_keys_for_mode("all")
    assert main_scene.normalized_scene4_frame_slots(None, all_keys) == list(main_scene.SCENE4_COLOR_KEYS)
    assert main_scene.normalized_scene4_shelf_slots(None, all_keys, "upper_shelf") == [
        "residual_waste",
        "food_waste",
    ]
    assert main_scene.normalized_scene4_shelf_slots(None, all_keys, "lower_shelf") == [
        "hazardous_waste",
        "recyclable_waste",
    ]
    assert main_scene.SCENE4_PLACE_LABELS == {
        "frame": "框",
        "upper_shelf": "上层",
        "lower_shelf": "下层",
    }
    assert "self.btn_scene4_tag" not in text
    assert "def start_scene4_tag" not in text
    assert "self.fixed_layout = False" in scene4_board_class
    assert "self.scene4_coord_box = QGroupBox('场景4放置位置(m)')" not in text
    assert "self.scene4_coord_rows = []" not in text
    assert "self.scene4_place_combos = {}" not in text
    assert "def on_scene4_destination_row_changed" not in text
    assert "self.scene4_absolute_box = QGroupBox('场景4绝对放置位置 (m)', parent_widget)" in text
    assert "self.scene4_absolute_spins = {}" in text
    assert "def on_scene4_absolute_position_changed" in text
    assert "self.frame_slots = normalized_scene4_frame_slots(slots, self.item_keys)" in scene4_board_class
    assert "self.shelf_slots[destination] = normalized_scene4_shelf_slots" in scene4_board_class
    assert "_place_key_in_shelf_slot" in release_handler
    assert "_place_key_in_frame_slot" in release_handler
    assert "frameGridChanged.emit" in release_handler
    assert "slot_count = SCENE4_FRAME_SLOT_COUNT" in scene4_board_class
    assert "cols = 2" in scene4_board_class
    assert "rows = 2" in scene4_board_class
    assert "slot_count = max(slot_count, len(self.item_keys))" not in scene4_board_class
    assert ".adjusted(-4.0" not in destination_at
    assert "rects[destination].contains(point)" in destination_at
    assert "self.drag_destination == destination" in shelf_layer
    assert "QColor(245, 245, 245" in shelf_layer
    assert "self.drag_destination == SCENE4_PLACE_FRAME" in frame_layer
    assert "drawLine" not in shelf_layer
    assert ".adjusted(-4.0, -4.0, 4.0, 4.0)" not in destination_at
    assert "rects[destination].contains(point)" in destination_at
    assert "self.drag_destination == destination" in shelf_layer
    assert "QColor(245, 245, 245" in shelf_layer
    assert "self.drag_destination == SCENE4_PLACE_FRAME" in frame_layer
    assert scene["scene4_grid"]["waste_slots"] == list(main_scene.WASTE_KEYS)
    assert scene["scene4_grid"]["all_slots"] == list(main_scene.SCENE4_COLOR_KEYS)
    assert scene["scene4_grid"]["all_upper_slots"] == ["residual_waste", "food_waste"]
    assert scene["scene4_grid"]["all_lower_slots"] == ["hazardous_waste", "recyclable_waste"]
    assert "def set_scene4_mode(self, mode):" in text
    assert "self.scene4_board.set_scene_mode(mode, keys, labels, colors)" in text
    assert "self.scene4_board.frameGridChanged.connect(self.on_scene4_frame_grid_changed)" in text
    assert "def start_scene4_waste(self, waste_key=None):" in text
    assert "self.start_scene4_waste(key)" not in text


def test_scene3_and_scene4_defaults_match_board_size_requirements(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)

    assert main_scene.DEFAULT_CURRENT_SCENE == "scene_4"
    scene3 = main_scene.DEFAULT_SCENE_CONFIG["scenes"]["scene_3"]
    scene = main_scene.DEFAULT_SCENE_CONFIG["scenes"]["scene_4"]

    assert scene3["length_m"] == 0.263
    assert scene3["width_m"] == 0.263
    assert scene3["calibration_tag"]["center_in_map_m"] == {
        "x": -0.1115,
        "y": 0.1115,
        "z": 0.0,
    }
    assert scene3["scene3_grid"]["color_slots"] == list(main_scene.SCENE2_COLOR_KEYS)
    assert scene3["scene3_grid"]["waste_slots"] == list(main_scene.WASTE_KEYS)
    assert scene["length_m"] == 0.263
    assert scene["width_m"] == 0.263
    assert scene["home_pose"] == {
        "x": 145.0,
        "y": 0.0,
        "z": 290.0,
        "pitch": -90.0,
        "roll": 0.0,
        "claw": 0.0,
    }
    assert scene["calibration_pose"] == {
        "x": 145.0,
        "y": 0.0,
        "z": 290.0,
        "pitch": -90.0,
        "roll": 0.0,
        "claw": 0.0,
        "time_ms": 2000,
    }
    assert scene["rail"] == {
        "enabled": True,
        "total_steps": 4200,
        "subdivision": 2,
        "calibration_abs_position": 4000,
        "place_abs_position": 700,
        "reset_wait_sec": 18.0,
        "speed_steps_per_sec": 1000.0,
    }
    assert scene["place_targets"]["red"] == [0.285, 0.16, 0.015]
    assert scene["place_targets"]["green"] == [0.285, -0.16, 0.015]
    assert scene["place_targets"]["yellow"] == [0.115, 0.16, 0.015]
    assert scene["place_targets"]["blue"] == [0.115, -0.16, 0.015]
    assert scene["place_targets"]["residual_waste"] == [0.240, 0.060, 0.315]
    assert scene["place_targets"]["food_waste"] == [0.240, -0.060, 0.315]
    assert scene["place_targets"]["hazardous_waste"] == [0.100, 0.060, 0.190]
    assert scene["place_targets"]["recyclable_waste"] == [0.100, -0.060, 0.190]
    assert scene["scene4_pick"]["active_zone"] == "lower_board"
    assert scene["scene4_pick"]["lower_board"]["detection"]["min_v"] == 0
    assert scene["scene4_pick"]["lower_board"]["detection"]["max_v"] == 1080
    assert "upper_shelf" not in scene["scene4_pick"]
    assert len(scene["scene4_absolute_positions"]["frame_slots"]) == 4
    assert len(scene["scene4_absolute_positions"]["upper_shelf_slots"]) == 2
    assert len(scene["scene4_absolute_positions"]["lower_shelf_slots"]) == 2
    assert scene["scene4_place"]["targets"]["red"] == "frame"
    assert scene["scene4_place"]["targets"]["yellow"] == "frame"
    assert scene["scene4_place"]["targets"]["residual_waste"] == "upper_shelf"
    assert scene["scene4_place"]["targets"]["hazardous_waste"] == "lower_shelf"
    assert scene["scene4_shelf"]["length_m"] == 0.600
    assert scene["scene4_shelf"]["upper_z_m"] == 0.315
    assert scene["scene4_shelf"]["lower_z_m"] == 0.190
    assert scene["scene4_shelf"]["rail_slots"]["4"]["left"] == 3900
    assert scene["scene4_shelf"]["rail_slots"]["4"]["right"] == 1100
    assert scene["scene4_shelf"]["rail_slots"]["8"]["left"] == 7800
    assert scene["scene4_shelf"]["rail_slots"]["8"]["right"] == 2200
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["approach_pose"]["x"] == 270.0
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["approach_pose"]["z"] == 407.0
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["pose"]["x"] == 348.0
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["pose"]["z"] == 407.0
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["pose"]["pitch"] == 0.0
    assert scene["scene4_shelf"]["levels"]["lower_shelf"]["pose"]["x"] == 330.0
    assert scene["scene4_shelf"]["levels"]["lower_shelf"]["pose"]["z"] == 190.0
    assert scene["scene4_shelf"]["levels"]["lower_shelf"]["pose"]["pitch"] == 0.0


def test_scene3_and_scene4_normalization_force_board_size(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    cfg = {
        "current_scene": "scene_4",
        "scenes": {
            "scene_3": {
                "length_m": 0.280,
                "width_m": 0.220,
            },
            "scene_4": {
                "length_m": 0.280,
                "width_m": 0.220,
            },
        },
    }

    main_scene.MainWindow.normalize_scene_cfg(object(), cfg)

    assert cfg["scenes"]["scene_3"]["length_m"] == 0.263
    assert cfg["scenes"]["scene_3"]["width_m"] == 0.263
    assert cfg["scenes"]["scene_3"]["calibration_tag"]["center_in_map_m"] == {
        "x": -0.1115,
        "y": 0.1115,
        "z": 0.0,
    }
    assert cfg["scenes"]["scene_3"]["scene3_grid"]["color_slots"] == list(main_scene.SCENE2_COLOR_KEYS)
    assert cfg["scenes"]["scene_3"]["scene3_grid"]["waste_slots"] == list(main_scene.WASTE_KEYS)
    assert cfg["scenes"]["scene_4"]["length_m"] == 0.263
    assert cfg["scenes"]["scene_4"]["width_m"] == 0.263


def test_scene3_grid_normalization_maps_slots_to_place_targets(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    cfg = {
        "current_scene": "scene_3",
        "scenes": {
            "scene_3": {
                "scene3_grid": {
                    "color_slots": ["blue", "yellow", "green", "red"],
                    "color_slot_targets": [
                        [0.31, -0.12, 0.01],
                        [0.21, -0.11, 0.02],
                        [0.11, -0.10, 0.03],
                        [0.01, -0.09, 0.04],
                    ],
                    "waste_slots": ["food_waste", "recyclable_waste", "hazardous_waste", "residual_waste"],
                    "waste_slot_targets": [
                        [0.30, 0.12, 0.05],
                        [0.20, 0.11, 0.06],
                        [0.10, 0.10, 0.07],
                        [0.00, 0.09, 0.08],
                    ],
                },
            },
        },
    }

    main_scene.MainWindow.normalize_scene_cfg(object(), cfg)
    scene = cfg["scenes"]["scene_3"]

    assert scene["scene3_grid"]["color_slots"] == ["blue", "yellow", "green", "red"]
    assert scene["place_targets"]["blue"] == [0.31, -0.12, 0.01]
    assert scene["place_targets"]["yellow"] == [0.21, -0.11, 0.02]
    assert scene["scene3_grid"]["waste_slots"] == ["food_waste", "recyclable_waste", "hazardous_waste", "residual_waste"]
    assert scene["place_targets"]["food_waste"] == [0.30, 0.12, 0.05]
    assert scene["place_targets"]["residual_waste"] == [0.00, 0.09, 0.08]


def test_scene4_normalization_forces_fixed_targets_and_fills_rail(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    cfg = {
        "current_scene": "scene_4",
        "scenes": {
            "scene_4": {
                "place_targets": {
                    "red": [0.22, 0.05, 0.09],
                },
                "rail": {
                    "enabled": True,
                    "total_steps": "4200",
                    "subdivision": "2",
                    "calibration_abs_position": "4000",
                    "place_abs_position": "700",
                },
            },
        },
    }

    main_scene.MainWindow.normalize_scene_cfg(object(), cfg)
    scene = cfg["scenes"]["scene_4"]

    assert scene["length_m"] == 0.263
    assert scene["width_m"] == 0.263
    assert scene["place_targets"]["red"] == [0.285, 0.16, 0.015]
    assert scene["place_targets"]["yellow"] == [0.115, 0.16, 0.015]
    assert scene["place_targets"]["residual_waste"] == [0.24, 0.06, 0.315]
    assert scene["place_policy"]["only_left_y_positive"] is False
    assert scene["calibration_pose"]["x"] == 145.0
    assert scene["calibration_pose"]["time_ms"] == 2000
    assert scene["rail"]["enabled"] is True
    assert scene["rail"]["total_steps"] == 4200
    assert scene["rail"]["subdivision"] == 2
    assert scene["rail"]["calibration_abs_position"] == 4000
    assert scene["rail"]["place_abs_position"] == 700
    assert scene["scene4_pick"]["active_zone"] == "lower_board"
    assert scene["scene4_pick"]["lower_board"]["detection"]["min_v"] == 0
    assert scene["scene4_pick"]["lower_board"]["detection"]["max_v"] == 1080
    assert "upper_shelf" not in scene["scene4_pick"]
    assert scene["scene4_absolute_positions"]["upper_shelf_slots"][0] == [0.24, 0.06, 0.315]
    assert scene["scene4_place"]["targets"]["red"] == "frame"
    assert scene["scene4_place"]["targets"]["residual_waste"] == "upper_shelf"
    assert scene["scene4_shelf"]["rail_slots"]["4"]["left"] == 3900
    assert scene["scene4_shelf"]["rail_slots"]["4"]["right"] == 1100
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["approach_pose"]["x"] == 270.0
    assert scene["scene4_shelf"]["levels"]["upper_shelf"]["pose"]["x"] == 348.0


def test_scene4_normalization_preserves_destination_choices_and_snaps_fixed_targets(monkeypatch):
    main_scene = _load_main_scene(monkeypatch)
    cfg = {
        "current_scene": "scene_4",
        "scenes": {
            "scene_4": {
                "scene4_pick": {
                    "active_zone": "upper_shelf",
                },
                "scene4_place": {
                    "default_destination": "frame",
                    "targets": {
                        "red": "frame",
                        "blue": "upper_shelf",
                        "green": "lower_shelf",
                        "residual_waste": "frame",
                        "food_waste": "lower_shelf",
                    },
                },
                "scene4_grid": {
                    "color_slots": ["green", "red", "blue", "yellow"],
                    "waste_slots": ["food_waste", "residual_waste", "hazardous_waste", "recyclable_waste"],
                    "color_upper_slots": ["red", "blue"],
                    "color_lower_slots": ["yellow", "green"],
                    "waste_upper_slots": ["residual_waste", ""],
                    "waste_lower_slots": ["", "food_waste"],
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
                },
                "scene4_shelf": {
                    "rail_start_abs_position": "16000",
                    "rail_end_abs_position": "1200",
                    "levels": {
                        "shelf_level1": {
                            "target_z_m": "0.33",
                            "pose": {
                                "x": "382",
                                "z": "321",
                                "pitch": "-36",
                            },
                        },
                    },
                },
            },
        },
    }

    main_scene.MainWindow.normalize_scene_cfg(object(), cfg)
    scene = cfg["scenes"]["scene_4"]
    pick = scene["scene4_pick"]
    place = scene["scene4_place"]
    shelf = scene["scene4_shelf"]

    assert pick["active_zone"] == "lower_board"
    assert "upper_shelf" not in pick
    assert place["default_destination"] == "frame"
    assert place["targets"]["red"] == "frame"
    assert place["targets"]["blue"] == "upper_shelf"
    assert place["targets"]["green"] == "lower_shelf"
    assert place["targets"]["residual_waste"] == "frame"
    assert place["targets"]["food_waste"] == "lower_shelf"
    assert scene["scene4_grid"]["color_slots"] == ["green", "red", "blue", "yellow"]
    assert scene["scene4_grid"]["waste_slots"] == ["food_waste", "residual_waste", "hazardous_waste", "recyclable_waste"]
    assert scene["scene4_grid"]["all_slots"] == [
        "red",
        "green",
        "yellow",
        "blue",
    ]
    assert scene["scene4_grid"]["all_upper_slots"] == ["residual_waste", "food_waste"]
    assert scene["scene4_grid"]["all_lower_slots"] == ["hazardous_waste", "recyclable_waste"]
    assert scene["scene4_grid"]["color_upper_slots"] == ["red", "blue"]
    assert scene["scene4_grid"]["color_lower_slots"] == ["yellow", "green"]
    assert scene["scene4_grid"]["waste_upper_slots"] == ["residual_waste", ""]
    assert scene["scene4_grid"]["waste_lower_slots"] == ["", "food_waste"]
    assert scene["place_targets"]["red"] == [0.11, 0.21, 0.01]
    assert scene["place_targets"]["blue"] == [0.32, -0.02, 0.34]
    assert scene["place_targets"]["green"] == [0.22, -0.04, 0.19]
    assert scene["place_targets"]["residual_waste"] == [0.12, 0.22, 0.02]
    assert scene["place_targets"]["food_waste"] == [0.22, -0.04, 0.19]
    assert scene["scene4_absolute_positions"]["frame_slots"][1] == [0.12, 0.22, 0.02]
    assert shelf["rail_slots"]["4"]["left"] == 3900
    assert shelf["rail_slots"]["4"]["right"] == 1100
    assert shelf["levels"]["upper_shelf"]["target_z_m"] == 0.315
    assert shelf["levels"]["upper_shelf"]["approach_pose"]["x"] == 270.0
    assert shelf["levels"]["upper_shelf"]["approach_pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["x"] == 348.0
    assert shelf["levels"]["upper_shelf"]["pose"]["z"] == 407.0
    assert shelf["levels"]["upper_shelf"]["pose"]["pitch"] == 0.0
