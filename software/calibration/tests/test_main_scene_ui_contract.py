from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "main_scene.py"


def source_text():
    return SOURCE.read_text(encoding="utf-8")


def test_builtin_scene_names_match_sandbox_products():
    text = source_text()
    expected_names = (
        "无沙盘场景",
        "基础分拣沙盘",
        "标准分拣沙盘",
        "豪华分拣沙盘",
        "电动滑轨货仓沙盘滑轨",
        "双臂流水线沙盘",
    )
    for name in expected_names:
        assert name in text


def test_layout_does_not_force_desktop_minimums_on_7inch_screen():
    text = source_text()
    desktop_only_fragments = (
        "max(760",
        "max(560",
        "min-width: 132px",
        "self.setMinimumSize(600, 430)",
        "self.scene4_result_preview.setMinimumSize(360, 280)",
    )
    for fragment in desktop_only_fragments:
        assert fragment not in text


def test_tab_text_uses_upright_side_tabs():
    text = source_text()
    assert "setTabPosition(QTabWidget.West)" in text
    assert "UprightWestTabBar" in text
    assert ".rotate(" not in text


def test_common_config_removes_point_and_scene_management_controls():
    text = source_text()
    removed_fragments = (
        "点位按钮(定位测试)",
        "场景放置点(物理坐标,m)",
        "btn_add_scene",
        "btn_del_scene",
        "新增场景",
        "删除场景",
    )
    for fragment in removed_fragments:
        assert fragment not in text


def test_standard_sorting_has_only_start_stop_play_controls():
    text = source_text()
    assert "btn_scene2_tag" not in text
    assert "标签夹取放置" not in text
    assert "btn_scene2_start_play = QPushButton('开启玩法')" in text
    assert "btn_scene2_stop = QPushButton('关闭玩法')" in text
    assert "btn_scene2_sort_all" not in text
    assert "start_scene2_all_sorting" in text
    assert "self.scene2_grid.colorClicked.connect(self.start_scene2_grid_target)" not in text


def test_luxury_sorting_has_only_start_stop_play_controls():
    text = source_text()
    assert "btn_scene3_start_play = QPushButton('开启玩法')" in text
    assert "btn_scene3_stop = QPushButton('关闭玩法')" in text
    assert "btn_scene3_sort_all" not in text
    assert "start_scene3_all_sorting" in text
    assert "self.scene3_board.colorClicked.connect(self.start_scene3_color)" not in text
    assert "self.scene3_board.wasteClicked.connect(self.start_scene3_waste)" not in text


def test_slide_rail_sorting_has_combined_sorting():
    text = source_text()
    assert "btn_scene4_start_play = QPushButton('开启玩法')" in text
    assert "btn_scene4_stop = QPushButton('关闭玩法')" in text
    assert "btn_scene4_confirm_place = QPushButton('确认位置')" in text
    assert "btn_scene4_place_offset = QPushButton('放置偏差微调')" in text
    assert "btn_scene4_sort_all" not in text
    assert "btn_scene4_color" not in text
    assert "btn_scene4_waste" not in text
    assert "start_scene4_all_sorting" in text
    assert "confirm_scene4_placement" in text
    assert "SCENE4_MODE_ALL = 'all'" in text
    assert "self.set_scene4_mode(SCENE4_MODE_ALL)" in text
    assert "SCENE4_FRAME_SLOT_COUNT = 4" in text
    assert "SCENE4_MODE_ALL: 'all_slots'" in text


def test_play_start_stop_buttons_keep_active_state_highlight():
    text = source_text()
    assert "QPushButton#playStateButton:checked" in text
    assert "def configure_play_state_buttons(self, start_button, stop_button, running=False)" in text
    assert "def set_scene_play_state(self, scene_id, running)" in text
    assert "self.configure_play_state_buttons(self.btn_scene2_start_play, self.btn_scene2_stop, running=False)" in text
    assert "self.configure_play_state_buttons(self.btn_scene3_start_play, self.btn_scene3_stop, running=False)" in text
    assert "self.configure_play_state_buttons(self.btn_scene4_start_play, self.btn_scene4_stop, running=False)" in text
    assert "self.configure_play_state_buttons(self.btn_scene5_start_pipeline, self.btn_scene5_stop, running=False)" in text
    assert "self.set_scene_play_state(SCENE2_ID, True)" in text
    assert "self.set_scene_play_state(SCENE2_ID, False)" in text
    assert "self.set_scene_play_state(SCENE3_ID, True)" in text
    assert "self.set_scene_play_state(SCENE3_ID, False)" in text
    assert "self.set_scene_play_state(SCENE4_ID, True)" in text
    assert "self.set_scene_play_state(SCENE4_ID, False)" in text
    assert "self.set_scene_play_state(SCENE5_ID, True)" in text
    assert "self.set_scene_play_state(SCENE5_ID, False)" in text


def test_standard_and_luxury_sorting_pages_do_not_show_color_grids():
    text = source_text()
    assert "self.scene2_grid = ColorGridWidget()" not in text
    assert "self.scene3_board = Scene3BoardWidget()" not in text
    assert "scene2_grid_box" not in text
    assert "场景2颜色格子" not in text
    assert "场景3布局" not in text
    assert "self.scene2_grid = Scene4BoardWidget()" not in text
    assert "self.scene3_board = Scene4BoardWidget()" not in text
    assert "self.scene5_place_map = Scene4BoardWidget()" not in text


def test_standard_luxury_and_dual_arm_pages_use_dark_slide_rail_style():
    text = source_text()
    assert "background:#DCE3E7" not in text
    assert "color:#475569" not in text
    assert "color: #475569" not in text


def test_global_calibration_and_scene5_speed_presets_are_available():
    text = source_text()
    assert "self.add_scroll_tab(scene1_page, '全场景标定')" in text
    assert "PLACE_OFFSET_LIMIT_M = 0.010" in text
    assert "'global_place_offset': dict(DEFAULT_GLOBAL_PLACE_OFFSET)" in text
    assert "btn_global_place_offset = QPushButton('放置偏差微调')" not in text
    assert "self.btn_scene2_place_offset = QPushButton('放置偏差微调')" in text
    assert "self.btn_scene3_place_offset = QPushButton('放置偏差微调')" in text
    assert "self.btn_scene4_place_offset = QPushButton('放置偏差微调')" in text
    assert "SCENE5_CONVEYOR_SPEED_PRESETS = (" in text
    assert "('低速', -20)" in text
    assert "('中速', -50)" in text
    assert "('高速', -100)" in text
    assert "self.scene5_speed_buttons = {}" in text
    assert "self.btn_scene5_speed_apply" not in text
    assert "self.cb_scene5_conveyor_speed" not in text
    assert "btn.pressed.connect(lambda s=speed: self.apply_scene5_conveyor_speed(s))" in text
    assert "self.btn_scene5_place_offset = QPushButton('放置偏差微调')" in text
    assert "self.scene5_arm_a_preview" in text
    assert "self.scene5_arm_b_preview" in text
    assert "SCENE5_ARM_A_IMAGE_TOPIC" in text
    assert "SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC" in text
    assert "sp_scene5_conveyor_speed" not in text
    assert "btn_scene5_arm_a_start" not in text
    assert "btn_scene5_arm_b_start" not in text
    assert "btn_scene5_conveyor_start" not in text
    assert "全场景标定中微调" not in text
    assert "本场景中微调" in text


def test_scene4_snap_highlight_is_scoped_by_destination():
    text = source_text()
    scene4_board = text.split("class Scene4BoardWidget", 1)[1].split("class MainWindow", 1)[0]
    destination_at = scene4_board.split("def _destination_at", 1)[1].split("def _destination_for_key", 1)[0]
    shelf_layer = scene4_board.split("def _draw_shelf_layer", 1)[1].split("def _draw_frame_layer", 1)[0]
    frame_layer = scene4_board.split("def _draw_frame_layer", 1)[1].split("def _draw_card", 1)[0]
    draw_card = scene4_board.split("def _draw_card", 1)[1].split("def paintEvent", 1)[0]

    assert ".adjusted(-4.0, -4.0, 4.0, 4.0)" not in destination_at
    assert "rects[destination].contains(point)" in destination_at
    assert "for index, slot_rect in enumerate(self._shelf_slot_rects(destination))" in shelf_layer
    assert "self.drag_destination == destination" in shelf_layer
    assert "QColor(245, 245, 245" in shelf_layer
    assert "self.drag_destination == SCENE4_PLACE_FRAME" in frame_layer
    assert "SCENE4_CARD_LABELS" in text
    assert "'residual_waste': '其'" in text
    assert "'food_waste': '厨'" in text
    assert "'hazardous_waste': '害'" in text
    assert "'recyclable_waste': '回'" in text
    assert "label = SCENE4_CARD_LABELS.get(key" in draw_card
    assert "rect.translated(0, 1)" in draw_card
    assert "font.setPointSize(max(8, min(14" in draw_card


def test_color_blocks_are_3cm_and_waste_cards_are_4cm():
    text = source_text()
    assert "COLOR_BLOCK_SIZE_M = 0.030" in text
    assert "WASTE_CARD_SIZE_M = 0.040" in text
    assert "'color_object_height_m': COLOR_BLOCK_SIZE_M" in text
    assert "'garbage_object_height_m': WASTE_CARD_SIZE_M" in text

    root = Path("/home/ubuntu")
    object_sorting = (root / "ros2_ws/src/app/app/object_sorting.py").read_text(encoding="utf-8")
    waste_classification = (root / "ros2_ws/src/app/app/waste_classification.py").read_text(encoding="utf-8")
    scene5_loader = (root / "ros2_ws/src/example/example/motor/scene5_arm_a_loader.py").read_text(encoding="utf-8")
    scene5_play = (root / "ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml").read_text(encoding="utf-8")

    assert "COLOR_OBJECT_HEIGHT_M = 0.03" in object_sorting
    assert "WASTE_CARD_HEIGHT_M = 0.04" in waste_classification
    assert "COLOR_OBJECT_HEIGHT_M = 0.03" in scene5_loader
    assert "GARBAGE_OBJECT_HEIGHT_M = 0.04" in scene5_loader
    assert "color_object_height_m: 0.03" in scene5_play
    assert "garbage_object_height_m: 0.04" in scene5_play


def test_gripper_close_angles_match_object_sizes():
    root = Path("/home/ubuntu")
    pick_and_place = (root / "ros2_ws/src/app/app/utils/pick_and_place.py").read_text(encoding="utf-8")
    object_sorting = (root / "ros2_ws/src/app/app/object_sorting.py").read_text(encoding="utf-8")
    scene5_loader = (root / "ros2_ws/src/example/example/motor/scene5_arm_a_loader.py").read_text(encoding="utf-8")
    scene5_worker = (root / "ros2_ws/src/example/example/motor/waste_classification_motor_depth.py").read_text(encoding="utf-8")
    scene5_play = (root / "ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml").read_text(encoding="utf-8")

    assert "CLAW_GRAB = -35.0" in pick_and_place
    assert "COLOR_CLAW_GRAB_ANGLE = -28.0" in object_sorting
    assert "COLOR_CLAW_GRAB_ANGLE = -28.0" in scene5_loader
    assert "GRAB_CLAW = -35.0" in scene5_worker
    assert "COLOR_GRAB_CLAW = -28.0" in scene5_worker
    assert "close_claw: -35.0" in scene5_play


def test_global_common_label_was_renamed_to_global_calibration():
    text = source_text()
    assert "全场景通用" not in text
    assert "全场景标定" in text


def test_scene4_calibration_uses_scene4_plane():
    text = source_text()
    scene4_default = text.split("SCENE4_ID: {", 1)[1].split("SCENE5_ID: {", 1)[0]
    scene4_normalize = text.split("if scene_id == SCENE4_ID:", 1)[1].split("if scene_id == SCENE5_ID:", 1)[0]

    assert "'use_calibration_scene': SCENE4_ID" in scene4_default
    assert "scene['use_calibration_scene'] = SCENE4_ID" in scene4_normalize


def test_scene4_manual_position_prepares_runtime_kinematics():
    text = source_text()
    scene4_play_keys = text.split("SCENE4_ID: (", 1)[1].split("SCENE5_ID:", 1)[0]
    scene4_kinematics = text.split("DEFAULT_SCENE4_KINEMATICS = {", 1)[1].split("DEFAULT_SCENE5_CALIBRATION_POSE", 1)[0]
    move_to_position = text.split("def move_to_position", 1)[1].split("def resolve_target_position", 1)[0]
    controller = (Path("/home/ubuntu") / "ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py").read_text(encoding="utf-8")
    apply_scene4 = controller.split("def _apply_scene4_kinematics", 1)[1].split("def _scene_kinematics_params", 1)[0]

    assert "'kinematics'," in scene4_play_keys
    assert "'params': [182.45, 225.0, 36.97, 145.0, 0.0, 130.23, 0.0, 50.0, 142.5]" in scene4_kinematics
    assert "self.scene_runtime_prepare_client = self.create_client(Trigger, '/ros_robot_controller/scene_runtime/prepare')" in text
    assert "result = self.node.prepare_scene_runtime()" in move_to_position
    assert "场景4底层参数准备失败" in move_to_position
    assert "target = self._scene_kinematics_params(scene_cfg)" in apply_scene4
    assert "self.board.set_kinematics_param(target)" in apply_scene4
    assert "without reading current params" in apply_scene4
