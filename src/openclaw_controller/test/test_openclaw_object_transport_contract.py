from pathlib import Path


NODE_FILE = (
    Path(__file__).resolve().parents[1]
    / "openclaw_controller"
    / "openclaw_object_transport.py"
)


def read_node_text():
    return NODE_FILE.read_text(encoding="utf-8")


def get_function_body(text, function_name, next_function_name):
    return text.split(f"def {function_name}(", 1)[1].split(
        f"def {next_function_name}(", 1
    )[0]


def test_main_skips_grasp_when_yaw_result_is_none_before_using_result():
    text = read_node_text()
    main_body = get_function_body(text, "main", "enter_srv_callback")

    none_guard_index = main_body.index("if result is None:")
    result_use_index = main_body.index("result[0]")
    none_guard_block = main_body[none_guard_index:result_use_index]

    assert none_guard_index < result_use_index
    assert "continue" in none_guard_block


def test_pick_yaw_uses_object_sorting_grasp_calculation_contract():
    text = read_node_text()

    assert "def calculate_pick_grasp_yaw(self, position, target, target_info, intrinsic, projection_matrix):" in text
    assert "common.calculate_pixel_length(0.09, intrinsic, projection_matrix)" in text
    assert "common.calculate_pixel_length(0.015, intrinsic, projection_matrix)" in text
    assert "calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info, gripper_size, yaw)" in text
    assert "angle = utils.get_long_edge_angle(rect)" in text
    assert "yaw_val = target[4]" not in text


def test_transport_state_stays_busy_until_pick_and_place_cycle_finishes():
    text = read_node_text()
    transport_body = get_function_body(text, "transport_thread", "main")

    pick_success_body = transport_body.split("if finish:", 1)[1]
    before_place_branch = pick_success_body.split(
        "if target_name in ('red', 'green', 'blue'):",
        1,
    )[0]

    assert "self.start_transport = False" not in before_place_branch
