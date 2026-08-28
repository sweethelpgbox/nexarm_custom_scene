import inspect
import sys
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_action_group_sdk_methods_forward_expected_esp32_commands():
    from ros_robot_controller.ros_robot_controller_sdk import Board, ESP32Cmd

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def buf_write(self, target_id, cmd, data=[]):
            self.calls.append((target_id, cmd, list(data)))

    board = FakeBoard()

    Board.action_group_run(board, 7)
    Board.action_group_stop(board)
    Board.action_group_erase(board, 8)
    Board.action_group_download_raw(board, [1, 256, -1])

    assert board.calls == [
        (0xFF, ESP32Cmd.ACTION_GROUP_RUN, [7]),
        (0xFF, ESP32Cmd.ACTION_GROUP_STOP, []),
        (0xFF, ESP32Cmd.ACTION_GROUP_ERASE, [8]),
        (0xFF, ESP32Cmd.ACTION_GROUP_DOWNLOAD, [1, 0, 255]),
    ]


def test_action_group_node_callbacks_forward_to_board():
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def action_group_run(self, group_id):
            self.calls.append(("action_group_run", group_id))

        def action_group_stop(self):
            self.calls.append(("action_group_stop",))

        def action_group_erase(self, group_id):
            self.calls.append(("action_group_erase", group_id))

        def action_group_download_raw(self, payload):
            self.calls.append(("action_group_download_raw", list(payload)))

    controller = SimpleNamespace(board=FakeBoard())

    RosRobotController.action_group_run_callback(controller, SimpleNamespace(data=7))
    RosRobotController.action_group_stop_callback(controller, SimpleNamespace())
    RosRobotController.action_group_erase_callback(controller, SimpleNamespace(data=8))
    RosRobotController.action_group_download_callback(controller, SimpleNamespace(data=[1, 0, 255]))

    assert controller.board.calls == [
        ("action_group_run", 7),
        ("action_group_stop",),
        ("action_group_erase", 8),
        ("action_group_download_raw", [1, 0, 255]),
    ]


def test_ros_robot_controller_registers_action_group_subscriptions():
    from ros_robot_controller import ros_robot_controller_node

    source = inspect.getsource(ros_robot_controller_node.RosRobotController.__init__)

    expected = [
        "create_subscription(UInt8, '~/action_group/run'",
        "create_subscription(Empty, '~/action_group/stop'",
        "create_subscription(UInt8, '~/action_group/erase'",
        "create_subscription(UInt8MultiArray, '~/action_group/download'",
    ]

    for subscription in expected:
        assert subscription in source
