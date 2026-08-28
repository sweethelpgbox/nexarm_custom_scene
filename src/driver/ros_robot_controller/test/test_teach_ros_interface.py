import inspect
import sys
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_teach_sdk_methods_forward_expected_esp32_commands():
    from ros_robot_controller.ros_robot_controller_sdk import Board, ESP32Cmd

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def buf_write(self, target_id, cmd, data=[]):
            self.calls.append((target_id, cmd, list(data)))

        def _debug(self, message):
            pass

        def _hex(self, data):
            return " ".join(f"{int(v) & 0xFF:02X}" for v in data)

    board = FakeBoard()
    methods = [
        ("set_pc_sync_teach", (2,), ESP32Cmd.PC_SYNC_TEACH, [2]),
        ("action_edit_enter", (), ESP32Cmd.ACTION_EDIT_ENTER, []),
        ("action_edit_exit", (), ESP32Cmd.ACTION_EDIT_EXIT, []),
        ("action_edit_start", (), ESP32Cmd.ACTION_EDIT_START, []),
        ("action_edit_stop", (), ESP32Cmd.ACTION_EDIT_STOP, []),
        ("action_edit_play", (), ESP32Cmd.ACTION_EDIT_PLAY, []),
        ("action_edit_play_stop", (), ESP32Cmd.ACTION_EDIT_PLAY_STOP, []),
        ("action_edit_clear", (), ESP32Cmd.ACTION_EDIT_CLEAR, []),
        ("action_edit_query", (), ESP32Cmd.ACTION_EDIT_QUERY, []),
        ("sync_teach_enter", (), ESP32Cmd.SYNC_TEACH_ENTER, []),
        ("sync_teach_exit", (), ESP32Cmd.SYNC_TEACH_EXIT, []),
        ("sync_teach_rec_start", (), ESP32Cmd.SYNC_TEACH_REC_START, []),
        ("sync_teach_rec_stop", (), ESP32Cmd.SYNC_TEACH_REC_STOP, []),
        ("sync_teach_play", (), ESP32Cmd.SYNC_TEACH_PLAY, []),
        ("sync_teach_play_stop", (), ESP32Cmd.SYNC_TEACH_PLAY_STOP, []),
        ("sync_teach_clear", (), ESP32Cmd.SYNC_TEACH_CLEAR, []),
        ("sync_teach_query", (), ESP32Cmd.SYNC_TEACH_QUERY, []),
    ]

    for method_name, args, _cmd, _data in methods:
        getattr(Board, method_name)(board, *args)

    assert board.calls == [(0xFF, cmd, data) for _name, _args, cmd, data in methods]


def test_teach_node_callbacks_forward_to_board():
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def set_pc_sync_teach(self, mode):
            self.calls.append(("set_pc_sync_teach", mode))

        def get_action_edit_status(self):
            self.calls.append(("get_action_edit_status",))
            return {"mode": 1, "recording": 0, "playing": 1, "count": 258}

        def get_sync_teach_status(self):
            self.calls.append(("get_sync_teach_status",))
            return {"mode": 1, "recording": 1, "playing": 0, "count": 515, "overflow": 1}

        def __getattr__(self, name):
            if name.startswith(("action_edit_", "sync_teach_")):
                return lambda: self.calls.append((name,))
            raise AttributeError(name)

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(list(msg.data))

    controller = SimpleNamespace(
        board=FakeBoard(),
        action_edit_status_pub=FakePublisher(),
        sync_teach_status_pub=FakePublisher(),
    )

    RosRobotController.set_pc_sync_teach_callback(controller, SimpleNamespace(data=3))
    for callback_name in (
        "action_edit_enter_callback",
        "action_edit_exit_callback",
        "action_edit_start_callback",
        "action_edit_stop_callback",
        "action_edit_play_callback",
        "action_edit_play_stop_callback",
        "action_edit_clear_callback",
        "action_edit_query_callback",
        "sync_teach_enter_callback",
        "sync_teach_exit_callback",
        "sync_teach_rec_start_callback",
        "sync_teach_rec_stop_callback",
        "sync_teach_play_callback",
        "sync_teach_play_stop_callback",
        "sync_teach_clear_callback",
        "sync_teach_query_callback",
    ):
        getattr(RosRobotController, callback_name)(controller, SimpleNamespace())

    assert controller.board.calls == [
        ("set_pc_sync_teach", 3),
        ("action_edit_enter",),
        ("action_edit_exit",),
        ("action_edit_start",),
        ("action_edit_stop",),
        ("action_edit_play",),
        ("action_edit_play_stop",),
        ("action_edit_clear",),
        ("get_action_edit_status",),
        ("sync_teach_enter",),
        ("sync_teach_exit",),
        ("sync_teach_rec_start",),
        ("sync_teach_rec_stop",),
        ("sync_teach_play",),
        ("sync_teach_play_stop",),
        ("sync_teach_clear",),
        ("get_sync_teach_status",),
    ]
    assert controller.action_edit_status_pub.messages == [[1, 0, 1, 2, 1]]
    assert controller.sync_teach_status_pub.messages == [[1, 1, 0, 3, 2, 1]]


def test_ros_robot_controller_registers_teach_subscriptions():
    from ros_robot_controller import ros_robot_controller_node

    source = inspect.getsource(ros_robot_controller_node.RosRobotController.__init__)

    expected = [
        "create_publisher(UInt8MultiArray, '~/action_edit/status'",
        "create_publisher(UInt8MultiArray, '~/sync_teach/status'",
        "create_subscription(UInt8, '~/set_pc_sync_teach'",
        "create_subscription(Empty, '~/action_edit/enter'",
        "create_subscription(Empty, '~/action_edit/exit'",
        "create_subscription(Empty, '~/action_edit/start'",
        "create_subscription(Empty, '~/action_edit/stop'",
        "create_subscription(Empty, '~/action_edit/play'",
        "create_subscription(Empty, '~/action_edit/play_stop'",
        "create_subscription(Empty, '~/action_edit/clear'",
        "create_subscription(Empty, '~/action_edit/query'",
        "create_subscription(Empty, '~/sync_teach/enter'",
        "create_subscription(Empty, '~/sync_teach/exit'",
        "create_subscription(Empty, '~/sync_teach/rec_start'",
        "create_subscription(Empty, '~/sync_teach/rec_stop'",
        "create_subscription(Empty, '~/sync_teach/play'",
        "create_subscription(Empty, '~/sync_teach/play_stop'",
        "create_subscription(Empty, '~/sync_teach/clear'",
        "create_subscription(Empty, '~/sync_teach/query'",
    ]

    for subscription in expected:
        assert subscription in source


def test_teach_sdk_routes_and_parses_status_packets():
    from ros_robot_controller.ros_robot_controller_sdk import Board, ESP32Cmd

    class FakeBoard:
        def __init__(self):
            import queue

            self.enable_recv = True
            self.action_edit_queue = queue.Queue(maxsize=10)
            self.sync_teach_queue = queue.Queue(maxsize=10)
            self.calls = []

        def buf_write(self, target_id, cmd, data=[]):
            self.calls.append((target_id, cmd, list(data)))
            if cmd == ESP32Cmd.ACTION_EDIT_QUERY:
                self.action_edit_queue.put([1, 0, 1, 2, 1])
            elif cmd == ESP32Cmd.SYNC_TEACH_QUERY:
                self.sync_teach_queue.put([1, 1, 0, 3, 2, 1])

        def action_edit_query(self):
            Board.action_edit_query(self)

        def sync_teach_query(self):
            Board.sync_teach_query(self)

        def _debug(self, message):
            pass

        def _hex(self, data):
            return " ".join(f"{int(v) & 0xFF:02X}" for v in data)

    board = FakeBoard()
    Board.dispatch_packet(board, 0xFF, ESP32Cmd.ACTION_EDIT_QUERY, [9, 8, 7, 6, 5])
    Board.dispatch_packet(board, 0xFF, ESP32Cmd.SYNC_TEACH_QUERY, [9, 8, 7, 6, 5, 4])
    assert board.action_edit_queue.get_nowait() == [9, 8, 7, 6, 5]
    assert board.sync_teach_queue.get_nowait() == [9, 8, 7, 6, 5, 4]

    assert Board.get_action_edit_status(board) == {
        "mode": 1,
        "recording": 0,
        "playing": 1,
        "count": 258,
    }
    assert Board.get_sync_teach_status(board) == {
        "mode": 1,
        "recording": 1,
        "playing": 0,
        "count": 515,
        "overflow": 1,
    }
    assert board.calls == [
        (0xFF, ESP32Cmd.ACTION_EDIT_QUERY, []),
        (0xFF, ESP32Cmd.SYNC_TEACH_QUERY, []),
    ]
