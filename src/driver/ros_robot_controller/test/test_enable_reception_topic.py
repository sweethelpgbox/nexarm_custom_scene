import inspect
import sys
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_enable_reception_callback_forwards_bool_to_board():
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def enable_reception(self, enabled):
            self.calls.append(enabled)

    controller = SimpleNamespace(
        board=FakeBoard(),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    RosRobotController.enable_reception_callback(controller, SimpleNamespace(data=False))
    RosRobotController.enable_reception_callback(controller, SimpleNamespace(data=True))

    assert controller.board.calls == [False, True]


def test_ros_robot_controller_registers_enable_reception_subscription():
    from ros_robot_controller import ros_robot_controller_node

    source = inspect.getsource(ros_robot_controller_node.RosRobotController.__init__)

    assert "create_subscription(Bool, '~/enable_reception'" in source
    assert "self.enable_reception_callback" in source
