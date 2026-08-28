import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class FakeBoard:
    def __init__(self):
        self.calls = []

    def stepper_reset(self):
        self.calls.append(("reset",))

    def stepper_set_div(self, code):
        self.calls.append(("div", code))

    def stepper_run(self, steps):
        self.calls.append(("run", steps))


def test_absolute_move_resets_unknown_position_then_runs_delta():
    from ros_robot_controller.stepper_runtime import StepperRuntime

    board = FakeBoard()
    runtime = StepperRuntime(board, sleep_fn=lambda _seconds: None)

    result = runtime.move(
        position=15000,
        absolute=True,
        reset_first=False,
        set_subdivision=True,
        subdivision=0x07,
        wait_for_motion=True,
        reset_wait_sec=18.0,
    )

    assert result.success is True
    assert result.current_position == 15000
    assert board.calls == [("reset",), ("div", 0x07), ("run", 15000)]


def test_absolute_move_uses_relative_delta_after_position_is_known():
    from ros_robot_controller.stepper_runtime import StepperRuntime

    board = FakeBoard()
    runtime = StepperRuntime(board, sleep_fn=lambda _seconds: None)
    runtime.current_position = 15000

    result = runtime.move(position=1800, absolute=True)

    assert result.success is True
    assert result.current_position == 1800
    assert board.calls == [("run", -13200)]
