import threading
import time
from dataclasses import dataclass


@dataclass
class StepperMoveResult:
    success: bool
    message: str
    current_position: int


class StepperRuntime:
    def __init__(self, board, sleep_fn=time.sleep, default_speed_steps_per_sec=1000.0):
        self.board = board
        self.sleep_fn = sleep_fn
        self.default_speed_steps_per_sec = float(default_speed_steps_per_sec)
        self.current_position = None
        self.lock = threading.RLock()

    def reset(self, reset_wait_sec=18.0):
        with self.lock:
            wait_sec = max(0.0, float(reset_wait_sec))
            self.board.stepper_reset()
            if wait_sec > 0.0:
                self.sleep_fn(wait_sec)
            self.current_position = 0
            return StepperMoveResult(True, "stepper reset wait elapsed", 0)

    def move(
        self,
        position,
        absolute=True,
        reset_first=False,
        set_subdivision=False,
        subdivision=0x07,
        wait_for_motion=True,
        reset_wait_sec=18.0,
        speed_steps_per_sec=None,
    ):
        with self.lock:
            target = int(position)
            if reset_first or (absolute and self.current_position is None):
                self.board.stepper_reset()
                wait_sec = max(0.0, float(reset_wait_sec))
                if wait_sec > 0.0:
                    self.sleep_fn(wait_sec)
                self.current_position = 0

            if set_subdivision:
                self.board.stepper_set_div(int(subdivision) & 0xFF)

            if absolute:
                delta = target - int(self.current_position)
                final_position = target
            else:
                delta = target
                base_position = 0 if self.current_position is None else int(self.current_position)
                final_position = base_position + delta

            if delta != 0:
                self.board.stepper_run(delta)
                if wait_for_motion:
                    speed = float(speed_steps_per_sec or self.default_speed_steps_per_sec)
                    if speed > 0:
                        self.sleep_fn(abs(delta) / speed)

            self.current_position = int(final_position)
            return StepperMoveResult(
                True,
                f"stepper moved delta={delta}, current={self.current_position}",
                self.current_position,
            )
