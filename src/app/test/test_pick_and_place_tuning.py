from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1] / "app"


def test_pick_and_place_uses_higher_safe_height_and_shallower_final_descent():
    text = (APP_DIR / "utils" / "pick_and_place.py").read_text(encoding="utf-8")

    assert "PICK_STAGE_CLEARANCE = 0.03" in text
    assert "PICK_LIFT_CLEARANCE = 0.03" in text
    assert "PICK_FINAL_DESCENT_MARGIN = 0.005" in text
    assert "effective_depth = max(0.0, float(gripper_depth) - PICK_FINAL_DESCENT_MARGIN)" in text
    assert "z_down = (position[2] - effective_depth) * 1000.0" in text


def test_scene4_color_light_claw_grab_angle_is_minus_19_5():
    text = (APP_DIR / "object_sorting.py").read_text(encoding="utf-8")

    assert "COLOR_CLAW_GRAB_ANGLE = -19.5" in text


def test_scene4_color_shelf_place_reuses_light_claw_hold_angle():
    text = (APP_DIR / "object_sorting.py").read_text(encoding="utf-8")

    shelf_executor = text.split("def _execute_scene4_shelf_place", 1)[1].split("    def init_process", 1)[0]
    shelf_call = text.split("shelf_place = self._resolve_scene4_shelf_place", 1)[1].split(
        "if scene4_lower:",
        1,
    )[0]

    assert "claw_hold = float(shelf_place.get('claw_hold', pick_and_place.CLAW_GRAB))" in shelf_executor
    assert "if 'claw_hold_angle' in place_kwargs:" in shelf_call
    assert "shelf_place['claw_hold'] = place_kwargs['claw_hold_angle']" in shelf_call
