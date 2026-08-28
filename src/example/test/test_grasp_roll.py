import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
for module_name in list(sys.modules):
    if module_name == "example" or module_name.startswith("example."):
        del sys.modules[module_name]


from example.rgbd_function.include.shape_recognition import image_angle_to_arm_roll_deg
from example.rgbd_function.include.remove_too_high import (
    image_angle_to_arm_roll_deg as remove_high_image_angle_to_arm_roll_deg,
)


def test_shape_recognition_converts_image_angle_to_arm_roll():
    assert image_angle_to_arm_roll_deg(-60.0) == 60.0
    assert image_angle_to_arm_roll_deg(76.0) == -76.0


def test_remove_too_high_converts_image_angle_to_arm_roll():
    assert remove_high_image_angle_to_arm_roll_deg(None, -45.0) == 45.0
