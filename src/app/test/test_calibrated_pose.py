import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_pixel_to_calibrated_world_applies_shared_map_and_pixel_calibration(monkeypatch):
    from app import calibrated_pose

    def fake_pixels_to_world(pixels, intrinsic, projection_matrix):
        assert pixels == [(120, 80)]
        assert intrinsic.shape == (3, 3)
        assert projection_matrix.shape == (4, 4)
        return [np.array([0.1, 0.2, 0.0], dtype=np.float64)]

    monkeypatch.setattr(calibrated_pose.common, "pixels_to_world", fake_pixels_to_world)

    extristric = (
        np.array([[0.0], [0.0], [0.3]], dtype=np.float64),
        np.eye(3, dtype=np.float64),
    )
    white_area_pose_world = np.eye(4, dtype=np.float64)
    white_area_pose_world[:3, 3] = [0.01, -0.02, 0.0]
    calibration = {
        "pixel": {
            "offset": [-0.015, 0.004, 0.001],
            "scale": [2.0, 3.0, 1.0],
        }
    }

    position, projection_matrix = calibrated_pose.pixel_to_calibrated_world(
        (120, 80),
        np.eye(3, dtype=np.float64),
        extristric,
        white_area_pose_world,
        calibration,
        height=0.02,
    )

    assert np.allclose(position, [0.205, 0.544, 0.021])
    assert projection_matrix.shape == (4, 4)


def test_apply_kinematics_calibration_uses_same_offset_and_scale_for_all_callers():
    from app.calibrated_pose import apply_axis_calibration

    position = apply_axis_calibration(
        [0.1, 0.2, 0.03],
        {
            "kinematics": {
                "offset": [-0.008, 0.002, -0.001],
                "scale": [1.5, 2.0, 1.0],
            }
        },
        "kinematics",
    )

    assert np.allclose(position, [0.142, 0.402, 0.029])
