#!/usr/bin/env python3
# coding: utf-8

import os

import numpy as np

from sdk import common


def load_axis_calibration(config_path, calibration_file="calibration.yaml"):
    return common.get_yaml_data(os.path.join(config_path, calibration_file)) or {}


def apply_axis_calibration(position, calibration, section):
    section_cfg = calibration.get(section, {}) if isinstance(calibration, dict) else {}
    offset = tuple(float(v) for v in section_cfg.get("offset", (0.0, 0.0, 0.0)))
    scale = tuple(float(v) for v in section_cfg.get("scale", (1.0, 1.0, 1.0)))
    result = np.asarray(position, dtype=np.float64).copy()
    for i in range(3):
        result[i] = result[i] * scale[i] + offset[i]
    return result


def projection_from_extristric(extristric):
    tvec, rmat = extristric
    return np.row_stack((
        np.column_stack((rmat, tvec)),
        np.array([[0, 0, 0, 1]], dtype=np.float64),
    ))


def pixel_to_calibrated_world(
    pixel,
    intrinsic,
    extristric,
    white_area_pose_world,
    calibration,
    height=0.02,
):
    projection_matrix = projection_from_extristric(extristric)
    world_pose = np.asarray(
        common.pixels_to_world([pixel], intrinsic, projection_matrix)[0],
        dtype=np.float64,
    )

    white_area_mat = np.asarray(white_area_pose_world, dtype=np.float64).reshape(4, 4)
    local_point = np.ones(4, dtype=np.float64)
    local_point[:3] = world_pose
    position = np.matmul(white_area_mat, local_point)[:3]
    position[2] = float(height)
    position = apply_axis_calibration(position, calibration, "pixel")
    return position, projection_matrix
