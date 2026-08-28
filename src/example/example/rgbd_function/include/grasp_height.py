#!/usr/bin/python3
# coding=utf8


def side_grasp_z_from_top(top_z_m, object_height_m=None, min_z_m=0.008):
    top_z = max(float(top_z_m), float(min_z_m))
    if object_height_m is None:
        grasp_z = top_z * 0.5
    else:
        height = max(float(object_height_m), 0.0)
        grasp_z = top_z - height * 0.5
    return max(float(min_z_m), round(grasp_z, 6))
