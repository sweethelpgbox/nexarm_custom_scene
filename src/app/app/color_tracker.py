#!/usr/bin/env python3
# coding: utf8

import cv2
import sdk.pid as pid
from sdk import common
from sdk.common import get_area_max_contour, range_rgb

TRACK_X_MM = 110.0
TRACK_Y_INIT = 0.0
TRACK_Z_INIT = 220.0
TRACK_Y_MIN = -150.0
TRACK_Y_MAX = 150.0
TRACK_Z_MIN = 80.0
TRACK_Z_MAX = 350.0
DEADZONE_X_PX = 12.0
DEADZONE_Y_PX = 12.0
MAX_STEP_Y_MM = 15.0
MAX_STEP_Z_MM = 20.0
SMOOTH_ALPHA = 0.35


class ColorTracker:
    def __init__(self, target_color, track_x_mm=110.0):
        self.tracker_type = 'color'
        self.target_color = target_color
        self.track_x_mm = float(track_x_mm)
        self.pid_y = pid.PID(0.03, 0.0, 0.007)
        self.pid_z = pid.PID(0.05, 0.0, 0.007)
        self.y = TRACK_Y_INIT
        self.z = TRACK_Z_INIT

        self.data = common.get_yaml_data('/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml')
        self.lab_data = self.data['/**']['ros__parameters']

    def reset(self):
        self.y = TRACK_Y_INIT
        self.z = TRACK_Z_INIT
        self.pid_y.clear()
        self.pid_z.clear()

    def proc(self, source_image, result_image):
        if source_image is None:
            return result_image, None

        h, w = source_image.shape[:2]
        target_pose = None

        img_blur = cv2.GaussianBlur(source_image, (3, 3), 3)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB)
        mask = cv2.inRange(
            img_lab,
            tuple(self.lab_data['color_range_list'][self.target_color]['min']),
            tuple(self.lab_data['color_range_list'][self.target_color]['max'])
        )

        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area = get_area_max_contour(contours, 100)

        if max_contour_area[0] is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0])
            cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), range_rgb[self.target_color], 2)

            if abs(center_x - (w / 2)) > DEADZONE_X_PX:
                self.pid_y.SetPoint = w / 2
                self.pid_y.update(center_x)
                delta_y = max(-MAX_STEP_Y_MM, min(MAX_STEP_Y_MM, self.pid_y.output))
                target_y = max(TRACK_Y_MIN, min(TRACK_Y_MAX, self.y + delta_y))
                self.y += (target_y - self.y) * SMOOTH_ALPHA
            else:
                self.pid_y.clear()

            if abs(center_y - (h / 2)) > DEADZONE_Y_PX:
                self.pid_z.SetPoint = h / 2
                self.pid_z.update(center_y)
                delta_z = max(-MAX_STEP_Z_MM, min(MAX_STEP_Z_MM, self.pid_z.output))
                target_z = max(TRACK_Z_MIN, min(TRACK_Z_MAX, self.z + delta_z))
                self.z += (target_z - self.z) * SMOOTH_ALPHA
            else:
                self.pid_z.clear()

            target_pose = (self.track_x_mm, self.y, self.z)
            # else:
            #     self.pid_y.clear()
            #     self.pid_z.clear()
        else:
            self.pid_y.clear()
            self.pid_z.clear()

        return result_image, target_pose
