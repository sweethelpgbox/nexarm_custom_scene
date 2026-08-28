#!/usr/bin/env python3
# coding: utf8

import numpy as np
import mediapipe as mp
import sdk.pid as pid
from sdk.common import show_faces, mp_face_location, box_center, distance

TRACK_X_MM = 110.0
TRACK_Y_INIT = 0.0
TRACK_Z_INIT = 220.0
TRACK_Y_MIN = -150.0
TRACK_Y_MAX = 150.0
TRACK_Z_MIN = 80.0
TRACK_Z_MAX = 350.0

MAX_STEP_Y_MM = 10.0
MAX_STEP_Z_MM = 10.0
SMOOTH_ALPHA = 1.0
DEADZONE_X_PX = 18.0
DEADZONE_Y_PX = 14.0


class FaceTracker:
    def __init__(self, track_x_mm=110.0):
        self.tracker_type = 'face'
        self.track_x_mm = float(track_x_mm)
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.3,
        )

        self.pid_y = pid.PID(0.05, 0.0, 0.007)
        self.pid_z = pid.PID(0.05, 0.0, 0.007)
        self.detected_face = 0
        self.y = TRACK_Y_INIT
        self.z = TRACK_Z_INIT

    def reset(self):
        self.detected_face = 0
        self.y = TRACK_Y_INIT
        self.z = TRACK_Z_INIT
        self.pid_y.clear()
        self.pid_z.clear()

    # def proc(self, source_image, result_image):
    #     if source_image is None:
    #         return result_image, None

    #     results = self.face_detector.process(source_image)
    #     boxes, keypoints = mp_face_location(results, source_image)
    #     o_h, o_w = source_image.shape[:2]
    #     target_pose = None

    #     if len(boxes) > 0:
    #         self.detected_face += 1
    #         self.detected_face = min(self.detected_face, 20)

    #         if self.detected_face >= 5:
    #             center = [box_center(box) for box in boxes]
    #             dist = [distance(c, (o_w / 2, o_h / 2)) for c in center]
    #             face = min(zip(boxes, center, dist), key=lambda k: k[2])
    #             center_x, center_y = face[1]

    #             if abs(center_x - (o_w / 2)) > max(DEADZONE_X_PX, o_w * 0.02):
    #                 self.pid_y.SetPoint = o_w / 2
    #                 self.pid_y.update(center_x)
    #                 delta_y = max(-MAX_STEP_Y_MM, min(MAX_STEP_Y_MM, self.pid_y.output))
    #                 target_y = max(TRACK_Y_MIN, min(TRACK_Y_MAX, self.y + delta_y))
    #                 self.y += (target_y - self.y) * SMOOTH_ALPHA
    #             else:
    #                 self.pid_y.clear()

    #             if abs(center_y - (o_h / 2)) > max(DEADZONE_Y_PX, o_h * 0.02):
    #                 self.pid_z.SetPoint = o_h / 2
    #                 self.pid_z.update(center_y)
    #                 delta_z = max(-MAX_STEP_Z_MM, min(MAX_STEP_Z_MM, self.pid_z.output))
    #                 target_z = max(TRACK_Z_MIN, min(TRACK_Z_MAX, self.z + delta_z))
    #                 self.z += (target_z - self.z) * SMOOTH_ALPHA
    #             else:
    #                 self.pid_z.clear()

    #             target_pose = (self.track_x_mm, self.y, self.z)
    #     else:
    #         if self.detected_face > 0:
    #             self.detected_face -= 1
    #         else:
    #             self.pid_y.clear()
    #             self.pid_z.clear()

    #     result_image = show_faces(source_image, result_image, boxes, keypoints)
    #     return result_image, target_pose


    def proc(self, source_image, result_image):
        if source_image is None:
            return result_image, None

        results = self.face_detector.process(source_image)
        boxes, keypoints = mp_face_location(results, source_image)
        o_h, o_w = source_image.shape[:2]
        target_pose = None

        if len(boxes) > 0:
            self.detected_face += 1
            self.detected_face = min(self.detected_face, 20)

            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes]
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center]
                face = min(zip(boxes, center, dist), key=lambda k: k[2])
                center_x, center_y = face[1]

                self.pid_y.SetPoint = o_w / 2
                self.pid_z.SetPoint = o_h / 2

                err_x = center_x - o_w / 2
                err_y = center_y - o_h / 2

                if abs(err_x) > max(DEADZONE_X_PX, o_w * 0.02):
                    self.pid_y.update(center_x)
                    delta_y = float(np.clip(self.pid_y.output, -MAX_STEP_Y_MM, MAX_STEP_Y_MM))
                    self.y = float(np.clip(self.y + delta_y, TRACK_Y_MIN, TRACK_Y_MAX))
                else:
                    self.pid_y.clear()

                if abs(err_y) > max(DEADZONE_Y_PX, o_h * 0.02):
                    self.pid_z.update(center_y)
                    delta_z = float(np.clip(self.pid_z.output, -MAX_STEP_Z_MM, MAX_STEP_Z_MM))
                    self.z = float(np.clip(self.z + delta_z, TRACK_Z_MIN, TRACK_Z_MAX))
                else:
                    self.pid_z.clear()

                target_pose = (self.track_x_mm, self.y, self.z)
        else:
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_y.clear()
                self.pid_z.clear()

        result_image = show_faces(source_image, result_image, boxes, keypoints)
        return result_image, target_pose

