#!/usr/bin/python3
# coding=utf8
import os
import cv2
import time
import sdk.pid as pid


class ObjectTracker:
    def __init__(self, use_mouse=False):
        self.use_mouse = use_mouse
        if self.use_mouse:
            name = 'track'
            cv2.namedWindow(name, 1)
            cv2.setMouseCallback(name, self.onmouse)
        self.tracker = cv2.TrackerKCF_create()
        self.mouse_click = False
        self.selection = None
        self.track_window = None
        self.drag_start = None
        self.start_circle = True
        self.start_click = False
        self.active = False

        self.x_init = 105.0
        self.y_dis = 0.0
        self.z_dis = 200.0

        self.z_pid = pid.PID(0.065, 0.0, 0.001)
        self.depth_pid = pid.PID(0.00004, 0.0, 0.0)
        self.y_pid = pid.PID(0.05, 0.0, 0.0)

        # 追踪丢失后自动重试
        self._pending_bbox = None
        self._pending_image = None
        self._fail_count = 0
        self._max_fail = 5  # 连续丢失 N 帧后重新初始化

    def reset(self):
        self.tracker = cv2.TrackerKCF_create()
        self.mouse_click = False
        self.selection = None
        self.track_window = None
        self.drag_start = None
        self.start_circle = True
        self.start_click = False
        self.active = False

    def stop(self):
        self.active = False
        self.reset()

    def set_init_param(self, x_dis, y_dis, z_dis):
        self.x_init = float(x_dis)
        self.y_dis = float(y_dis)
        self.z_dis = float(z_dis)

    def update_pid(self, p1, p2, p3):
        self.z_pid = pid.PID(p1[0], p1[1], p1[2])
        self.depth_pid = pid.PID(p2[0], p2[1], p2[2])
        self.y_pid = pid.PID(p3[0], p3[1], p3[2])

    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_click = True
            self.drag_start = (x, y)
            self.track_window = None
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.reset()

    def set_track_target(self, target, image):
        self.reset()
        self._pending_bbox = tuple(target)
        self._pending_image = image.copy()
        self._fail_count = 0
        self.tracker = cv2.TrackerKCF_create()
        self.tracker.init(image, target)
        self.active = True

    def get_target(self, image):
        if self.start_circle and self.use_mouse:
            if self.track_window:
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 255, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click and not self.mouse_click:
                self.start_circle = False
            if not self.start_circle:
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
                self.active = True
        elif self.active:
            ok, box = self.tracker.update(image)
            if ok and min(box) > 0:
                self._fail_count = 0
                self._pending_bbox = tuple(box)
                self._pending_image = image.copy()
                return image, box
            # 追踪丢失，尝试重新初始化
            self._fail_count += 1
            if self._fail_count <= self._max_fail and self._pending_bbox is not None:
                self.tracker = cv2.TrackerKCF_create()
                self.tracker.init(image, self._pending_bbox)
                cv2.putText(image, "Re-initializing...", (10, 460),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)
            else:
                cv2.putText(image, "Tracking failure detected !", (10, 460),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                self.active = False
        return image, None

    def track(self, image):
        image, box = self.get_target(image)
        if box is None:
            return None

        img_h, img_w = image.shape[:2]
        p1 = (int(box[0]), int(box[1]))
        p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

        cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
        center_x = (p1[0] + p2[0]) / 2.0
        center_y = (p1[1] + p2[1]) / 2.0

        if abs(center_x - img_w / 2.0) < 15:
            center_x = img_w / 2.0
        if abs(center_y - img_h / 2.0) < 15:
            center_y = img_h / 2.0

        self.y_pid.SetPoint = img_w / 2.0
        self.y_pid.update(center_x)
        dy = max(-12.0, min(12.0, self.y_pid.output))
        self.y_dis += dy
        self.y_dis = max(-150.0, min(150.0, self.y_dis))

        self.z_pid.SetPoint = img_h / 2.0
        self.z_pid.update(center_y)
        dz = max(-12.0, min(12.0, self.z_pid.output))
        self.z_dis += dz
        self.z_dis = max(160.0, min(250.0, self.z_dis))

        return [float(self.y_dis), float(self.z_dis), image]


if __name__ == '__main__':
    cap = cv2.VideoCapture(-1)
    track = ObjectTracker(True)
    track.set_init_param(105.0, 0.0, 200.0)
    while True:
        try:
            ret, image = cap.read()
            if ret:
                result = track.track(image)
                frame = image if result is None else result[-1]
                cv2.imshow('track', frame)
                cv2.waitKey(1)
            else:
                time.sleep(0.01)
        except KeyboardInterrupt:
            break
    cap.release()
    cv2.destroyAllWindows()
