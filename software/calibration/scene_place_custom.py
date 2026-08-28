#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import math
import yaml
import threading
import struct
import numpy as np
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger, SetBool
from interfaces.srv import SetStringBool, SetStringList, SetFloat64
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from sensor_msgs.msg import Image
from PyQt5.QtCore import pyqtSignal, Qt, QPointF, QRectF
from PyQt5.QtGui import QImage, QPixmap, QColor, QPen, QBrush
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
    QMessageBox,
    QGroupBox,
    QListWidget,
    QSplitter,
)

try:
    from vispy import app as vapp
    from vispy import scene as vscene
    from vispy.visuals.transforms import MatrixTransform
    try:
        vapp.use_app('pyqt5')
    except Exception:
        pass
    HAS_VISPY = True
except Exception:
    HAS_VISPY = False
    MatrixTransform = None


DEFAULT_SCENE_ID = 'scene_1'
DEFAULT_HOME = {
    'x': 110.0,
    'y': 0.0,
    'z': 220.0,
    'pitch': -90.0,
    'roll': 0.0,
    'claw': 0.0,
    'time_ms': 1000,
}
DEFAULT_PLACE_POLICY = {
    'only_left_y_positive': True,
    'min_place_z': 0.015,
}
DEFAULT_PLACE_TARGETS = {
    'red': [0.087, 0.133, 0.015],
    'green': [0.017, 0.133, 0.015],
    'blue': [-0.053, 0.133, 0.015],
    'tag1': [-0.053, 0.063, 0.015],
    'tag2': [0.017, 0.063, 0.015],
    'tag3': [0.087, 0.063, 0.015],
    'residual_waste': [0.095, 0.214, 0.020],
    'food_waste': [0.040, 0.214, 0.020],
    'hazardous_waste': [-0.018, 0.214, 0.020],
    'recyclable_waste': [-0.070, 0.214, 0.020],
    'center': [0.235, 0.0, 0.015],
    'left_top': [0.285, 0.16, 0.015],
    'right_top': [0.285, -0.16, 0.015],
    'left_bottom': [0.115, 0.16, 0.015],
    'right_bottom': [0.115, -0.16, 0.015],
    'tag_stackup': [0.017, 0.063, 0.015],
}
TARGET_ROWS = [
    ('red', '红色/red'),
    ('green', '绿色/green'),
    ('blue', '蓝色/blue'),
    ('tag1', '标签1/tag1'),
    ('tag2', '标签2'),
    ('tag3', '标签3'),
    ('residual_waste', '其他垃圾'),
    ('food_waste', '厨余垃圾'),
    ('hazardous_waste', '有害垃圾'),
    ('recyclable_waste', '可回收'),
    ('center', '中心'),
    ('left_top', '左上'),
    ('right_top', '右上'),
    ('left_bottom', '左下'),
    ('right_bottom', '右下'),
    ('tag_stackup', '标签码垛'),
]

OBJECT_SORT_KEYS = ['red', 'green', 'blue', 'tag1', 'tag2', 'tag3']
WASTE_SORT_KEYS = ['residual_waste', 'food_waste', 'hazardous_waste', 'recyclable_waste']
TASK_TYPE_LABELS = {
    'color_track': '颜色识别(跟踪)',
    'color': '颜色分拣',
    'waste': '垃圾分类',
    'tag': '标签分拣',
    'stackup': '标签码垛',
}
TASK_TARGET_OPTIONS = {
    'color_track': [('red', '红色'), ('green', '绿色'), ('blue', '蓝色')],
    'color': [('red', '红色'), ('green', '绿色'), ('blue', '蓝色')],
    'waste': [('residual_waste', '其他垃圾'), ('food_waste', '厨余垃圾'),
              ('hazardous_waste', '有害垃圾'), ('recyclable_waste', '可回收垃圾')],
    'tag': [('tag1', '标签1'), ('tag2', '标签2'), ('tag3', '标签3')],
    'stackup': [('tag_stackup', '标签码垛')],
}
COLOR_TRACK_INDEX = {'red': 1.0, 'green': 2.0, 'blue': 3.0}

chassis_type = os.environ.get('CHASSIS_TYPE', '')
if chassis_type == 'Slide_Rails':
    CALIB_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration.yaml"
    SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
else:
    CALIB_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration.yaml"
    SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"

RESULT_IMAGE_TOPIC = os.environ.get('SCENE_RESULT_TOPIC', '/calibration/image_result')
RGB_IMAGE_TOPIC = os.environ.get('SCENE_RGB_TOPIC', '/depth_cam/rgb/image_raw')
DEPTH_IMAGE_TOPIC = os.environ.get('SCENE_DEPTH_TOPIC', '/depth_cam/depth/image_raw')
RGB_TOPIC_FALLBACKS = [
    RGB_IMAGE_TOPIC,
    '/depth_cam/rgb/image_raw',
    '/camera/color/image_raw',
    '/camera/rgb/image_raw',
    '/depth_cam/image_raw',
    '/usb_cam/image_raw',
]

init_finish = False


class ArmControlNode(Node):
    def __init__(self, name):
        global init_finish
        if not rclpy.ok():
            rclpy.init()
        super().__init__(name)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.enter_calibration_client = self.create_client(Trigger, 'calibration/enter')
        self.exit_calibration_client = self.create_client(Trigger, 'calibration/exit')

        self.object_sort_enter_client = self.create_client(Trigger, 'object_sorting/enter')
        self.object_sort_exit_client = self.create_client(Trigger, 'object_sorting/exit')
        self.object_sort_enable_client = self.create_client(SetBool, 'object_sorting/enable_sorting')
        self.object_sort_target_client = self.create_client(SetStringBool, 'object_sorting/set_target')

        self.object_track_enter_client = self.create_client(Trigger, 'object_tracking/enter')
        self.object_track_exit_client = self.create_client(Trigger, 'object_tracking/exit')
        self.object_track_enable_client = self.create_client(SetBool, 'object_tracking/enable_color_tracking')
        self.object_track_target_client = self.create_client(SetFloat64, 'object_tracking/set_target_color')

        self.waste_enter_client = self.create_client(Trigger, 'waste_classification/enter')
        self.waste_exit_client = self.create_client(Trigger, 'waste_classification/exit')
        self.waste_enable_client = self.create_client(SetBool, 'waste_classification/enable_transport')
        self.waste_target_client = self.create_client(SetStringList, 'waste_classification/set_target')

        self.tag_stack_enter_client = self.create_client(Trigger, 'tag_stackup/enter')
        self.tag_stack_exit_client = self.create_client(Trigger, 'tag_stackup/exit')
        self.tag_stack_enable_client = self.create_client(SetBool, 'tag_stackup/enable_stackup')

        self.preview_topics = {
            'result': RESULT_IMAGE_TOPIC,
            'rgb': RGB_IMAGE_TOPIC,
            'depth': DEPTH_IMAGE_TOPIC,
        }
        rgb_candidates = []
        for t in RGB_TOPIC_FALLBACKS:
            tt = str(t or '').strip()
            if tt and tt not in rgb_candidates:
                rgb_candidates.append(tt)
        self.preview_candidates = {
            'result': [self.preview_topics['result']],
            'rgb': rgb_candidates,
            'depth': [self.preview_topics['depth']],
        }
        self.preview_callbacks = {
            'result': None,
            'rgb': None,
            'depth': None,
            'status': None,
        }
        self.preview_online = {
            'result': False,
            'rgb': False,
            'depth': False,
        }
        self.result_sub = self.create_subscription(
            Image,
            self.preview_topics['result'],
            lambda msg: self._on_result_image(msg, self.preview_topics['result']),
            qos_profile_sensor_data,
        )
        self.rgb_subs = []
        for topic in self.preview_candidates['rgb']:
            self.rgb_subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    (lambda msg, t=topic: self._on_rgb_image(msg, t)),
                    qos_profile_sensor_data,
                )
            )
        self.depth_sub = self.create_subscription(
            Image,
            self.preview_topics['depth'],
            lambda msg: self._on_depth_image(msg, self.preview_topics['depth']),
            qos_profile_sensor_data,
        )
        self.arm_state_callback = None
        self.arm_state_sub = self.create_subscription(
            ArmFullState, '/ros_robot_controller/arm/full_state', self._on_arm_full_state, 5
        )
        self._rgb_debug_count = 0
        self._rgb_debug_last_ts = 0.0

        while self.arm_pub.get_subscription_count() == 0:
            self.get_logger().info('等待 ros_robot_controller 订阅...')
            time.sleep(0.5)
        init_finish = True

    def set_preview_callbacks(self, on_result=None, on_rgb=None, on_depth=None, on_status=None):
        self.preview_callbacks['result'] = on_result
        self.preview_callbacks['rgb'] = on_rgb
        self.preview_callbacks['depth'] = on_depth
        self.preview_callbacks['status'] = on_status
        if on_status is not None:
            on_status(
                f'视频订阅: result={self.preview_topics["result"]} | '
                f'rgb={self.preview_topics["rgb"]} | '
                f'depth={self.preview_topics["depth"]}'
            )

    def _on_result_image(self, msg, topic):
        self._emit_preview('result', msg, topic)

    def _on_rgb_image(self, msg, topic):
        self._rgb_debug_count += 1
        now = time.time()
        if now - self._rgb_debug_last_ts > 2.0:
            self._rgb_debug_last_ts = now
            self.get_logger().info(
                f'[RGB_DEBUG] topic={topic} encoding={msg.encoding} w={msg.width} h={msg.height} step={msg.step} count={self._rgb_debug_count}'
            )
        self._emit_preview('rgb', msg, topic)

    def _on_depth_image(self, msg, topic):
        self._emit_preview('depth', msg, topic)

    def _emit_preview(self, key, msg, topic=''):
        if topic:
            self.preview_topics[key] = topic
        cb = self.preview_callbacks.get(key)
        if cb is not None:
            cb(msg)
        if not self.preview_online.get(key, False):
            self.preview_online[key] = True
            status_cb = self.preview_callbacks.get('status')
            if status_cb is not None:
                status_cb(f'{key} 视频流已连接: {self.preview_topics.get(key, "")}')

    def set_arm_state_callback(self, callback):
        self.arm_state_callback = callback

    def _on_arm_full_state(self, msg):
        cb = self.arm_state_callback
        if cb is not None:
            cb(msg)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def send_request(self, client, msg, timeout_sec=3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'服务不可用: {client.srv_name}')
            return None
        future = client.call_async(msg)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f'服务超时: {client.srv_name}')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warn(f'服务异常: {client.srv_name} ({exc})')
            return None

    def _response_success(self, response):
        return response is not None and bool(getattr(response, 'success', False))

    def _response_message(self, response, default=''):
        if response is None:
            return default
        msg = getattr(response, 'message', '')
        if isinstance(msg, str) and msg.strip():
            return msg.strip()
        return default

    def enter_calibration(self):
        return self.send_request(self.enter_calibration_client, Trigger.Request(), timeout_sec=2.0)

    def exit_calibration(self):
        return self.send_request(self.exit_calibration_client, Trigger.Request(), timeout_sec=2.0)

    def init_pose(self, home):
        time_ms = int(home.get('time_ms', 1000))
        self.publish_arm(home['x'], home['y'], home['z'], home['pitch'], home['roll'], home['claw'], time_ms)
        time.sleep(max(0.0, time_ms / 1000.0))
        self.publish_arm(home['x'], home['y'], home['z'], home['pitch'], home['roll'], home['claw'], time_ms)
        time.sleep(max(0.0, time_ms / 1000.0))

    def move_to(self, position_m, roll_deg, home, time_ms=1500):
        x_mm = float(position_m[0]) * 1000.0
        y_mm = float(position_m[1]) * 1000.0
        z_mm = float(position_m[2]) * 1000.0
        self.publish_arm(x_mm, y_mm, z_mm, home['pitch'], float(roll_deg), home['claw'], time_ms)

    def task_stop_all(self):
        feedback = []
        req = SetBool.Request(); req.data = False
        res = self.send_request(self.object_track_enable_client, req)
        if res is not None:
            feedback.append(f'object_tracking/enable_color_tracking(false): {self._response_message(res, "ok")}')
        res = self.send_request(self.object_track_exit_client, Trigger.Request())
        if res is not None:
            feedback.append(f'object_tracking/exit: {self._response_message(res, "ok")}')

        for key in OBJECT_SORT_KEYS:
            msg = SetStringBool.Request()
            msg.data_str = key
            msg.data_bool = False
            res = self.send_request(self.object_sort_target_client, msg)
            if res is not None:
                feedback.append(f'object_sorting/set_target({key}=false): {self._response_message(res, "ok")}')
        req = SetBool.Request(); req.data = False
        res = self.send_request(self.object_sort_enable_client, req)
        if res is not None:
            feedback.append(f'object_sorting/enable_sorting(false): {self._response_message(res, "ok")}')

        req = SetBool.Request(); req.data = False
        res = self.send_request(self.waste_enable_client, req)
        if res is not None:
            feedback.append(f'waste_classification/enable_transport(false): {self._response_message(res, "ok")}')

        req = SetBool.Request(); req.data = False
        res = self.send_request(self.tag_stack_enable_client, req)
        if res is not None:
            feedback.append(f'tag_stackup/enable_stackup(false): {self._response_message(res, "ok")}')

        res = self.send_request(self.object_sort_exit_client, Trigger.Request())
        if res is not None:
            feedback.append(f'object_sorting/exit: {self._response_message(res, "ok")}')
        res = self.send_request(self.waste_exit_client, Trigger.Request())
        if res is not None:
            feedback.append(f'waste_classification/exit: {self._response_message(res, "ok")}')
        res = self.send_request(self.tag_stack_exit_client, Trigger.Request())
        if res is not None:
            feedback.append(f'tag_stackup/exit: {self._response_message(res, "ok")}')
        return True, ' | '.join(feedback) if feedback else 'stop_all done'

    def run_task(self, task_type, target_key):
        _ok_stop, stop_feedback = self.task_stop_all()
        feedback = [stop_feedback]

        if task_type == 'color_track':
            res = self.send_request(self.object_track_enter_client, Trigger.Request())
            if not self._response_success(res):
                return False, f'object_tracking/enter failed ({self._response_message(res, "no response")})'
            feedback.append(f'object_tracking/enter: {self._response_message(res, "ok")}')

            color_idx = COLOR_TRACK_INDEX.get(target_key, 1.0)
            req_color = SetFloat64.Request()
            req_color.data = float(color_idx)
            res = self.send_request(self.object_track_target_client, req_color)
            if not self._response_success(res):
                return False, f'object_tracking/set_target_color failed ({self._response_message(res, "no response")})'
            feedback.append(f'object_tracking/set_target_color({target_key}={int(color_idx)}): {self._response_message(res, "ok")}')

            req_enable = SetBool.Request()
            req_enable.data = True
            res = self.send_request(self.object_track_enable_client, req_enable)
            if not self._response_success(res):
                return False, f'object_tracking/enable_color_tracking failed ({self._response_message(res, "no response")})'
            feedback.append(f'object_tracking/enable_color_tracking(true): {self._response_message(res, "ok")}')
            return True, ' | '.join(feedback)

        if task_type == 'color' or task_type == 'tag':
            res = self.send_request(self.object_sort_enter_client, Trigger.Request())
            if not self._response_success(res):
                return False, f'object_sorting/enter failed ({self._response_message(res, "no response")})'
            feedback.append(f'object_sorting/enter: {self._response_message(res, "ok")}')
            for key in OBJECT_SORT_KEYS:
                msg = SetStringBool.Request()
                msg.data_str = key
                msg.data_bool = (key == target_key)
                res = self.send_request(self.object_sort_target_client, msg)
                if not self._response_success(res):
                    return False, f'object_sorting/set_target failed: {key} ({self._response_message(res, "no response")})'
                feedback.append(
                    f'object_sorting/set_target({key}={"true" if key == target_key else "false"}): {self._response_message(res, "ok")}'
                )
            req = SetBool.Request(); req.data = True
            res = self.send_request(self.object_sort_enable_client, req)
            if not self._response_success(res):
                return False, f'object_sorting/enable_sorting failed ({self._response_message(res, "no response")})'
            feedback.append(f'object_sorting/enable_sorting(true): {self._response_message(res, "ok")}')
            return True, ' | '.join(feedback)

        if task_type == 'waste':
            res = self.send_request(self.waste_enter_client, Trigger.Request())
            if not self._response_success(res):
                return False, f'waste_classification/enter failed ({self._response_message(res, "no response")})'
            feedback.append(f'waste_classification/enter: {self._response_message(res, "ok")}')
            msg = SetStringList.Request()
            msg.data = [target_key]
            res = self.send_request(self.waste_target_client, msg)
            if not self._response_success(res):
                return False, f'waste_classification/set_target failed: {target_key} ({self._response_message(res, "no response")})'
            feedback.append(f'waste_classification/set_target({target_key}): {self._response_message(res, "ok")}')
            req = SetBool.Request(); req.data = True
            res = self.send_request(self.waste_enable_client, req)
            if not self._response_success(res):
                return False, f'waste_classification/enable_transport failed ({self._response_message(res, "no response")})'
            feedback.append(f'waste_classification/enable_transport(true): {self._response_message(res, "ok")}')
            return True, ' | '.join(feedback)

        if task_type == 'stackup':
            res = self.send_request(self.tag_stack_enter_client, Trigger.Request())
            if not self._response_success(res):
                return False, f'tag_stackup/enter failed ({self._response_message(res, "no response")})'
            feedback.append(f'tag_stackup/enter: {self._response_message(res, "ok")}')
            req = SetBool.Request(); req.data = True
            res = self.send_request(self.tag_stack_enable_client, req)
            if not self._response_success(res):
                return False, f'tag_stackup/enable_stackup failed ({self._response_message(res, "no response")})'
            feedback.append(f'tag_stackup/enable_stackup(true): {self._response_message(res, "ok")}')
            return True, ' | '.join(feedback)
        return False, f'unknown task type: {task_type}'

class ArmSimWidget(QWidget):
    target_moved = pyqtSignal(str, float, float)
    target_clicked = pyqtSignal(str)
    URDF_ENV = 'NEXARM_URDF_PATH'
    MESH_ENV = 'NEXARM_MESH_DIR'
    TRI_LIMIT_ENV = 'SCENE_3D_MAX_TRIANGLES'
    SHADING_ENV = 'SCENE_3D_SHADING'
    URDF_CANDIDATES = [
        '/home/ubuntu/ros2_ws/src/simulations/nexarm_description/urdf/nexarm.urdf',
        '/home/ubuntu/ros2_ws/src/simulations/nexarm_description/nexarm.urdf',
        '/home/ubuntu/ros2_ws/src/driver/nexarm_qt/nexarm_qt/ui/nexarm.urdf',
    ]
    SCENE_URDF_BASE = '/home/ubuntu/software'
    JOINT_FEED_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_base_joint']
    SERVO_CENTER = [2048.0, 2048.0, 1198.0, 1238.0, 2048.0, 2048.0]
    SERVO_RATIO = [0.05859375, 0.087890625, 0.087890625, 0.087890625, 0.05859375, 0.05859375]
    SERVO_DIR = [1.0, 1.0, -1.0, -1.0, -1.0, -1.0]
    URDF_DIR = [1.0, -1.0, 1.0, -1.0, 1.0, 1.0]
    MODEL_RGBA = (0.82, 0.84, 0.86, 1.0)
    GRID_CELLS = 3
    GRID_CELL_M = 0.03
    GRID_HALF_SPAN_M = 0.45
    TARGET_SIZE_M = 0.022
    TARGET_HEIGHT_M = 0.020
    TARGET_Z_M = 0.0
    TARGET_BOUNDS = {
        'x_min': -0.20,
        'x_max': 0.28,
        'y_min': -0.22,
        'y_max': 0.28,
    }
    TARGET_COLORS = {
        'red': (0.94, 0.26, 0.26, 1.0),
        'green': (0.18, 0.74, 0.35, 1.0),
        'blue': (0.23, 0.49, 0.90, 1.0),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 300)
        self.state = {
            'x': float(DEFAULT_HOME['x']),
            'y': float(DEFAULT_HOME['y']),
            'z': float(DEFAULT_HOME['z']),
            'pitch': float(DEFAULT_HOME['pitch']),
            'roll': float(DEFAULT_HOME['roll']),
            'claw': float(DEFAULT_HOME['claw']),
            'yaw': 0.0,
            'joint_angles': [0.0] * 6,
            'servos': [0] * 6,
        }
        self.model_loaded = False
        self.load_error = ''
        self.root_link = 'base_link'
        self.link_visuals = {}
        self.joints = {}
        self.child_joints = {}
        self.parent_joint = {}
        self.mesh_nodes = []
        self.material_colors = {}
        self.scene_urdf_source = ''
        self.loaded_urdf_path = ''
        self._auto_decimate_enabled = False
        self.target_xy = {
            'red': [float(DEFAULT_PLACE_TARGETS['red'][0]), float(DEFAULT_PLACE_TARGETS['red'][1])],
            'green': [float(DEFAULT_PLACE_TARGETS['green'][0]), float(DEFAULT_PLACE_TARGETS['green'][1])],
            'blue': [float(DEFAULT_PLACE_TARGETS['blue'][0]), float(DEFAULT_PLACE_TARGETS['blue'][1])],
        }
        self.target_z = {
            'red': float(DEFAULT_PLACE_TARGETS['red'][2]),
            'green': float(DEFAULT_PLACE_TARGETS['green'][2]),
            'blue': float(DEFAULT_PLACE_TARGETS['blue'][2]),
        }
        self.target_nodes = {}
        self._drag_target_key = None
        self._drag_last_pos = None
        self._drag_moved = False
        self._drag_press_pos = None
        self._side_selected_key = None
        self._place_edit_enabled = True
        tri_limit_text = os.environ.get(self.TRI_LIMIT_ENV, '0').strip()
        try:
            self.max_triangles_per_mesh = int(tri_limit_text)
        except Exception:
            self.max_triangles_per_mesh = 0
        if self.max_triangles_per_mesh > 0:
            self.max_triangles_per_mesh = max(1200, min(self.max_triangles_per_mesh, 60000))
        shading_text = os.environ.get(self.SHADING_ENV, 'flat').strip().lower()
        if shading_text in ('none', 'off', '0'):
            self.mesh_shading = None
        elif shading_text in ('smooth',):
            self.mesh_shading = 'smooth'
        else:
            self.mesh_shading = 'flat'
        self.total_triangles_loaded = 0

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        if not HAS_VISPY:
            msg = QLabel('vispy 不可用，无法显示 3D 视图')
            msg.setAlignment(Qt.AlignCenter)
            lay.addWidget(msg)
            self.canvas = None
            return

        self.canvas = vscene.SceneCanvas(keys=None, show=False, bgcolor='#F1F5F9')
        native = self.canvas.native
        if isinstance(native, QWidget):
            lay.addWidget(native)
        else:
            # Some backends expose a non-QWidget native; try wrapping as QWindow.
            try:
                container = QWidget.createWindowContainer(native)
                lay.addWidget(container)
            except Exception:
                msg = QLabel(f'vispy 后端嵌入失败: {type(native).__name__}')
                msg.setAlignment(Qt.AlignCenter)
                lay.addWidget(msg)
                self.canvas = None
                return

        self.view = self.canvas.central_widget.add_view()
        self.view.camera = vscene.cameras.TurntableCamera(
            fov=45.0, distance=0.7, elevation=22.0, azimuth=35.0, center=(0.10, 0.0, 0.11)
        )
        self.view.camera.interactive = True
        vscene.visuals.XYZAxis(parent=self.view.scene)
        self._add_floor_grid()
        self._create_target_nodes()
        self.canvas.events.mouse_press.connect(self._on_canvas_mouse_press)
        self.canvas.events.mouse_move.connect(self._on_canvas_mouse_move)
        self.canvas.events.mouse_release.connect(self._on_canvas_mouse_release)
        self.status_label = QLabel('URDF: 加载中...')
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.status_label.setStyleSheet(
            'background-color: rgba(15, 23, 42, 160); color: #E5E7EB; padding: 4px 8px; border-radius: 4px;'
        )
        lay.addWidget(self.status_label)
        self._load_urdf_model()
        self._apply_state_to_model(self.state)

    def _add_floor_grid(self):
        half = self.GRID_HALF_SPAN_M
        step = self.GRID_CELL_M
        z = 0.00035
        minor_pts = []
        major_pts = []
        lines = int(round((2.0 * half) / step))
        for i in range(lines + 1):
            u = -half + i * step
            dst = major_pts if (i % self.GRID_CELLS == 0) else minor_pts
            dst.extend([[u, -half, z], [u, half, z]])
            dst.extend([[-half, u, z], [half, u, z]])
        if minor_pts:
            minor = np.asarray(minor_pts, dtype=np.float32)
            vscene.visuals.Line(
                pos=minor,
                connect=np.arange(minor.shape[0], dtype=np.int32).reshape(-1, 2),
                color=(0.75, 0.77, 0.80, 0.60),
                width=1.0,
                parent=self.view.scene,
            )
        if major_pts:
            major = np.asarray(major_pts, dtype=np.float32)
            vscene.visuals.Line(
                pos=major,
                connect=np.arange(major.shape[0], dtype=np.int32).reshape(-1, 2),
                color=(0.62, 0.65, 0.69, 0.95),
                width=1.2,
                parent=self.view.scene,
            )

    def _create_target_nodes(self):
        s = self.TARGET_SIZE_M * 0.5
        h = self.TARGET_HEIGHT_M
        verts = np.array([
            [-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0],
            [-s, -s, h], [s, -s, h], [s, s, h], [-s, s, h],
        ], dtype=np.float32)
        faces = np.array([
            [0, 1, 2], [0, 2, 3],  # bottom
            [4, 6, 5], [4, 7, 6],  # top
            [0, 4, 5], [0, 5, 1],  # side -y
            [1, 5, 6], [1, 6, 2],  # side +x
            [2, 6, 7], [2, 7, 3],  # side +y
            [3, 7, 4], [3, 4, 0],  # side -x
        ], dtype=np.uint32)
        for key in ('red', 'green', 'blue'):
            vis = vscene.visuals.Mesh(
                vertices=verts,
                faces=faces,
                color=self.TARGET_COLORS[key],
                shading='flat',
                parent=self.view.scene,
            )
            vis.set_gl_state('opaque', depth_test=True, cull_face=False, blend=False)
            vis.transform = MatrixTransform()
            self.target_nodes[key] = vis
        self._update_target_nodes()

    @staticmethod
    def _safe_pos_tuple(pos):
        try:
            return float(pos[0]), float(pos[1])
        except Exception:
            return 0.0, 0.0

    def _canvas_to_scene_xy(self, pos):
        if self.canvas is None:
            return None
        px, py = self._safe_pos_tuple(pos)
        try:
            tr = self.canvas.scene.node_transform(self.view.scene)
            mapped = tr.imap(np.array([px, py], dtype=np.float32))
            return float(mapped[0]), float(mapped[1])
        except Exception:
            return None

    def _pick_target_by_screen(self, pos):
        if self.canvas is None:
            return None
        try:
            vis = self.canvas.visual_at(pos)
        except Exception:
            return None
        for key, node in self.target_nodes.items():
            if vis is node:
                return key
        return None

    def _update_target_nodes(self):
        for key, vis in self.target_nodes.items():
            x, y = self.target_xy.get(key, [0.0, 0.0])
            z = float(self.target_z.get(key, self.TARGET_Z_M))
            m = np.eye(4, dtype=np.float32)
            m[0, 3] = float(x)
            m[1, 3] = float(y)
            m[2, 3] = max(self.TARGET_Z_M, z)
            tr = vis.transform
            if not isinstance(tr, MatrixTransform):
                tr = MatrixTransform()
                vis.transform = tr
            tr.matrix = m.T

    def set_place_targets(self, place_targets):
        for key in ('red', 'green', 'blue'):
            pos = place_targets.get(key, DEFAULT_PLACE_TARGETS[key])
            x = float(pos[0])
            y = float(pos[1])
            z = float(pos[2]) if len(pos) >= 3 else float(DEFAULT_PLACE_TARGETS[key][2])
            x = self._clamp(x, self.TARGET_BOUNDS['x_min'], self.TARGET_BOUNDS['x_max'])
            y = self._clamp(y, self.TARGET_BOUNDS['y_min'], self.TARGET_BOUNDS['y_max'])
            self.target_xy[key] = [x, y]
            self.target_z[key] = z
        self._update_target_nodes()

    def set_active_target_from_side(self, key):
        if key in self.target_xy:
            self._side_selected_key = key

    def set_place_edit_enabled(self, enabled):
        self._place_edit_enabled = bool(enabled)
        if not self._place_edit_enabled:
            self._drag_target_key = None
            self._drag_last_pos = None
        if hasattr(self, 'view') and hasattr(self.view, 'camera') and self.view.camera is not None:
            self.view.camera.interactive = True

    def _pixel_to_world_delta(self, dx_px, dy_px):
        # Clamp per-event delta to avoid occasional jump frames.
        dx_px = self._clamp(dx_px, -16.0, 16.0)
        dy_px = self._clamp(dy_px, -16.0, 16.0)
        # Use screen-proportional mapping for stable dragging.
        w = max(220.0, float(self.width()))
        span = self.GRID_HALF_SPAN_M * 2.0
        scale = span / w
        wx = dx_px * scale
        wy = -dy_px * scale
        return float(wx), float(wy)

    def _on_canvas_mouse_press(self, event):
        if event.button != 1:
            return
        if not self._place_edit_enabled:
            return
        key = self._pick_target_by_screen(event.pos)
        self._drag_moved = False
        self._drag_press_pos = self._safe_pos_tuple(event.pos)
        if key is None and self._side_selected_key is not None:
            key = self._side_selected_key
        if key is None:
            self._drag_target_key = None
            return
        self._drag_target_key = key
        self._drag_last_pos = self._safe_pos_tuple(event.pos)
        if hasattr(event, 'handled'):
            event.handled = True
        if hasattr(self, 'view') and hasattr(self.view, 'camera') and self.view.camera is not None:
            self.view.camera.interactive = False

    def _on_canvas_mouse_move(self, event):
        if self._drag_target_key is None:
            return
        cur_pos = self._safe_pos_tuple(event.pos)
        if self._drag_last_pos is None:
            return
        dx_px = cur_pos[0] - self._drag_last_pos[0]
        dy_px = cur_pos[1] - self._drag_last_pos[1]
        if abs(dx_px) + abs(dy_px) > 0.1:
            self._drag_moved = True
        dx, dy = self._pixel_to_world_delta(dx_px, dy_px)
        x, y = self.target_xy[self._drag_target_key]
        x = self._clamp(x + dx, self.TARGET_BOUNDS['x_min'], self.TARGET_BOUNDS['x_max'])
        y = self._clamp(y + dy, self.TARGET_BOUNDS['y_min'], self.TARGET_BOUNDS['y_max'])
        self.target_xy[self._drag_target_key] = [x, y]
        self._drag_last_pos = cur_pos
        self._update_target_nodes()
        if hasattr(event, 'handled'):
            event.handled = True

    def _on_canvas_mouse_release(self, event):
        key = self._drag_target_key
        if key is None or event.button != 1:
            return
        self._drag_target_key = None
        self._drag_last_pos = None
        if hasattr(self, 'view') and hasattr(self.view, 'camera') and self.view.camera is not None:
            self.view.camera.interactive = True
        if hasattr(event, 'handled'):
            event.handled = True
        x, y = self.target_xy.get(key, [0.0, 0.0])
        self.target_moved.emit(key, float(x), float(y))
        if not self._drag_moved:
            self.target_clicked.emit(key)

    @staticmethod
    def _parse_xyz_rpy(elem):
        if elem is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        xyz = [float(v) for v in elem.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in elem.get('rpy', '0 0 0').split()]
        if len(xyz) != 3:
            xyz = [0.0, 0.0, 0.0]
        if len(rpy) != 3:
            rpy = [0.0, 0.0, 0.0]
        return xyz, rpy

    @staticmethod
    def _axis_angle_matrix(axis, angle):
        axis = np.asarray(axis, dtype=np.float64)
        n = float(np.linalg.norm(axis))
        if n < 1e-9:
            return np.eye(3, dtype=np.float64)
        axis /= n
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c
        return np.array([
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ], dtype=np.float64)

    @staticmethod
    def _rpy_matrix(rpy):
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
        return rz @ ry @ rx

    @staticmethod
    def _make_transform(xyz, rpy):
        t = np.eye(4, dtype=np.float64)
        t[:3, :3] = ArmSimWidget._rpy_matrix(rpy)
        t[:3, 3] = np.asarray(xyz, dtype=np.float64)
        return t

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _pick_urdf_from_dir(self, dir_path):
        if not dir_path:
            return ''
        d = os.path.abspath(str(dir_path))
        if not os.path.isdir(d):
            return ''
        urdf_candidates = []
        direct = os.path.join(d, 'urdf')
        if os.path.isdir(direct):
            for name in sorted(os.listdir(direct)):
                if name.lower().endswith('.urdf'):
                    urdf_candidates.append(os.path.join(direct, name))
        if not urdf_candidates:
            for name in sorted(os.listdir(d)):
                if name.lower().endswith('.urdf'):
                    urdf_candidates.append(os.path.join(d, name))
        return urdf_candidates[0] if urdf_candidates else ''

    def _expand_urdf_source(self, source_text):
        src = str(source_text or '').strip()
        if not src:
            return ''
        if os.path.isfile(src):
            return src
        if os.path.isdir(src):
            return self._pick_urdf_from_dir(src)
        return ''

    def set_scene_urdf_source(self, source_text):
        src = str(source_text or '').strip()
        if src == self.scene_urdf_source:
            return
        self.scene_urdf_source = src
        self._load_urdf_model()
        self._apply_state_to_model(self.state)
        if self.canvas is not None:
            self.canvas.update()

    def _clear_loaded_mesh_nodes(self):
        for node in self.mesh_nodes:
            vis = node.get('visual')
            if vis is None:
                continue
            try:
                vis.parent = None
            except Exception:
                pass
        self.mesh_nodes = []

    def _resolve_urdf_path(self):
        # Prefer legacy articulated URDF chain to keep joint animation stable.
        for path in self.URDF_CANDIDATES:
            if os.path.isfile(path):
                return path

        env_path = os.environ.get(self.URDF_ENV, '').strip()
        env_resolved = self._expand_urdf_source(env_path)
        if env_resolved:
            return env_resolved

        scene_path = self._expand_urdf_source(self.scene_urdf_source)
        if scene_path:
            return scene_path
        return ''

    def _resolve_mesh_path(self, mesh_filename, urdf_path):
        if not mesh_filename:
            return ''
        if os.path.isfile(mesh_filename):
            return mesh_filename

        filename = mesh_filename
        rel_hint = ''
        if filename.startswith('package://'):
            pkg_str = filename[len('package://'):]
            parts = pkg_str.split('/', 1)
            rel_hint = parts[1] if len(parts) > 1 else ''
            filename = os.path.basename(filename)
        mesh_base = os.path.basename(filename)
        urdf_dir = os.path.dirname(urdf_path)
        pkg_dir = os.path.dirname(urdf_dir)

        candidates = []
        mesh_env = os.environ.get(self.MESH_ENV, '').strip()
        if mesh_env:
            candidates.append(mesh_env)
        candidates.extend([
            os.path.join(pkg_dir, 'meshes'),
            os.path.join(urdf_dir, '..', 'meshes'),
            '/home/ubuntu/ros2_ws/src/simulations/nexarm_description/meshes',
        ])

        for cdir in candidates:
            cdir = os.path.abspath(cdir)
            if rel_hint:
                p = os.path.join(cdir, rel_hint)
                if os.path.isfile(p):
                    return p
            p = os.path.join(cdir, mesh_base)
            if os.path.isfile(p):
                return p
        return ''

    def _load_stl_mesh(self, path):
        with open(path, 'rb') as f:
            header = f.read(80)
            if len(header) == 80:
                cnt_bytes = f.read(4)
                if len(cnt_bytes) == 4:
                    tri_count = struct.unpack('<I', cnt_bytes)[0]
                    payload = f.read(tri_count * 50)
                    if len(payload) == tri_count * 50:
                        raw = np.frombuffer(payload, dtype=np.uint8).reshape(tri_count, 50)
                        vb = raw[:, 12:48].copy()
                        verts = vb.view('<f4').reshape(tri_count, 3, 3).reshape(-1, 3)
                        return self._decimate_triangle_stream(verts.astype(np.float32))

        verts = []
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if line.startswith('vertex'):
                    parts = line.split()
                    if len(parts) >= 4:
                        verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        if len(verts) < 3:
            raise RuntimeError(f'empty STL: {path}')
        v = np.asarray(verts, dtype=np.float32)
        n = (len(v) // 3) * 3
        if n < 3:
            raise RuntimeError(f'invalid STL: {path}')
        v = v[:n]
        return self._decimate_triangle_stream(v)

    def _decimate_triangle_stream(self, verts):
        tri = verts.reshape(-1, 3, 3)
        tri_count = tri.shape[0]
        max_tri = self.max_triangles_per_mesh
        # For exported monolithic CAD meshes, keep interaction responsive by default.
        if self._auto_decimate_enabled and max_tri <= 0 and tri_count > 120000:
            max_tri = 35000
        if max_tri > 0 and tri_count > max_tri:
            step = int(math.ceil(tri_count / float(max_tri)))
            tri = tri[::step]
        out_v = tri.reshape(-1, 3).astype(np.float32)
        out_f = np.arange(out_v.shape[0], dtype=np.uint32).reshape(-1, 3)
        return out_v, out_f

    @staticmethod
    def _parse_mesh_scale(mesh_elem):
        if mesh_elem is None:
            return np.array([1.0, 1.0, 1.0], dtype=np.float32)
        txt = str(mesh_elem.get('scale', '') or '').strip()
        if not txt:
            return np.array([1.0, 1.0, 1.0], dtype=np.float32)
        try:
            vals = [float(v) for v in txt.replace(',', ' ').split() if v.strip()]
            if len(vals) == 1:
                s = float(vals[0])
                return np.array([s, s, s], dtype=np.float32)
            if len(vals) >= 3:
                return np.array([float(vals[0]), float(vals[1]), float(vals[2])], dtype=np.float32)
        except Exception:
            pass
        return np.array([1.0, 1.0, 1.0], dtype=np.float32)

    def _parse_material_color(self, visual_elem):
        return self.MODEL_RGBA

    def _collect_link_visuals(self, root, urdf_path):
        link_visuals = {}
        link_names = set()
        for link_elem in root.findall('link'):
            lname = link_elem.get('name', '')
            if not lname:
                continue
            link_names.add(lname)
            visuals = []
            for visual_elem in link_elem.findall('visual'):
                origin = visual_elem.find('origin')
                v_xyz, v_rpy = self._parse_xyz_rpy(origin)
                mesh_elem = visual_elem.find('./geometry/mesh')
                if mesh_elem is None:
                    continue
                mesh_filename = mesh_elem.get('filename', '')
                mesh_path = self._resolve_mesh_path(mesh_filename, urdf_path)
                if not mesh_path:
                    continue
                mesh_scale = self._parse_mesh_scale(mesh_elem)
                color = self._parse_material_color(visual_elem)
                visuals.append({
                    'mesh_path': mesh_path,
                    'mesh_scale': mesh_scale,
                    'origin_xyz': v_xyz,
                    'origin_rpy': v_rpy,
                    'color': color,
                })
            link_visuals[lname] = visuals
        return link_visuals, link_names

    def _load_urdf_model(self):
        if self.canvas is None:
            return
        self._clear_loaded_mesh_nodes()
        self.model_loaded = False
        self.load_error = ''
        self.loaded_urdf_path = ''
        urdf_path = self._resolve_urdf_path()
        if not urdf_path:
            src_hint = self.scene_urdf_source if self.scene_urdf_source else '(default candidates)'
            self.load_error = f'URDF 文件未找到: {src_hint}'
            self.status_label.setText(f'URDF: {self.load_error}')
            return

        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            self._auto_decimate_enabled = ('_urdf_export' in urdf_path)

            self.material_colors = {}
            for m in root.findall('material'):
                mname = m.get('name', '').strip()
                c = m.find('color')
                if not mname or c is None:
                    continue
                rgba = [float(v) for v in c.get('rgba', '0.78 0.82 0.93 1').split()]
                if len(rgba) == 3:
                    rgba.append(1.0)
                if len(rgba) == 4:
                    self.material_colors[mname] = tuple(rgba)

            self.link_visuals, link_names = self._collect_link_visuals(root, urdf_path)
            all_verts = []

            self.joints = {}
            self.child_joints = {}
            self.parent_joint = {}
            for joint_elem in root.findall('joint'):
                jname = joint_elem.get('name', '')
                if not jname:
                    continue
                jtype = joint_elem.get('type', 'fixed')
                parent_elem = joint_elem.find('parent')
                child_elem = joint_elem.find('child')
                if parent_elem is None or child_elem is None:
                    continue
                parent = parent_elem.get('link', '')
                child = child_elem.get('link', '')
                if not parent or not child:
                    continue
                origin = joint_elem.find('origin')
                j_xyz, j_rpy = self._parse_xyz_rpy(origin)
                axis_elem = joint_elem.find('axis')
                axis = [0.0, 0.0, 1.0]
                if axis_elem is not None:
                    axis = [float(v) for v in axis_elem.get('xyz', '0 0 1').split()]
                    if len(axis) != 3:
                        axis = [0.0, 0.0, 1.0]
                self.joints[jname] = {
                    'name': jname,
                    'type': jtype,
                    'parent': parent,
                    'child': child,
                    'axis': np.asarray(axis, dtype=np.float64),
                    'origin_T': self._make_transform(j_xyz, j_rpy),
                }
                self.child_joints.setdefault(parent, []).append(jname)
                self.parent_joint[child] = jname

            roots = [name for name in link_names if name not in self.parent_joint]
            if 'base_link' in roots:
                self.root_link = 'base_link'
            elif roots:
                self.root_link = roots[0]

            loaded = 0
            self.mesh_nodes = []
            self.total_triangles_loaded = 0
            # Keep model clearly visible; exported single-mesh URDF also uses regular flat shading.
            shading_mode = self.mesh_shading
            for link_name, visuals in self.link_visuals.items():
                for vis in visuals:
                    verts, faces = self._load_stl_mesh(vis['mesh_path'])
                    mscale = np.asarray(vis.get('mesh_scale', [1.0, 1.0, 1.0]), dtype=np.float32).reshape(1, 3)
                    verts = (verts.astype(np.float32) * mscale).astype(np.float32, copy=False)
                    visual = vscene.visuals.Mesh(
                        vertices=verts,
                        faces=faces,
                        color=vis['color'],
                        shading=shading_mode,
                        parent=self.view.scene,
                    )
                    visual.set_gl_state('opaque', depth_test=True, cull_face=False, blend=False)
                    visual.transform = MatrixTransform()
                    vis_T = self._make_transform(vis['origin_xyz'], vis['origin_rpy'])
                    self.mesh_nodes.append({
                        'link': link_name,
                        'visual': visual,
                        'vis_T': vis_T,
                    })
                    loaded += 1
                    self.total_triangles_loaded += int(faces.shape[0])

                    r = vis_T[:3, :3]
                    t = vis_T[:3, 3]
                    v_local = (verts.astype(np.float64) @ r.T) + t
                    all_verts.append(v_local)

            if loaded == 0:
                self.load_error = '未从 URDF 加载到任何 STL mesh'
                self.status_label.setText(f'URDF: {self.load_error}')
                return

            if all_verts:
                vv = np.concatenate(all_verts, axis=0)
                vmin = vv.min(axis=0)
                vmax = vv.max(axis=0)
                center = (vmin + vmax) / 2.0
                extent = max(float(np.max(vmax - vmin)), 0.22)
                self.view.camera.center = tuple(center.tolist())
                self.view.camera.distance = max(0.45, extent * 3.2)

            self.model_loaded = True
            self.load_error = ''
            self.loaded_urdf_path = urdf_path
            shading_name = shading_mode if shading_mode is not None else 'none'
            self.status_label.setText(
                f'URDF: {os.path.basename(urdf_path)} 已加载 ({loaded} meshes, {self.total_triangles_loaded} tris, shading={shading_name})'
            )
        except Exception as e:
            self.model_loaded = False
            self.load_error = str(e)
            self.status_label.setText(f'URDF加载失败: {e}')

    def _state_to_joint_map(self, state):
        joint_map = {}
        servos = [float(v) for v in state.get('servos', [])]
        if len(servos) >= 6:
            for i, jname in enumerate(self.JOINT_FEED_ORDER):
                deg = (servos[i] - self.SERVO_CENTER[i]) * self.SERVO_RATIO[i]
                deg = deg * self.SERVO_DIR[i] * self.URDF_DIR[i]
                joint_map[jname] = math.radians(deg)
        else:
            angles = [float(v) for v in state.get('joint_angles', [])]
            if angles:
                max_abs = max(abs(v) for v in angles)
                if max_abs > (2.0 * math.pi + 0.5):
                    angles = [math.radians(v) for v in angles]
                for i, jname in enumerate(self.JOINT_FEED_ORDER):
                    if i < len(angles):
                        joint_map[jname] = angles[i]

        claw = float(state.get('claw', -60.0))
        open_ratio = self._clamp((30.0 - claw) / 90.0, 0.0, 1.0)
        jaw_shift = 0.010 * open_ratio
        joint_map.setdefault('left_jaw_joint', -jaw_shift)
        joint_map.setdefault('right_jaw_joint', jaw_shift)
        return joint_map

    def _joint_motion_matrix(self, joint, value):
        jtype = joint['type']
        axis = joint['axis']
        motion = np.eye(4, dtype=np.float64)
        if jtype in ('revolute', 'continuous'):
            motion[:3, :3] = self._axis_angle_matrix(axis, value)
        elif jtype == 'prismatic':
            motion[:3, 3] = axis * value
        return motion

    def _compute_link_transforms(self, state):
        link_tf = {self.root_link: np.eye(4, dtype=np.float64)}
        joint_map = self._state_to_joint_map(state)
        pending = [self.root_link]
        while pending:
            parent = pending.pop(0)
            parent_T = link_tf[parent]
            for jname in self.child_joints.get(parent, []):
                joint = self.joints.get(jname)
                if joint is None:
                    continue
                q = joint_map.get(jname, 0.0)
                child_T = parent_T @ joint['origin_T'] @ self._joint_motion_matrix(joint, q)
                link_tf[joint['child']] = child_T
                pending.append(joint['child'])
        return link_tf

    def _apply_state_to_model(self, state):
        if self.canvas is None:
            return
        if not self.model_loaded:
            return
        link_tf = self._compute_link_transforms(state)
        for node in self.mesh_nodes:
            lnk = node['link']
            base_T = link_tf.get(lnk, np.eye(4, dtype=np.float64))
            world_T = base_T @ node['vis_T']
            row_major = world_T.T.astype(np.float32)
            tr = node['visual'].transform
            if not isinstance(tr, MatrixTransform):
                tr = MatrixTransform()
                node['visual'].transform = tr
            tr.matrix = row_major

    def set_state(self, state):
        self.state.update(state)
        self._apply_state_to_model(self.state)

    def zoom_camera(self, zoom_in=True):
        if not hasattr(self, 'view') or self.view is None or self.view.camera is None:
            return
        cam = self.view.camera
        dist = float(getattr(cam, 'distance', 0.7) or 0.7)
        if zoom_in:
            dist *= 0.86
        else:
            dist *= 1.16
        cam.distance = max(0.18, min(3.2, dist))

    def reset_camera_view(self):
        if not hasattr(self, 'view') or self.view is None or self.view.camera is None:
            return
        cam = self.view.camera
        cam.fov = 45.0
        cam.distance = 0.7
        cam.elevation = 22.0
        cam.azimuth = 35.0


class ColorPlacementWidget(QWidget):
    block_clicked = pyqtSignal(str)
    block_moved = pyqtSignal(str, float, float)
    GRID_CELLS = 3
    MARGIN = 10
    COLOR_MAP = {
        'red': QColor(239, 68, 68),
        'green': QColor(34, 197, 94),
        'blue': QColor(59, 130, 246),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 190)
        self._pos_norm = {
            'red': QPointF(0.18, 0.18),
            'green': QPointF(0.50, 0.50),
            'blue': QPointF(0.82, 0.18),
        }
        self._drag_key = None
        self._active_key = None
        self._drag_offset = QPointF(0.0, 0.0)
        self._press_point = QPointF(0.0, 0.0)
        self._moved = False
        self.world_x_min = -0.05
        self.world_x_max = 0.10
        self.world_y_min = 0.05
        self.world_y_max = 0.20

    def _grid_rect(self):
        w = max(40, self.width() - self.MARGIN * 2)
        h = max(40, self.height() - self.MARGIN * 2)
        side = float(min(w, h))
        x = (self.width() - side) * 0.5
        y = (self.height() - side) * 0.5
        return QRectF(x, y, side, side)

    def _block_size(self, grid_rect):
        return max(18.0, grid_rect.width() / (self.GRID_CELLS + 1.2))

    def _norm_to_point(self, norm_pos, grid_rect):
        x = grid_rect.left() + norm_pos.x() * grid_rect.width()
        y = grid_rect.top() + norm_pos.y() * grid_rect.height()
        return QPointF(x, y)

    @staticmethod
    def _clamp01(v):
        return max(0.0, min(1.0, float(v)))

    def _point_to_norm(self, pt, grid_rect):
        nx = (pt.x() - grid_rect.left()) / max(grid_rect.width(), 1e-6)
        ny = (pt.y() - grid_rect.top()) / max(grid_rect.height(), 1e-6)
        return QPointF(self._clamp01(nx), self._clamp01(ny))

    def _hit_test(self, pos):
        grid = self._grid_rect()
        bs = self._block_size(grid)
        for key in ('blue', 'green', 'red'):
            center = self._norm_to_point(self._pos_norm[key], grid)
            rect = QRectF(center.x() - bs * 0.5, center.y() - bs * 0.5, bs, bs)
            if rect.contains(pos):
                return key
        return None

    def _norm_to_world(self, norm_pos):
        x = self.world_x_min + norm_pos.x() * (self.world_x_max - self.world_x_min)
        y = self.world_y_min + norm_pos.y() * (self.world_y_max - self.world_y_min)
        return float(x), float(y)

    def _world_to_norm(self, x, y):
        sx = max(1e-6, self.world_x_max - self.world_x_min)
        sy = max(1e-6, self.world_y_max - self.world_y_min)
        nx = (float(x) - self.world_x_min) / sx
        ny = (float(y) - self.world_y_min) / sy
        return QPointF(self._clamp01(nx), self._clamp01(ny))

    def set_scene_targets(self, place_targets):
        xs = []
        ys = []
        for key in ('red', 'green', 'blue'):
            p = place_targets.get(key, DEFAULT_PLACE_TARGETS[key])
            xs.append(float(p[0]))
            ys.append(float(p[1]))
        x_min = min(xs) - 0.03
        x_max = max(xs) + 0.03
        y_min = min(ys) - 0.03
        y_max = max(ys) + 0.03
        span_x = max(0.09, x_max - x_min)
        span_y = max(0.09, y_max - y_min)
        cx = (x_min + x_max) * 0.5
        cy = (y_min + y_max) * 0.5
        self.world_x_min = cx - span_x * 0.5
        self.world_x_max = cx + span_x * 0.5
        self.world_y_min = cy - span_y * 0.5
        self.world_y_max = cy + span_y * 0.5
        for key in ('red', 'green', 'blue'):
            p = place_targets.get(key, DEFAULT_PLACE_TARGETS[key])
            self._pos_norm[key] = self._world_to_norm(p[0], p[1])
        self.update()

    def set_active_key(self, key_or_none):
        key = key_or_none if key_or_none in ('red', 'green', 'blue') else None
        if key == self._active_key:
            return
        self._active_key = key
        self.update()

    def paintEvent(self, _event):
        from PyQt5.QtGui import QPainter
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        grid = self._grid_rect()
        p.setPen(QPen(QColor(148, 163, 184), 1))
        p.setBrush(QBrush(QColor(241, 245, 249)))
        p.drawRoundedRect(grid, 8, 8)

        cell = grid.width() / self.GRID_CELLS
        p.setPen(QPen(QColor(148, 163, 184), 1))
        for i in range(1, self.GRID_CELLS):
            x = grid.left() + i * cell
            y = grid.top() + i * cell
            p.drawLine(int(x), int(grid.top()), int(x), int(grid.bottom()))
            p.drawLine(int(grid.left()), int(y), int(grid.right()), int(y))

        bs = self._block_size(grid)
        for key in ('red', 'green', 'blue'):
            center = self._norm_to_point(self._pos_norm[key], grid)
            rect = QRectF(center.x() - bs * 0.5, center.y() - bs * 0.5, bs, bs)
            p.setPen(QPen(QColor(30, 41, 59), 1))
            p.setBrush(QBrush(self.COLOR_MAP[key]))
            p.drawRoundedRect(rect, 5, 5)
            if key == self._active_key:
                p.setPen(QPen(QColor(15, 23, 42), 2))
                p.setBrush(Qt.NoBrush)
                p.drawRoundedRect(rect.adjusted(-2, -2, 2, 2), 6, 6)
        p.end()

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        key = self._hit_test(event.pos())
        if key is None:
            grid = self._grid_rect()
            if self._active_key is not None and grid.contains(QPointF(event.pos())):
                self._pos_norm[self._active_key] = self._point_to_norm(QPointF(event.pos()), grid)
                x, y = self._norm_to_world(self._pos_norm[self._active_key])
                self.block_moved.emit(self._active_key, x, y)
                self.update()
            self._drag_key = None
            return
        grid = self._grid_rect()
        center = self._norm_to_point(self._pos_norm[key], grid)
        self._drag_key = key
        self._drag_offset = QPointF(event.pos()) - center
        self._press_point = QPointF(event.pos())
        self._moved = False

    def mouseMoveEvent(self, event):
        if self._drag_key is None:
            return
        if (QPointF(event.pos()) - self._press_point).manhattanLength() > 3.0:
            self._moved = True
        grid = self._grid_rect()
        center = QPointF(event.pos()) - self._drag_offset
        self._pos_norm[self._drag_key] = self._point_to_norm(center, grid)
        self.update()

    def mouseReleaseEvent(self, _event):
        if self._drag_key is None:
            return
        key = self._drag_key
        self._drag_key = None
        x, y = self._norm_to_world(self._pos_norm[key])
        self.block_moved.emit(key, x, y)
        if not self._moved:
            self.block_clicked.emit(key)
        self.update()


class MainWindow(QMainWindow):
    task_status_signal = pyqtSignal(str)
    task_highlight_signal = pyqtSignal(int)
    task_feedback_signal = pyqtSignal(str)
    result_image_signal = pyqtSignal(object)
    rgb_image_signal = pyqtSignal(object)
    depth_image_signal = pyqtSignal(object)
    preview_status_signal = pyqtSignal(str)
    arm_state_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.node = None
        threading.Thread(target=self.ros_node, daemon=True).start()
        while not init_finish:
            time.sleep(0.1)

        self.params = self.load_calibration_params()
        self.scene_cfg = self.load_scene_cfg()
        self.row_widgets = {}

        self.task_queue = []
        self.task_worker_running = False
        self.task_worker_stop = False
        self.manual_pose = dict(DEFAULT_HOME)
        self.last_preview_images = {'result': None, 'rgb': None, 'depth': None}
        self._preview_decode_warned = set()
        self._last_arm_ui_update_ts = 0.0
        try:
            self._arm_ui_interval = float(os.environ.get('SCENE_3D_UPDATE_INTERVAL', '0.08'))
        except Exception:
            self._arm_ui_interval = 0.08
        self._arm_ui_interval = max(0.03, min(self._arm_ui_interval, 0.20))
        self._selected_color_key = None
        self.place_map_scale_x = 1.0
        self.place_map_scale_y = 1.0
        self.place_map_offset_x = 0.0
        self.place_map_offset_y = 0.0
        self._load_place_map_params()
        self.place_exec_comp_enabled = str(os.environ.get('SCENE_PLACE_EXEC_COMP', '1')).strip().lower() not in ('0', 'false', 'no')
        self.latest_arm_state = {
            'x': float(DEFAULT_HOME['x']),
            'y': float(DEFAULT_HOME['y']),
            'z': float(DEFAULT_HOME['z']),
            'pitch': float(DEFAULT_HOME['pitch']),
            'roll': float(DEFAULT_HOME['roll']),
            'claw': float(DEFAULT_HOME['claw']),
            'yaw': 0.0,
            'joint_angles': [0.0] * 6,
            'servos': [0] * 6,
        }
        self.arm_3d_widget = None
        self._closing = False

        self.setup_ui()
        self.task_status_signal.connect(self._set_task_status_ui)
        self.task_highlight_signal.connect(self._highlight_task_row_ui)
        self.task_feedback_signal.connect(self._append_feedback_ui)
        self.result_image_signal.connect(self._update_result_preview)
        self.rgb_image_signal.connect(self._update_rgb_preview)
        self.depth_image_signal.connect(self._update_depth_preview)
        self.preview_status_signal.connect(self._set_preview_status_ui)
        self.arm_state_signal.connect(self._on_arm_state_ui)

        self.node.set_preview_callbacks(
            on_result=None,
            on_rgb=self._on_rgb_image_msg,
            on_depth=None,
            on_status=self._on_preview_status_msg,
        )
        self.node.set_arm_state_callback(self._on_arm_state_msg)
        self.load_scene_combo()

    def ros_node(self):
        self.node = ArmControlNode('scene_place_custom')
        rclpy.spin(self.node)
        self.node.destroy_node()

    def default_scene(self, scene_name='Scene 1'):
        return {
            'name': scene_name,
            'length_m': 0.158,
            'width_m': 0.175,
            'urdf_source': '',
            'home_pose': dict(DEFAULT_HOME),
            'place_policy': dict(DEFAULT_PLACE_POLICY),
            'place_targets': {k: list(v) for k, v in DEFAULT_PLACE_TARGETS.items()},
        }

    def _guess_scene_urdf_source(self, scene_id):
        sid = str(scene_id or '').strip()
        if not sid:
            return ''
        idx = None
        if sid.startswith('scene_'):
            tail = sid.split('_', 1)[1]
            try:
                idx = max(int(tail) - 1, 0)
            except Exception:
                idx = None
        if idx is None:
            if sid == DEFAULT_SCENE_ID:
                idx = 0
            else:
                return ''
        return os.path.join(ArmSimWidget.SCENE_URDF_BASE, f'{idx:02d}_urdf_export')

    def _scene_urdf_source(self, scene_id, scene_cfg):
        scene = scene_cfg if isinstance(scene_cfg, dict) else {}
        for key in ('urdf_source', 'urdf_dir', 'urdf_path'):
            v = scene.get(key)
            if isinstance(v, str) and v.strip():
                cand = v.strip()
                if os.path.isdir(cand) or os.path.isfile(cand):
                    return cand
        return self._guess_scene_urdf_source(scene_id)

    def _sync_urdf_from_scene(self):
        if not hasattr(self, 'arm_3d_widget') or self.arm_3d_widget is None:
            return
        sid = self.current_scene_id()
        scene = self.current_scene()
        src = self._scene_urdf_source(sid, scene)
        self.arm_3d_widget.set_scene_urdf_source(src)
        if hasattr(self, 'lb_sim_status') and self.lb_sim_status is not None:
            if src:
                self.lb_sim_status.setText(f'内嵌仿真: 场景{sid} URDF -> {src}')
            else:
                self.lb_sim_status.setText('内嵌仿真: 使用默认URDF')

    def normalize_scene_cfg(self, cfg):
        scenes = cfg.get('scenes')
        if not isinstance(scenes, dict) or not scenes:
            cfg['scenes'] = {DEFAULT_SCENE_ID: self.default_scene('Scene 1')}
            cfg['current_scene'] = DEFAULT_SCENE_ID
            scenes = cfg['scenes']
        current = cfg.get('current_scene')
        if current not in scenes:
            cfg['current_scene'] = next(iter(scenes.keys()))

        for sid, scene in scenes.items():
            if not isinstance(scene, dict):
                scenes[sid] = self.default_scene(f'Scene {sid}')
                scene = scenes[sid]
            scene.setdefault('name', sid)
            scene.setdefault('length_m', 0.158)
            scene.setdefault('width_m', 0.175)
            scene.setdefault('urdf_source', '')
            if not str(scene.get('urdf_source', '')).strip():
                scene['urdf_source'] = self._guess_scene_urdf_source(sid)
            if not isinstance(scene.get('home_pose'), dict):
                scene['home_pose'] = dict(DEFAULT_HOME)
            for k, v in DEFAULT_HOME.items():
                scene['home_pose'].setdefault(k, v)
            if sid == DEFAULT_SCENE_ID:
                scene['home_pose']['x'] = 110.0

            if not isinstance(scene.get('place_policy'), dict):
                scene['place_policy'] = {}
            scene['place_policy'].setdefault('only_left_y_positive', bool(sid == DEFAULT_SCENE_ID))
            scene['place_policy'].setdefault('min_place_z', DEFAULT_PLACE_POLICY['min_place_z'])

            if not isinstance(scene.get('place_targets'), dict):
                scene['place_targets'] = {}
            for key, value in DEFAULT_PLACE_TARGETS.items():
                pv = scene['place_targets'].get(key)
                if not isinstance(pv, list) or len(pv) != 3:
                    scene['place_targets'][key] = list(value)

    def load_calibration_params(self):
        with open(CALIB_YAML_PATH, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def load_scene_cfg(self):
        if not os.path.exists(SCENE_YAML_PATH):
            cfg = {'current_scene': DEFAULT_SCENE_ID, 'scenes': {DEFAULT_SCENE_ID: self.default_scene()}}
            self.normalize_scene_cfg(cfg)
            return cfg
        with open(SCENE_YAML_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        self.normalize_scene_cfg(cfg)
        return cfg

    def _load_place_map_params(self):
        sx = float(self.place_map_scale_x)
        sy = float(self.place_map_scale_y)
        ox = float(self.place_map_offset_x)
        oy = float(self.place_map_offset_y)
        source = 'default'

        def _to_float(value, default):
            try:
                return float(value)
            except Exception:
                return default

        def _apply_map(cfg, src_name):
            nonlocal sx, sy, ox, oy, source
            if not isinstance(cfg, dict):
                return False
            changed = False
            scale = cfg.get('scale')
            if isinstance(scale, (list, tuple)) and len(scale) >= 2:
                sx = _to_float(scale[0], sx)
                sy = _to_float(scale[1], sy)
                changed = True
            offset = cfg.get('offset')
            if isinstance(offset, (list, tuple)) and len(offset) >= 2:
                ox = _to_float(offset[0], ox)
                oy = _to_float(offset[1], oy)
                changed = True
            key_map = (
                ('scale_x', 'sx', 'x_scale'),
                ('scale_y', 'sy', 'y_scale'),
                ('offset_x', 'ox', 'x_offset', 'bias_x'),
                ('offset_y', 'oy', 'y_offset', 'bias_y'),
            )
            vals = [sx, sy, ox, oy]
            for idx, aliases in enumerate(key_map):
                for key in aliases:
                    if key in cfg:
                        val = cfg.get(key)
                        if val is None:
                            continue
                        if isinstance(val, str) and val.strip() == '':
                            continue
                        vals[idx] = _to_float(val, vals[idx])
                        changed = True
                        break
            sx, sy, ox, oy = vals
            if changed:
                source = src_name
            return changed

        calib = self.params if isinstance(self.params, dict) else {}
        if _apply_map(calib.get('place_map'), 'calibration.yaml: place_map'):
            pass
        else:
            pixel = calib.get('pixel')
            if isinstance(pixel, dict):
                pixel_scale = pixel.get('scale')
                if isinstance(pixel_scale, (list, tuple)) and len(pixel_scale) >= 2:
                    sx = _to_float(pixel_scale[0], sx)
                    sy = _to_float(pixel_scale[1], sy)
                    source = 'calibration.yaml: pixel.scale'
                pixel_offset = pixel.get('offset')
                if isinstance(pixel_offset, (list, tuple)) and len(pixel_offset) >= 2:
                    ox = _to_float(pixel_offset[0], ox)
                    oy = _to_float(pixel_offset[1], oy)
                    source = 'calibration.yaml: pixel.offset'

        if isinstance(self.scene_cfg, dict):
            _apply_map(self.scene_cfg.get('place_map'), 'calibration_scene.yaml: root place_map')
            sid = self.scene_cfg.get('current_scene', DEFAULT_SCENE_ID)
            scene = self.scene_cfg.get('scenes', {}).get(sid, {}) if sid else {}
            _apply_map(scene.get('place_map'), f'calibration_scene.yaml: scenes.{sid}.place_map')
            policy = scene.get('place_policy') if isinstance(scene, dict) else None
            if isinstance(policy, dict):
                _apply_map(policy.get('place_map'), f'calibration_scene.yaml: scenes.{sid}.place_policy.place_map')
                policy_map = {}
                for src_key, dst_key in (
                    ('map_scale_x', 'scale_x'),
                    ('map_scale_y', 'scale_y'),
                    ('map_offset_x', 'offset_x'),
                    ('map_offset_y', 'offset_y'),
                ):
                    v = policy.get(src_key)
                    if v is not None:
                        policy_map[dst_key] = v
                _apply_map(policy_map, f'calibration_scene.yaml: scenes.{sid}.place_policy.map_*')

        env_map = {
            'SCENE_PLACE_MAP_SCALE_X': 'scale_x',
            'SCENE_PLACE_MAP_SCALE_Y': 'scale_y',
            'SCENE_PLACE_MAP_OFFSET_X': 'offset_x',
            'SCENE_PLACE_MAP_OFFSET_Y': 'offset_y',
        }
        env_cfg = {}
        for env_key, cfg_key in env_map.items():
            env_val = os.environ.get(env_key, '').strip()
            if env_val != '':
                env_cfg[cfg_key] = env_val
        if env_cfg:
            _apply_map(env_cfg, 'environment: SCENE_PLACE_MAP_*')

        if abs(sx) < 1e-9:
            sx = 1.0
        if abs(sy) < 1e-9:
            sy = 1.0
        self.place_map_scale_x = float(sx)
        self.place_map_scale_y = float(sy)
        self.place_map_offset_x = float(ox)
        self.place_map_offset_y = float(oy)
        msg = (
            '[PLACE_MAP] source=%s scale=(%.6f, %.6f) offset=(%.6f, %.6f)'
            % (source, self.place_map_scale_x, self.place_map_scale_y, self.place_map_offset_x, self.place_map_offset_y)
        )
        try:
            if self.node is not None:
                self.node.get_logger().info(msg)
            else:
                print(msg)
        except Exception:
            print(msg)

    def save_scene_cfg(self):
        self.normalize_scene_cfg(self.scene_cfg)
        os.makedirs(os.path.dirname(SCENE_YAML_PATH), exist_ok=True)
        with open(SCENE_YAML_PATH, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.scene_cfg, f, sort_keys=False, allow_unicode=True)

    def setup_ui(self):
        self.setWindowTitle('场景放置点自定义上位机')
        target_w, target_h = 800, 480
        size_env = os.environ.get('SCENE_UI_SIZE', '').strip().lower()
        if size_env:
            sep = 'x' if 'x' in size_env else ','
            try:
                sw, sh = size_env.split(sep, 1)
                target_w = max(640, int(sw))
                target_h = max(400, int(sh))
            except Exception:
                target_w, target_h = 800, 480
        else:
            app = QApplication.instance()
            screen = app.primaryScreen() if app is not None else None
            if screen is not None:
                geo = screen.availableGeometry()
                sw = int(geo.width())
                sh = int(geo.height())
                # 超宽屏/双屏（如 3840x1080）默认放大到更适合查看的尺寸
                if sw >= 3000 and sh >= 1000:
                    target_w = min(1920, sw)
                    target_h = min(980, sh)
        self.resize(target_w, target_h)
        self.setMinimumSize(800, 480)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background-color: #ECEFF1; color: #263238; }
            QGroupBox {
                background-color: #F5F7FA;
                border: 1px solid #CFD8DC;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { min-height: 24px; }
            QListWidget, QComboBox, QDoubleSpinBox {
                background-color: #FFFFFF;
                border: 1px solid #B0BEC5;
                border-radius: 4px;
            }
            """
        )
        root = QWidget(self)
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)

        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        page_dashboard = QWidget()
        page_dashboard_layout = QVBoxLayout(page_dashboard)
        page_scene = QWidget()
        page_scene_layout = QVBoxLayout(page_scene)
        page_task = QWidget()
        page_task_layout = QVBoxLayout(page_task)
        tabs.addTab(page_dashboard, '实时工作台')
        tabs.addTab(page_scene, '场景点位')
        tabs.addTab(page_task, '任务编排')
        self._build_dashboard_ui(page_dashboard_layout)

        scene_box = QGroupBox('场景')
        scene_layout = QGridLayout(scene_box)
        page_scene_layout.addWidget(scene_box)
        self.cb_scene = QComboBox()
        self.btn_add_scene = QPushButton('新增场景')
        self.btn_del_scene = QPushButton('删除场景')
        self.btn_save_scene = QPushButton('保存场景文件')
        scene_layout.addWidget(QLabel('场景ID'), 0, 0)
        scene_layout.addWidget(self.cb_scene, 0, 1)
        scene_layout.addWidget(self.btn_add_scene, 0, 2)
        scene_layout.addWidget(self.btn_del_scene, 0, 3)
        scene_layout.addWidget(self.btn_save_scene, 0, 4)

        self.chk_left_only = QCheckBox('scene 左侧限制: 所有放置点自动 y>=0')
        self.sp_min_z = QDoubleSpinBox()
        self.sp_min_z.setRange(0.000, 0.300)
        self.sp_min_z.setDecimals(3)
        self.sp_min_z.setSingleStep(0.001)
        scene_layout.addWidget(self.chk_left_only, 1, 0, 1, 3)
        scene_layout.addWidget(QLabel('最小放置Z(m)'), 1, 3)
        scene_layout.addWidget(self.sp_min_z, 1, 4)

        targets_box = QGroupBox('颜色/类别 -> 放置物理坐标')
        targets_layout = QGridLayout(targets_box)
        page_scene_layout.addWidget(targets_box)

        targets_layout.addWidget(QLabel('目标'), 0, 0)
        targets_layout.addWidget(QLabel('X(m)'), 0, 1)
        targets_layout.addWidget(QLabel('Y(m)'), 0, 2)
        targets_layout.addWidget(QLabel('Z(m)'), 0, 3)
        targets_layout.addWidget(QLabel('测试'), 0, 4)

        for row, (key, name) in enumerate(TARGET_ROWS, start=1):
            lb = QLabel(name)
            spx = QDoubleSpinBox(); spx.setRange(-0.500, 0.500); spx.setDecimals(3); spx.setSingleStep(0.001)
            spy = QDoubleSpinBox(); spy.setRange(-0.500, 0.500); spy.setDecimals(3); spy.setSingleStep(0.001)
            spz = QDoubleSpinBox(); spz.setRange(0.000, 0.300); spz.setDecimals(3); spz.setSingleStep(0.001)
            btn = QPushButton('移动到该点')
            btn.pressed.connect(lambda k=key: self.move_target(k))
            spx.valueChanged.connect(lambda v, k=key: self.on_target_changed(k, 0, v))
            spy.valueChanged.connect(lambda v, k=key: self.on_target_changed(k, 1, v))
            spz.valueChanged.connect(lambda v, k=key: self.on_target_changed(k, 2, v))
            targets_layout.addWidget(lb, row, 0)
            targets_layout.addWidget(spx, row, 1)
            targets_layout.addWidget(spy, row, 2)
            targets_layout.addWidget(spz, row, 3)
            targets_layout.addWidget(btn, row, 4)
            self.row_widgets[key] = (spx, spy, spz, btn)

        action_box = QGroupBox('场景动作')
        action_layout = QHBoxLayout(action_box)
        page_scene_layout.addWidget(action_box)
        self.btn_home = QPushButton('回默认位姿')
        self.btn_enter_calib = QPushButton('进入标定')
        self.btn_exit_calib = QPushButton('退出标定')
        self.lb_sim_status = QLabel('内嵌仿真: 等待状态流...')
        action_layout.addWidget(self.btn_home)
        action_layout.addWidget(self.btn_enter_calib)
        action_layout.addWidget(self.btn_exit_calib)
        action_layout.addWidget(self.lb_sim_status)

        task_box = QGroupBox('即用抓取任务')
        task_layout = QGridLayout(task_box)
        page_task_layout.addWidget(task_box)
        self.cb_task_type = QComboBox()
        self.cb_task_type.addItem('颜色识别(跟踪)', userData='color_track')
        self.cb_task_type.addItem('颜色分拣', userData='color')
        self.cb_task_type.addItem('垃圾分类', userData='waste')
        self.cb_task_type.addItem('标签分拣', userData='tag')
        self.cb_task_type.addItem('标签码垛', userData='stackup')
        self.cb_task_target = QComboBox()
        self.btn_task_add = QPushButton('添加到顺序')
        self.btn_task_remove = QPushButton('删除选中')
        self.btn_task_clear = QPushButton('清空顺序')
        self.btn_task_run = QPushButton('执行顺序')
        self.btn_task_stop = QPushButton('停止任务')
        self.btn_task_start_now = QPushButton('启动当前玩法')
        self.btn_task_stop_now = QPushButton('停止当前玩法')
        self.lb_task_queue = QListWidget()
        self.lb_task_status = QLabel('状态: 空闲')
        self.lb_task_feedback = QListWidget()

        task_layout.addWidget(QLabel('任务类型'), 0, 0)
        task_layout.addWidget(self.cb_task_type, 0, 1)
        task_layout.addWidget(QLabel('目标'), 0, 2)
        task_layout.addWidget(self.cb_task_target, 0, 3)
        task_layout.addWidget(self.btn_task_add, 0, 4)
        task_layout.addWidget(self.lb_task_queue, 1, 0, 3, 4)
        task_layout.addWidget(self.btn_task_remove, 1, 4)
        task_layout.addWidget(self.btn_task_clear, 2, 4)
        task_layout.addWidget(self.btn_task_run, 3, 4)
        task_layout.addWidget(self.btn_task_stop, 4, 4)
        task_layout.addWidget(self.btn_task_start_now, 5, 4)
        task_layout.addWidget(self.btn_task_stop_now, 6, 4)
        task_layout.addWidget(self.lb_task_status, 4, 0, 1, 4)
        task_layout.addWidget(QLabel('回传日志'), 5, 0, 1, 4)
        task_layout.addWidget(self.lb_task_feedback, 6, 0, 1, 4)

        self.cb_scene.currentIndexChanged.connect(self.on_scene_changed)
        self.btn_add_scene.pressed.connect(self.add_scene)
        self.btn_del_scene.pressed.connect(self.delete_scene)
        self.btn_save_scene.pressed.connect(self.save_scene_clicked)
        self.chk_left_only.toggled.connect(self.on_policy_changed)
        self.sp_min_z.valueChanged.connect(self.on_min_z_changed)
        self.btn_home.pressed.connect(self.go_home)
        self.btn_enter_calib.pressed.connect(lambda: self.node.enter_calibration())
        self.btn_exit_calib.pressed.connect(lambda: self.node.exit_calibration())

        self.cb_task_type.currentIndexChanged.connect(self.on_task_type_changed)
        self.btn_task_add.pressed.connect(self.add_task_item)
        self.btn_task_remove.pressed.connect(self.remove_task_item)
        self.btn_task_clear.pressed.connect(self.clear_task_items)
        self.btn_task_run.pressed.connect(self.start_task_queue)
        self.btn_task_stop.pressed.connect(self.stop_task_queue)
        self.btn_task_start_now.pressed.connect(self.start_current_task)
        self.btn_task_stop_now.pressed.connect(self.stop_task_queue)

        self.refresh_task_target_combo()

    def _build_dashboard_ui(self, page_layout):
        panel = QWidget()
        panel_layout = QHBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(8)
        page_layout.addWidget(panel, 1)

        left_box = QGroupBox('URDF 机械臂')
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(4, 8, 4, 4)
        self.arm_3d_widget = ArmSimWidget()
        self.arm_3d_widget.target_moved.connect(self._on_color_block_moved)
        self.arm_3d_widget.target_clicked.connect(self._on_color_block_clicked)
        left_layout.addWidget(self.arm_3d_widget, 1)
        panel_layout.addWidget(left_box, 4)

        right_panel = QWidget()
        right_panel.setFixedWidth(290)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        rgb_box = QGroupBox('RGB回传')
        rgb_layout = QVBoxLayout(rgb_box)
        self.lb_rgb_preview = QLabel()
        self._init_preview_label(self.lb_rgb_preview, RGB_IMAGE_TOPIC, 270, 190)
        rgb_layout.addWidget(self.lb_rgb_preview)
        right_layout.addWidget(rgb_box)

        place_box = QGroupBox('颜色源(拖到3D)')
        place_layout = QVBoxLayout(place_box)
        palette_row = QHBoxLayout()
        self.btn_pick_red = QPushButton('红')
        self.btn_pick_green = QPushButton('绿')
        self.btn_pick_blue = QPushButton('蓝')
        for btn in (self.btn_pick_red, self.btn_pick_green, self.btn_pick_blue):
            btn.setCheckable(True)
        self.btn_pick_red.setStyleSheet(
            'QPushButton{background:#fca5a5;color:#7f1d1d;font-weight:bold;}'
            'QPushButton:checked{background:#ef4444;color:#fff;}'
        )
        self.btn_pick_green.setStyleSheet(
            'QPushButton{background:#86efac;color:#14532d;font-weight:bold;}'
            'QPushButton:checked{background:#22c55e;color:#fff;}'
        )
        self.btn_pick_blue.setStyleSheet(
            'QPushButton{background:#93c5fd;color:#1e3a8a;font-weight:bold;}'
            'QPushButton:checked{background:#3b82f6;color:#fff;}'
        )
        self.btn_pick_red.pressed.connect(lambda: self._on_side_color_pick('red'))
        self.btn_pick_green.pressed.connect(lambda: self._on_side_color_pick('green'))
        self.btn_pick_blue.pressed.connect(lambda: self._on_side_color_pick('blue'))
        palette_row.addWidget(self.btn_pick_red)
        palette_row.addWidget(self.btn_pick_green)
        palette_row.addWidget(self.btn_pick_blue)
        place_layout.addLayout(palette_row)
        self.btn_view_mode = QPushButton('切到视角模式')
        self.btn_view_mode.setCheckable(True)
        self.btn_view_mode.toggled.connect(self._on_toggle_view_mode)
        place_layout.addWidget(self.btn_view_mode)
        zoom_row = QHBoxLayout()
        self.btn_zoom_in = QPushButton('放大+')
        self.btn_zoom_out = QPushButton('缩小-')
        self.btn_view_reset = QPushButton('视角复位')
        self.btn_zoom_in.pressed.connect(lambda: self._on_zoom_view(True))
        self.btn_zoom_out.pressed.connect(lambda: self._on_zoom_view(False))
        self.btn_view_reset.pressed.connect(self._on_reset_view)
        zoom_row.addWidget(self.btn_zoom_in)
        zoom_row.addWidget(self.btn_zoom_out)
        zoom_row.addWidget(self.btn_view_reset)
        place_layout.addLayout(zoom_row)
        hint = QLabel('先点颜色按钮，再在3D里拖动立方体到目标点；点击3D立方体可直接启动该颜色抓取')
        hint.setStyleSheet('color: #475569;')
        hint.setWordWrap(True)
        place_layout.addWidget(hint)
        right_layout.addWidget(place_box)
        right_layout.addStretch(1)

        self.lb_result_preview = None
        self.lb_depth_preview = None
        panel_layout.addWidget(right_panel, 2)

        status_box = QGroupBox('状态')
        status_layout = QHBoxLayout(status_box)
        text_col = QVBoxLayout()
        self.lb_preview_status = QLabel('视频状态: 等待中...')
        self.lb_preview_status.setStyleSheet('color: #546E7A;')
        self.lb_preview_status.setWordWrap(True)
        self.lb_arm_state = QLabel('机械臂状态: 等待 /ros_robot_controller/arm/full_state')
        self.lb_arm_state.setStyleSheet('color: #374151;')
        self.lb_arm_state.setWordWrap(True)
        self.lb_manual_pose = QLabel('')
        self.lb_manual_pose.setStyleSheet('font-weight: bold; color: #37474F;')
        self.lb_manual_pose.setWordWrap(True)
        text_col.addWidget(self.lb_preview_status)
        text_col.addWidget(self.lb_arm_state)
        text_col.addWidget(self.lb_manual_pose)
        status_layout.addLayout(text_col, 1)
        action_col = QVBoxLayout()
        self.btn_stop_gameplay = QPushButton('停止玩法')
        self.btn_stop_gameplay.pressed.connect(self.stop_task_queue)
        action_col.addWidget(self.btn_stop_gameplay)
        action_col.addStretch(1)
        status_layout.addLayout(action_col)
        page_layout.addWidget(status_box)
        self._update_manual_pose_label()

    def _init_preview_label(self, label, topic, min_w, min_h):
        label.setMinimumSize(min_w, min_h)
        label.setAlignment(Qt.AlignCenter)
        label.setWordWrap(True)
        label.setStyleSheet(
            'background-color: #0F172A;'
            'color: #D1D5DB;'
            'border: 1px solid #334155;'
            'border-radius: 6px;'
        )
        label.setText(f'等待视频流...\n{topic}')

    def _on_result_image_msg(self, msg):
        if self._closing:
            return
        try:
            self.result_image_signal.emit(msg)
        except RuntimeError:
            pass

    def _on_rgb_image_msg(self, msg):
        if self._closing:
            return
        try:
            self.rgb_image_signal.emit(msg)
        except RuntimeError:
            pass

    def _on_depth_image_msg(self, msg):
        if self._closing:
            return
        try:
            self.depth_image_signal.emit(msg)
        except RuntimeError:
            pass

    def _on_preview_status_msg(self, text):
        if self._closing:
            return
        try:
            self.preview_status_signal.emit(text)
        except RuntimeError:
            pass

    def _on_arm_state_msg(self, msg):
        if self._closing:
            return
        state = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'pitch': float(msg.pitch),
            'roll': float(msg.roll),
            'claw': float(msg.claw),
            'yaw': float(msg.yaw),
            'joint_angles': [float(v) for v in msg.joint_angles],
            'servos': [int(v) for v in msg.servos],
        }
        try:
            self.arm_state_signal.emit(state)
        except RuntimeError:
            pass

    def _on_arm_state_ui(self, state):
        now = time.time()
        if now - self._last_arm_ui_update_ts < self._arm_ui_interval:
            return
        self._last_arm_ui_update_ts = now
        self.latest_arm_state = dict(state)
        self.manual_pose['x'] = float(state['x'])
        self.manual_pose['y'] = float(state['y'])
        self.manual_pose['z'] = float(state['z'])
        self.manual_pose['pitch'] = float(state['pitch'])
        self.manual_pose['roll'] = float(state['roll'])
        self.manual_pose['claw'] = float(state['claw'])
        self._update_manual_pose_label()
        if hasattr(self, 'lb_arm_state'):
            self.lb_arm_state.setText(
                f"机械臂状态: X={state['x']:.1f} Y={state['y']:.1f} Z={state['z']:.1f} "
                f"Pitch={state['pitch']:.1f} Roll={state['roll']:.1f} Claw={state['claw']:.1f}"
            )
        if self.arm_3d_widget is not None:
            self.arm_3d_widget.set_state(state)
        if hasattr(self, 'lb_sim_status'):
            if self.arm_3d_widget is None:
                self.lb_sim_status.setText('内嵌仿真: 组件未初始化')
            elif getattr(self.arm_3d_widget, 'model_loaded', False):
                self.lb_sim_status.setText('内嵌仿真: URDF 联动中')
            else:
                err = getattr(self.arm_3d_widget, 'load_error', '') or '模型未加载'
                self.lb_sim_status.setText(f'内嵌仿真: {err}')

    def _set_preview_status_ui(self, text):
        if hasattr(self, 'lb_preview_status'):
            self.lb_preview_status.setText(f'视频状态: {text}')

    def _update_result_preview(self, msg):
        self._update_preview_from_msg('result', self.lb_result_preview, msg)

    def _update_rgb_preview(self, msg):
        self._update_preview_from_msg('rgb', self.lb_rgb_preview, msg)

    def _update_depth_preview(self, msg):
        self._update_preview_from_msg('depth', self.lb_depth_preview, msg)

    def _update_preview_from_msg(self, key, label, msg):
        if label is None:
            return
        qimg = self._ros_image_to_qimage(msg)
        if qimg is None:
            enc = str(getattr(msg, 'encoding', '') or '').lower()
            warn_key = f'{key}:{enc}'
            if warn_key not in self._preview_decode_warned:
                self._preview_decode_warned.add(warn_key)
                self._set_preview_status_ui(f'{key} 解码失败: encoding={enc}, w={msg.width}, h={msg.height}, step={msg.step}')
            return
        self.last_preview_images[key] = qimg
        pix = QPixmap.fromImage(qimg)
        label.setPixmap(pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _ros_image_to_qimage(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0 or step <= 0:
            return None

        enc_raw = str(getattr(msg, 'encoding', '') or '')
        enc = enc_raw.lower().replace(' ', '').replace('-', '')
        data = msg.data
        try:
            if enc in ('rgb8', 'bgr8'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w * 3].reshape(h, w, 3)
                if enc == 'bgr8':
                    arr = arr[:, :, ::-1].copy()
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_RGB888).copy()
            if enc in ('rgba8', 'bgra8'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w * 4].reshape(h, w, 4)
                if enc == 'bgra8':
                    arr = arr[:, :, [2, 1, 0, 3]].copy()
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_RGBA8888).copy()
            if enc in ('yuyv', 'yuyv422', 'yuv422', 'uyvy', 'yuv422yuy2'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                packed = row[:, :w * 2]
                rgb = self._yuv422_to_rgb888(packed, w, enc)
                return QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
            if enc in ('8uc3', '8sc3'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w * 3].reshape(h, w, 3)
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_RGB888).copy()
            if enc in ('8uc4', '8sc4'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w * 4].reshape(h, w, 4)
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_RGBA8888).copy()

            if enc in ('mono8', '8uc1'):
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w].copy()
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_Grayscale8).copy()

            if enc in ('16uc1', 'mono16'):
                row = np.frombuffer(data, dtype=np.uint16).reshape(h, step // 2)
                depth = row[:, :w].astype(np.float32)
                return self._depth_to_qimage(depth)

            if enc in ('32fc1',):
                row = np.frombuffer(data, dtype=np.float32).reshape(h, step // 4)
                depth = row[:, :w].astype(np.float32)
                return self._depth_to_qimage(depth)
            if step >= w * 3:
                row = np.frombuffer(data, dtype=np.uint8).reshape(h, step)
                arr = row[:, :w * 3].reshape(h, w, 3)
                return QImage(arr.data, w, h, arr.strides[0], QImage.Format_RGB888).copy()

        except Exception:
            return None
        return None

    def _yuv422_to_rgb888(self, packed, w, enc):
        h = packed.shape[0]
        ww = int(w)
        if ww % 2 != 0:
            ww -= 1
        p = packed[:, :ww * 2].reshape(h, ww // 2, 4).astype(np.float32)
        if enc == 'uyvy':
            u = p[:, :, 0]
            y0 = p[:, :, 1]
            v = p[:, :, 2]
            y1 = p[:, :, 3]
        else:
            y0 = p[:, :, 0]
            u = p[:, :, 1]
            y1 = p[:, :, 2]
            v = p[:, :, 3]
        y = np.empty((h, ww), dtype=np.float32)
        y[:, 0::2] = y0
        y[:, 1::2] = y1
        uu = np.repeat(u, 2, axis=1)
        vv = np.repeat(v, 2, axis=1)
        c = y - 16.0
        d = uu - 128.0
        e = vv - 128.0
        r = (298.082 * c + 408.583 * e) / 256.0
        g = (298.082 * c - 100.291 * d - 208.120 * e) / 256.0
        b = (298.082 * c + 516.412 * d) / 256.0
        rgb = np.stack([r, g, b], axis=-1)
        rgb = np.clip(rgb, 0.0, 255.0).astype(np.uint8)
        if ww != w:
            pad = np.zeros((h, 1, 3), dtype=np.uint8)
            rgb = np.concatenate([rgb, pad], axis=1)
        return rgb

    def _depth_to_qimage(self, depth):
        finite = np.isfinite(depth)
        valid = depth[finite & (depth > 1e-6)]
        if valid.size == 0:
            gray = np.zeros(depth.shape, dtype=np.uint8)
        else:
            low = float(np.percentile(valid, 5))
            high = float(np.percentile(valid, 99))
            if high <= low:
                high = low + 1.0
            norm = (depth - low) / (high - low)
            norm = np.clip(norm, 0.0, 1.0)
            gray = (norm * 255.0).astype(np.uint8)
        return QImage(gray.data, gray.shape[1], gray.shape[0], gray.strides[0], QImage.Format_Grayscale8).copy()

    def _sync_manual_pose_from_scene(self):
        home = self.current_scene().get('home_pose', DEFAULT_HOME)
        self.manual_pose = {
            'x': float(home.get('x', DEFAULT_HOME['x'])),
            'y': float(home.get('y', DEFAULT_HOME['y'])),
            'z': float(home.get('z', DEFAULT_HOME['z'])),
            'pitch': float(home.get('pitch', DEFAULT_HOME['pitch'])),
            'roll': float(home.get('roll', DEFAULT_HOME['roll'])),
            'claw': float(home.get('claw', DEFAULT_HOME['claw'])),
        }
        self._update_manual_pose_label()

    def _update_manual_pose_label(self):
        p = self.manual_pose
        self.lb_manual_pose.setText(
            '手动位姿: '
            f'X={p["x"]:.1f}  Y={p["y"]:.1f}  Z={p["z"]:.1f}  '
            f'Pitch={p["pitch"]:.1f}  Roll={p["roll"]:.1f}  Claw={p["claw"]:.1f}'
        )

    def _manual_go_home(self):
        self.go_home()
        self._sync_manual_pose_from_scene()

    def _manual_move_relative(self, dx=0.0, dy=0.0, dz=0.0, dclaw=0.0):
        if self.node is None:
            return
        self.manual_pose['x'] = max(0.0, min(550.0, self.manual_pose['x'] + float(dx)))
        self.manual_pose['y'] = max(-550.0, min(550.0, self.manual_pose['y'] + float(dy)))
        self.manual_pose['z'] = max(0.0, min(570.0, self.manual_pose['z'] + float(dz)))
        self.manual_pose['claw'] = max(-90.0, min(90.0, self.manual_pose['claw'] + float(dclaw)))
        self.node.publish_arm(
            self.manual_pose['x'],
            self.manual_pose['y'],
            self.manual_pose['z'],
            self.manual_pose['pitch'],
            self.manual_pose['roll'],
            self.manual_pose['claw'],
            int(self.sp_dash_time.value()) if hasattr(self, 'sp_dash_time') else 350,
        )
        self._update_manual_pose_label()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        labels = {
            'result': getattr(self, 'lb_result_preview', None),
            'rgb': getattr(self, 'lb_rgb_preview', None),
            'depth': getattr(self, 'lb_depth_preview', None),
        }
        for key, label in labels.items():
            if label is None:
                continue
            qimg = self.last_preview_images.get(key)
            if qimg is None:
                continue
            pix = QPixmap.fromImage(qimg)
            label.setPixmap(pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def ensure_urdf_sim_running(self):
        # 保留函数名以兼容历史调用；现改为内嵌仿真，不再启动外部 URDF 进程。
        self.lb_sim_status.setText('内嵌仿真: 等待状态流...')

    def current_scene_id(self):
        sid = self.cb_scene.currentData()
        if not sid:
            sid = self.scene_cfg.get('current_scene', DEFAULT_SCENE_ID)
        return sid

    def current_scene(self):
        sid = self.current_scene_id()
        return self.scene_cfg['scenes'][sid]

    def load_scene_combo(self):
        self.cb_scene.blockSignals(True)
        self.cb_scene.clear()
        for sid, scene in self.scene_cfg['scenes'].items():
            self.cb_scene.addItem(f'{sid} ({scene.get("name", sid)})', userData=sid)
        cur = self.scene_cfg.get('current_scene', DEFAULT_SCENE_ID)
        idx = self.cb_scene.findData(cur)
        self.cb_scene.setCurrentIndex(max(0, idx))
        self.cb_scene.blockSignals(False)
        self.on_scene_changed()

    def refresh_scene_widgets(self):
        scene = self.current_scene()
        policy = scene.get('place_policy', {})
        self.chk_left_only.blockSignals(True)
        self.sp_min_z.blockSignals(True)
        self.chk_left_only.setChecked(bool(policy.get('only_left_y_positive', False)))
        self.sp_min_z.setValue(float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z'])))
        self.chk_left_only.blockSignals(False)
        self.sp_min_z.blockSignals(False)

        for key, (spx, spy, spz, btn) in self.row_widgets.items():
            pos = scene['place_targets'].get(key, DEFAULT_PLACE_TARGETS[key])
            spx.blockSignals(True); spy.blockSignals(True); spz.blockSignals(True)
            spx.setValue(float(pos[0]))
            spy.setValue(float(pos[1]))
            spz.setValue(float(pos[2]))
            spx.blockSignals(False); spy.blockSignals(False); spz.blockSignals(False)
            if bool(policy.get('only_left_y_positive', False)) and key in ('right_top', 'right_bottom'):
                btn.setEnabled(False)
            else:
                btn.setEnabled(True)

    def on_scene_changed(self):
        self.scene_cfg['current_scene'] = self.current_scene_id()
        self._load_place_map_params()
        self._sync_urdf_from_scene()
        self.refresh_scene_widgets()
        self._sync_manual_pose_from_scene()
        self._sync_color_blocks_from_scene()

    def on_target_changed(self, key, axis, value):
        scene = self.current_scene()
        pos = list(scene['place_targets'].get(key, DEFAULT_PLACE_TARGETS[key]))
        pos[axis] = float(value)
        scene['place_targets'][key] = pos
        if key in ('red', 'green', 'blue'):
            self._sync_color_blocks_from_scene()

    def _sync_color_blocks_from_scene(self):
        scene = self.current_scene()
        targets = scene.get('place_targets', {})
        if hasattr(self, 'arm_3d_widget') and self.arm_3d_widget is not None:
            visual_targets = dict(targets)
            for key in ('red', 'green', 'blue'):
                p = list(visual_targets.get(key, DEFAULT_PLACE_TARGETS[key]))
                p[0], p[1] = self._target_to_visual_xy(p[0], p[1], key=key)
                visual_targets[key] = p
            self.arm_3d_widget.set_place_targets(visual_targets)

    def _kinematics_scale_offset(self):
        kin = self.params.get('kinematics', {}) if isinstance(self.params, dict) else {}
        scale = kin.get('scale', [1.0, 1.0, 1.0])
        offset = kin.get('offset', [0.0, 0.0, 0.0])
        try:
            sx = float(scale[0]); sy = float(scale[1])
            ox = float(offset[0]); oy = float(offset[1])
        except Exception:
            sx, sy, ox, oy = 1.0, 1.0, 0.0, 0.0
        if abs(sx) < 1e-9:
            sx = 1.0
        if abs(sy) < 1e-9:
            sy = 1.0
        return sx, sy, ox, oy

    def _place_exec_model(self, key=None):
        return 'stackup' if key == 'tag_stackup' else 'sorting'

    def _target_to_exec_xy(self, tx, ty, key=None):
        x = float(tx)
        y = float(ty)
        sx, sy, ox, oy = self._kinematics_scale_offset()
        angle = math.degrees(math.atan2(y, x))
        model = self._place_exec_model(key)
        if angle > 45.0:
            ex = x * sy
            ey = y * sx
            if model == 'stackup':
                ex -= oy
            else:
                ex += oy
            ey += ox
            return ex, ey
        if angle < -45.0:
            ex = x * sy + oy
            ey = y * sx - ox
            return ex, ey
        ex = x * sx + ox
        ey = y * sy + oy
        return ex, ey

    def _exec_to_target_xy(self, ex, ey, key=None):
        ex = float(ex)
        ey = float(ey)
        sx, sy, ox, oy = self._kinematics_scale_offset()
        model = self._place_exec_model(key)

        candidates = []
        # mid branch: -45 <= angle <= 45
        x0 = (ex - ox) / sx
        y0 = (ey - oy) / sy
        a0 = math.degrees(math.atan2(y0, x0))
        candidates.append((x0, y0, a0, 0))

        # + branch: angle > 45
        if model == 'stackup':
            x1 = (ex + oy) / sy
        else:
            x1 = (ex - oy) / sy
        y1 = (ey - ox) / sx
        a1 = math.degrees(math.atan2(y1, x1))
        candidates.append((x1, y1, a1, 1))

        # - branch: angle < -45
        x2 = (ex - oy) / sy
        y2 = (ey + ox) / sx
        a2 = math.degrees(math.atan2(y2, x2))
        candidates.append((x2, y2, a2, -1))

        for xx, yy, aa, branch in candidates:
            if branch == 0 and -45.0 <= aa <= 45.0:
                return float(xx), float(yy)
            if branch == 1 and aa > 45.0:
                return float(xx), float(yy)
            if branch == -1 and aa < -45.0:
                return float(xx), float(yy)

        # Fallback: choose closest branch boundary
        best = None
        for xx, yy, aa, branch in candidates:
            if branch == 0:
                score = 0.0 if -45.0 <= aa <= 45.0 else min(abs(aa - 45.0), abs(aa + 45.0))
            elif branch == 1:
                score = 0.0 if aa > 45.0 else abs(aa - 45.0)
            else:
                score = 0.0 if aa < -45.0 else abs(aa + 45.0)
            if best is None or score < best[0]:
                best = (score, xx, yy)
        return float(best[1]), float(best[2])

    def _target_to_visual_xy(self, tx, ty, key=None):
        if self.place_exec_comp_enabled:
            tx, ty = self._target_to_exec_xy(tx, ty, key=key)
        sx = self.place_map_scale_x if abs(self.place_map_scale_x) > 1e-9 else 1.0
        sy = self.place_map_scale_y if abs(self.place_map_scale_y) > 1e-9 else 1.0
        vx = (float(tx) - self.place_map_offset_x) / sx
        vy = (float(ty) - self.place_map_offset_y) / sy
        return vx, vy

    def _visual_to_target_xy(self, vx, vy, key=None):
        tx = float(vx) * self.place_map_scale_x + self.place_map_offset_x
        ty = float(vy) * self.place_map_scale_y + self.place_map_offset_y
        if self.place_exec_comp_enabled:
            tx, ty = self._exec_to_target_xy(tx, ty, key=key)
        return tx, ty

    def _set_side_color_selected(self, key_or_none):
        self._selected_color_key = key_or_none if key_or_none in ('red', 'green', 'blue') else None
        checked_map = {
            'red': self._selected_color_key == 'red',
            'green': self._selected_color_key == 'green',
            'blue': self._selected_color_key == 'blue',
        }
        if hasattr(self, 'btn_pick_red'):
            self.btn_pick_red.setChecked(checked_map['red'])
        if hasattr(self, 'btn_pick_green'):
            self.btn_pick_green.setChecked(checked_map['green'])
        if hasattr(self, 'btn_pick_blue'):
            self.btn_pick_blue.setChecked(checked_map['blue'])

    def _on_toggle_view_mode(self, checked):
        view_mode = bool(checked)
        if hasattr(self, 'btn_view_mode'):
            self.btn_view_mode.setText('切到放置模式' if view_mode else '切到视角模式')
        if self.arm_3d_widget is not None:
            self.arm_3d_widget.set_place_edit_enabled(not view_mode)
        if view_mode:
            self._set_side_color_selected(None)
            self._append_feedback('已切换到视角模式：可旋转/缩放，暂停放置编辑')
        else:
            self._append_feedback('已切换到放置模式：可拖动3D立方体修改放置点')

    def _on_zoom_view(self, zoom_in):
        if self.arm_3d_widget is not None:
            self.arm_3d_widget.zoom_camera(bool(zoom_in))

    def _on_reset_view(self):
        if self.arm_3d_widget is not None:
            self.arm_3d_widget.reset_camera_view()

    def _on_side_color_pick(self, key):
        if getattr(self, 'btn_view_mode', None) is not None and self.btn_view_mode.isChecked():
            self._append_feedback('当前在视角模式，请先切回放置模式再选颜色')
            return
        if self._selected_color_key == key:
            self._set_side_color_selected(None)
            self._append_feedback(f'颜色 {key} 已确认，启动该颜色抓取')
            self._on_color_block_clicked(key)
            return

        self._set_side_color_selected(key)
        if self.arm_3d_widget is not None:
            self.arm_3d_widget.set_active_target_from_side(key)
        self._append_feedback(f'已选择颜色源: {key}，请在3D内拖动该颜色立方体到目标位置；再次点同色将启动玩法')

    def _on_color_block_moved(self, key, x_m, y_m):
        scene = self.current_scene()
        pos = list(scene['place_targets'].get(key, DEFAULT_PLACE_TARGETS[key]))
        tx, ty = self._visual_to_target_xy(x_m, y_m, key=key)
        pos[0] = float(tx)
        pos[1] = float(ty)
        pos = self.apply_place_policy(pos)
        scene['place_targets'][key] = pos
        if key in self.row_widgets:
            spx, spy, spz, _btn = self.row_widgets[key]
            spx.blockSignals(True); spy.blockSignals(True); spz.blockSignals(True)
            spx.setValue(float(pos[0]))
            spy.setValue(float(pos[1]))
            spz.setValue(float(pos[2]))
            spx.blockSignals(False); spy.blockSignals(False); spz.blockSignals(False)
        self._sync_color_blocks_from_scene()
        self.save_scene_cfg()
        self._append_feedback(f'更新放置点: {key} -> x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}')

    def _on_color_block_clicked(self, key):
        if self.task_worker_running:
            self._append_feedback(f'忽略点击({key}): 任务执行中')
            return
        self._set_side_color_selected(None)
        idx_type = self.cb_task_type.findData('color')
        if idx_type >= 0:
            self.cb_task_type.setCurrentIndex(idx_type)
        idx_target = self.cb_task_target.findData(key)
        if idx_target >= 0:
            self.cb_task_target.setCurrentIndex(idx_target)
        self.start_current_task()

    def on_policy_changed(self, checked):
        scene = self.current_scene()
        scene['place_policy']['only_left_y_positive'] = bool(checked)
        self.refresh_scene_widgets()

    def on_min_z_changed(self, value):
        scene = self.current_scene()
        scene['place_policy']['min_place_z'] = float(value)

    def add_scene(self):
        i = 1
        while f'scene_{i}' in self.scene_cfg['scenes']:
            i += 1
        sid = f'scene_{i}'
        self.scene_cfg['scenes'][sid] = self.default_scene(f'Scene {i}')
        self.scene_cfg['current_scene'] = sid
        self.load_scene_combo()

    def delete_scene(self):
        sid = self.current_scene_id()
        if sid == DEFAULT_SCENE_ID:
            QMessageBox.warning(self, '提示', 'scene_1 为默认场景，不能删除')
            return
        self.scene_cfg['scenes'].pop(sid, None)
        self.scene_cfg['current_scene'] = next(iter(self.scene_cfg['scenes'].keys()))
        self.load_scene_combo()

    def save_scene_clicked(self):
        self.save_scene_cfg()
        QMessageBox.information(self, '保存成功', SCENE_YAML_PATH)

    def calibration_position(self, position):
        x, y, z = float(position[0]), float(position[1]), float(position[2])
        yaw = math.degrees(math.atan2(y, x))
        offset = self.params['kinematics']['offset']
        scale = self.params['kinematics']['scale']
        if yaw > 45:
            yaw = math.degrees(math.atan2(-x, y))
            x, y, z = x * scale[1], y * scale[0], z * scale[2]
            x, y, z = x - offset[1], y + offset[0], z + offset[2]
        elif yaw < -45:
            yaw = math.degrees(math.atan2(x, -y))
            x, y, z = x * scale[1], y * scale[0], z * scale[2]
            x, y, z = x + offset[1], y - offset[0], z + offset[2]
        else:
            x, y, z = x * scale[0], y * scale[1], z * scale[2]
            x, y, z = x + offset[0], y + offset[1], z + offset[2]
        return [x, y, z], float(yaw)

    def apply_place_policy(self, pos):
        scene = self.current_scene()
        policy = scene.get('place_policy', {})
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if bool(policy.get('only_left_y_positive', False)) and y < 0.0:
            y = abs(y)
        min_z = float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z']))
        if z < min_z:
            z = min_z
        return [x, y, z]

    def move_target(self, key):
        scene = self.current_scene()
        pos = scene['place_targets'].get(key)
        if pos is None:
            return
        self.go_home()
        pos = self.apply_place_policy(pos)
        pos, roll_deg = self.calibration_position(pos)
        home = scene.get('home_pose', DEFAULT_HOME)
        self.node.move_to(pos, roll_deg, home)

    def go_home(self):
        home = self.current_scene().get('home_pose', DEFAULT_HOME)
        self.node.init_pose(home)

    def refresh_task_target_combo(self):
        task_type = self.cb_task_type.currentData()
        self.cb_task_target.blockSignals(True)
        self.cb_task_target.clear()
        for key, label in TASK_TARGET_OPTIONS.get(task_type, []):
            self.cb_task_target.addItem(label, userData=key)
        self.cb_task_target.blockSignals(False)

    def on_task_type_changed(self):
        self.refresh_task_target_combo()

    def _task_display_text(self, task):
        task_type = task.get('type')
        target = task.get('target')
        type_label = TASK_TYPE_LABELS.get(task_type, task_type)
        target_dict = dict((k, v) for k, v in TASK_TARGET_OPTIONS.get(task_type, []))
        target_label = target_dict.get(target, str(target))
        return f'{type_label} -> {target_label}'

    def add_task_item(self):
        task_type = self.cb_task_type.currentData()
        target = self.cb_task_target.currentData()
        if not task_type or not target:
            return
        task = {'type': task_type, 'target': target}
        self.task_queue.append(task)
        self.lb_task_queue.addItem(self._task_display_text(task))

    def remove_task_item(self):
        row = self.lb_task_queue.currentRow()
        if row < 0 or row >= len(self.task_queue):
            return
        self.task_queue.pop(row)
        self.lb_task_queue.takeItem(row)

    def clear_task_items(self):
        if self.task_worker_running:
            return
        self.task_queue = []
        self.lb_task_queue.clear()
        self.lb_task_status.setText('状态: 空闲')

    def _set_task_status(self, text):
        self.task_status_signal.emit(text)

    def _append_feedback(self, text):
        self.task_feedback_signal.emit(text)

    def _highlight_task_row(self, index):
        self.task_highlight_signal.emit(index)

    def _set_task_status_ui(self, text):
        self.lb_task_status.setText(f'状态: {text}')

    def _highlight_task_row_ui(self, index):
        self.lb_task_queue.setCurrentRow(index)

    def _append_feedback_ui(self, text):
        self.lb_task_feedback.addItem(text)
        self.lb_task_feedback.scrollToBottom()

    def start_current_task(self):
        if self.task_worker_running:
            QMessageBox.warning(self, '提示', '任务正在执行中')
            return
        task_type = self.cb_task_type.currentData()
        target = self.cb_task_target.currentData()
        if not task_type or not target:
            QMessageBox.warning(self, '提示', '请先选择玩法和目标')
            return
        text = self._task_display_text({'type': task_type, 'target': target})
        self._set_task_status(f'启动玩法: {text}')
        self._append_feedback(f'启动请求 -> {text}')
        self.task_worker_running = True
        threading.Thread(target=self._start_current_task_worker, args=(task_type, target, text), daemon=True).start()

    def _start_current_task_worker(self, task_type, target, text):
        try:
            ok, reason = self.node.run_task(task_type, target)
            if ok:
                self._set_task_status(f'玩法已启动: {text}')
                self._append_feedback(f'启动成功: {reason}')
            else:
                self._set_task_status(f'启动失败: {text}')
                self._append_feedback(f'启动失败: {reason}')
        finally:
            self.task_worker_running = False

    def start_task_queue(self):
        if self.task_worker_running:
            QMessageBox.warning(self, '提示', '任务正在执行中')
            return
        if not self.task_queue:
            QMessageBox.warning(self, '提示', '请先添加抓取任务顺序')
            return
        self.task_worker_stop = False
        self.task_worker_running = True
        threading.Thread(target=self._task_worker, daemon=True).start()

    def stop_task_queue(self):
        self.task_worker_stop = True
        self.task_worker_running = False
        _ok, feedback = self.node.task_stop_all()
        self._set_task_status('已停止')
        self._append_feedback(f'停止请求: {feedback}')

    def _task_worker(self):
        try:
            total = len(self.task_queue)
            for idx, task in enumerate(list(self.task_queue)):
                if self.task_worker_stop:
                    self._set_task_status('已停止')
                    return
                self._highlight_task_row(idx)
                text = self._task_display_text(task)
                self._set_task_status(f'执行中 {idx + 1}/{total}: {text}')
                self._append_feedback(f'执行任务: {text}')
                ok, reason = self.node.run_task(task['type'], task['target'])
                if not ok:
                    self.node.task_stop_all()
                    self._set_task_status(f'任务失败: {text} ({reason})')
                    self._append_feedback(f'任务失败: {reason}')
                    return
                self._append_feedback(f'任务启动成功: {reason}')
                wait_sec = 10.0 if task['type'] != 'stackup' else 20.0
                deadline = time.time() + wait_sec
                while time.time() < deadline:
                    if self.task_worker_stop:
                        self._set_task_status('已停止')
                        return
                    time.sleep(0.1)
            self.node.task_stop_all()
            self._set_task_status('执行完成')
            self._append_feedback('顺序执行完成')
        finally:
            self.task_worker_running = False

    def closeEvent(self, event):
        self._closing = True
        try:
            if self.node is not None:
                self.node.set_arm_state_callback(None)
                self.node.set_preview_callbacks(on_result=None, on_rgb=None, on_depth=None, on_status=None)
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
