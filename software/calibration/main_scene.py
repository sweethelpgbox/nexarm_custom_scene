#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import yaml
import math
import json
import copy
import signal
import subprocess
import rclpy
import threading

_CALIBRATION_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.py')
_calib_proc = None


def _start_calibration_subprocess():
    global _calib_proc
    try:
        _calib_proc = subprocess.Popen(
            ['python3', _CALIBRATION_PY],
            cwd=os.path.dirname(_CALIBRATION_PY),
        )
    except Exception as e:
        print(f'[main_scene] calibration.py 启动失败: {e}')


def _stop_calibration_subprocess():
    global _calib_proc
    if _calib_proc is not None and _calib_proc.poll() is None:
        try:
            _calib_proc.send_signal(signal.SIGINT)
            _calib_proc.wait(timeout=5)
        except Exception:
            try:
                _calib_proc.terminate()
            except Exception:
                pass
    _calib_proc = None
from PyQt5.QtCore import Qt, pyqtSignal, QPointF, QRectF, QSize
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QComboBox,
    QCheckBox,
    QLabel,
    QDoubleSpinBox,
    QMessageBox,
    QWidget,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QTabWidget,
    QTabBar,
    QScrollArea,
    QSizePolicy,
    QDialog,
    QSplitter,
    QFrame,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import json as _json
from std_msgs.msg import Int8, String as RosString
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetString, SetStringBool, SetStringList
from ros_robot_controller_msgs.msg import ArmCoords
from sensor_msgs.msg import Image, CompressedImage


SCENE0_ID = 'scene_0'
DEFAULT_SCENE_ID = 'scene_1'
SCENE2_ID = 'scene_2'
SCENE3_ID = 'scene_3'
SCENE4_ID = 'scene_4'
SCENE5_ID = 'scene_5'
BUILTIN_SCENE_NAMES = {
    SCENE0_ID: '无沙盘场景',
    DEFAULT_SCENE_ID: '基础分拣沙盘',
    SCENE2_ID: '标准分拣沙盘',
    SCENE3_ID: '豪华分拣沙盘',
    SCENE4_ID: '电动滑轨货仓沙盘',
    SCENE5_ID: '双臂流水线沙盘',
}
SCENE_PLAY_IDS = {
    SCENE0_ID: 'idle',
    DEFAULT_SCENE_ID: 'basic_sorting',
    SCENE2_ID: 'color_tag_sorting',
    SCENE3_ID: 'waste_classification',
    SCENE4_ID: 'slide_rail_sorting',
    SCENE5_ID: 'dual_arm_conveyor',
}
APP_PLAY_CONFIG_DIR = '/home/ubuntu/ros2_ws/src/app/config/plays'
EXAMPLE_MOTOR_PLAY_CONFIG_DIR = '/home/ubuntu/ros2_ws/src/example/example/motor/plays'
SCENE_PLAY_CONFIG_PATHS = {
    SCENE0_ID: os.path.join(APP_PLAY_CONFIG_DIR, 'scene0_idle.yaml'),
    DEFAULT_SCENE_ID: os.path.join(APP_PLAY_CONFIG_DIR, 'scene1_calibration.yaml'),
    SCENE2_ID: os.path.join(APP_PLAY_CONFIG_DIR, 'scene2_color_tag_sorting.yaml'),
    SCENE3_ID: os.path.join(APP_PLAY_CONFIG_DIR, 'scene3_waste_classification.yaml'),
    SCENE4_ID: os.path.join(APP_PLAY_CONFIG_DIR, 'scene4_slide_rail_sorting.yaml'),
    SCENE5_ID: os.path.join(EXAMPLE_MOTOR_PLAY_CONFIG_DIR, 'scene5_dual_arm.yaml'),
}
SCENE_PLAY_CONFIG_KEYS = {
    SCENE0_ID: ('place_policy', 'place_targets'),
    DEFAULT_SCENE_ID: ('place_policy', 'place_targets', 'place_roll'),
    SCENE2_ID: ('place_policy', 'place_targets', 'color_grid'),
    SCENE3_ID: ('place_policy', 'place_targets', 'place_pitch', 'scene3_grid'),
    SCENE4_ID: (
        'place_policy',
        'place_targets',
        'rail',
        'scene4_pick',
        'scene4_place',
        'scene4_absolute_positions',
        'scene4_grid',
        'scene4_shelf',
        'kinematics',
    ),
    SCENE5_ID: ('place_policy', 'place_targets', 'scene5_grid', 'scene5_dual_arm'),
}
CALIBRATION_TYPERC_PATH = os.environ.get('CALIBRATION_TYPERC_PATH', '/home/ubuntu/ros2_ws/.typerc')


def clean_typerc_export_value(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def load_typerc_env_defaults(path=CALIBRATION_TYPERC_PATH):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.read().splitlines()
    except Exception:
        return
    keys = {
        'CALIBRATION_CURRENT_SCENE',
        'CALIBRATION_DEFAULT_SCENE',
        'SCENE',
        'SCENE5_ARM_ROLE',
        'CHASSIS_TYPE',
    }
    for line in lines:
        line = line.strip()
        if not line.startswith('export ') or '=' not in line:
            continue
        key, value = line[len('export '):].split('=', 1)
        key = key.strip()
        if key in keys and not os.environ.get(key):
            os.environ[key] = clean_typerc_export_value(value)


load_typerc_env_defaults()


def configured_scene_env():
    return (
        os.environ.get('CALIBRATION_CURRENT_SCENE')
        or os.environ.get('SCENE')
        or os.environ.get('CALIBRATION_DEFAULT_SCENE')
    )


ENV_CURRENT_SCENE = configured_scene_env()
DEFAULT_CURRENT_SCENE = ENV_CURRENT_SCENE or SCENE0_ID
if DEFAULT_CURRENT_SCENE not in (SCENE0_ID, DEFAULT_SCENE_ID, SCENE2_ID, SCENE3_ID, SCENE4_ID, SCENE5_ID):
    DEFAULT_CURRENT_SCENE = SCENE0_ID
    ENV_CURRENT_SCENE = None
SCENE2_COLOR_KEYS = ('yellow', 'red', 'green', 'blue')
SCENE4_COLOR_KEYS = ('red', 'green', 'yellow', 'blue')
SCENE2_COLOR_LABELS = {
    'yellow': '黄',
    'red': '红',
    'green': '绿',
    'blue': '蓝',
}
SCENE2_COLOR_QCOLORS = {
    'yellow': QColor(250, 204, 21),
    'red': QColor(239, 68, 68),
    'green': QColor(34, 197, 94),
    'blue': QColor(59, 130, 246),
}
SCENE2_SLOT_TARGET_KEYS = ('yellow', 'red', 'green', 'blue')
TAG_KEYS = ('tag1', 'tag2', 'tag3')
TAG_LABELS = {
    'tag1': '标签1',
    'tag2': '标签2',
    'tag3': '标签3',
}
TAG_QCOLORS = {
    'tag1': QColor(251, 146, 60),
    'tag2': QColor(168, 85, 247),
    'tag3': QColor(20, 184, 166),
}
OBJECT_SORT_KEYS = SCENE2_COLOR_KEYS + TAG_KEYS
WASTE_KEYS = ('residual_waste', 'food_waste', 'hazardous_waste', 'recyclable_waste')
WASTE_LABELS = {
    'residual_waste': '其他垃圾',
    'food_waste': '厨余垃圾',
    'hazardous_waste': '有害垃圾',
    'recyclable_waste': '可回收',
}
SCENE4_CARD_LABELS = {
    'yellow': '黄',
    'red': '红',
    'green': '绿',
    'blue': '蓝',
    'residual_waste': '其',
    'food_waste': '厨',
    'hazardous_waste': '害',
    'recyclable_waste': '回',
}
WASTE_QCOLORS = {
    'residual_waste': QColor(107, 114, 128),
    'food_waste': QColor(34, 197, 94),
    'hazardous_waste': QColor(239, 68, 68),
    'recyclable_waste': QColor(59, 130, 246),
}
SCENE4_MODE_COLOR = 'color'
SCENE4_MODE_WASTE = 'waste'
SCENE4_MODE_ALL = 'all'
SCENE4_MODES = (SCENE4_MODE_COLOR, SCENE4_MODE_ALL)
SCENE4_CONFIG_MODES = SCENE4_MODES
SCENE5_COLOR_KEYS = SCENE2_COLOR_KEYS
SCENE5_WASTE_KEYS = ('food_waste', 'hazardous_waste', 'recyclable_waste', 'residual_waste')
SCENE4_FRAME_SLOT_COUNT = 4
SCENE4_SHELF_SLOT_COUNT = 4
SCENE4_SHELF_SLOT_Y_VALUES = [0.060, 0.020, -0.020, -0.060]
SCENE4_GRID_SLOT_FIELDS = {
    SCENE4_MODE_COLOR: 'color_slots',
    SCENE4_MODE_WASTE: 'waste_slots',
    SCENE4_MODE_ALL: 'all_slots',
}
SCENE3_COLOR_TARGETS = {
    'yellow': [-0.080, 0.290, 0.065],
    'red': [0.090, 0.290, 0.060],
    'green': [0.290, 0.300, 0.070],
    'blue': [0.185, 0.300, 0.060],
}
SCENE3_WASTE_TARGETS = {
    'residual_waste': [-0.050, -0.210, 0.180],
    'food_waste': [0.305, -0.190, 0.180],
    'hazardous_waste': [0.085, -0.210, 0.185],
    'recyclable_waste': [0.195, -0.205, 0.180],
}
SCENE3_PLACE_PITCH = {
    'yellow': -65.0,
    'red': -65.0,
    'green': -65.0,
    'blue': -65.0,
    'residual_waste': -45.0,
    'food_waste': -43.0,
    'hazardous_waste': -45.0,
    'recyclable_waste': -43.0,
}
SCENE3_GROUP_COLOR = 'color'
SCENE3_GROUP_WASTE = 'waste'
SCENE3_GRID_SLOT_FIELDS = {
    SCENE3_GROUP_COLOR: 'color_slots',
    SCENE3_GROUP_WASTE: 'waste_slots',
}
SCENE3_GRID_TARGET_FIELDS = {
    SCENE3_GROUP_COLOR: 'color_slot_targets',
    SCENE3_GROUP_WASTE: 'waste_slot_targets',
}
DEFAULT_SCENE3_GRID = {
    'color_slots': list(SCENE2_COLOR_KEYS),
    'color_slot_targets': [list(SCENE3_COLOR_TARGETS[key]) for key in SCENE2_COLOR_KEYS],
    'waste_slots': list(WASTE_KEYS),
    'waste_slot_targets': [list(SCENE3_WASTE_TARGETS[key]) for key in WASTE_KEYS],
}
SCENE4_SHELF_LENGTH_M = 0.600
SCENE4_SHELF_WIDTH_M = 0.263
SCENE4_CALIBRATION_LENGTH_M = 0.263
SCENE4_CALIBRATION_WIDTH_M = 0.263
SCENE4_SHELF_LEVEL1_Z_M = 0.315
SCENE4_SHELF_LEVEL2_Z_M = 0.190
SCENE4_SHELF_LEVEL_GAP_M = 0.080
SCENE4_PLACE_FRAME = 'frame'
SCENE4_SHELF_LEVEL1 = 'upper_shelf'
SCENE4_SHELF_LEVEL2 = 'lower_shelf'
SCENE4_SHELF_LEVELS = (SCENE4_SHELF_LEVEL1, SCENE4_SHELF_LEVEL2)
SCENE4_PLACE_LABELS = {
    SCENE4_PLACE_FRAME: '框',
    SCENE4_SHELF_LEVEL1: '上层',
    SCENE4_SHELF_LEVEL2: '下层',
}
SCENE4_SHELF_SLOT_FIELDS = {
    SCENE4_MODE_COLOR: {
        SCENE4_SHELF_LEVEL1: 'color_upper_slots',
        SCENE4_SHELF_LEVEL2: 'color_lower_slots',
    },
    SCENE4_MODE_WASTE: {
        SCENE4_SHELF_LEVEL1: 'waste_upper_slots',
        SCENE4_SHELF_LEVEL2: 'waste_lower_slots',
    },
    SCENE4_MODE_ALL: {
        SCENE4_SHELF_LEVEL1: 'all_upper_slots',
        SCENE4_SHELF_LEVEL2: 'all_lower_slots',
    },
}
SCENE4_COLOR_TARGETS = {
    'red': [0.240, 0.060, SCENE4_SHELF_LEVEL1_Z_M],
    'green': [0.240, -0.060, SCENE4_SHELF_LEVEL1_Z_M],
    'yellow': [0.100, 0.060, SCENE4_SHELF_LEVEL2_Z_M],
    'blue': [0.100, -0.060, SCENE4_SHELF_LEVEL2_Z_M],
}
SCENE4_WASTE_TARGET_MAP = {
    'residual_waste': 'red',
    'food_waste': 'green',
    'hazardous_waste': 'yellow',
    'recyclable_waste': 'blue',
}
SCENE4_WASTE_TARGETS = {
    'residual_waste': list(SCENE4_COLOR_TARGETS['red']),
    'food_waste': list(SCENE4_COLOR_TARGETS['green']),
    'hazardous_waste': list(SCENE4_COLOR_TARGETS['yellow']),
    'recyclable_waste': list(SCENE4_COLOR_TARGETS['blue']),
}
SCENE4_FRAME_SLOT_TARGETS = [
    [0.255, 0.12, 0.08],
    [0.255, -0.12, 0.08],
    [0.13, 0.03, 0.08],
    [0.13, -0.13, 0.08],
]
SCENE4_ALL_FRAME_TARGETS = {
    key: list(SCENE4_FRAME_SLOT_TARGETS[index])
    for index, key in enumerate(SCENE4_COLOR_KEYS)
}
SCENE4_DEFAULT_ALL_TARGETS = {
    **{key: list(pos) for key, pos in SCENE4_ALL_FRAME_TARGETS.items()},
}
DEFAULT_SCENE4_ABSOLUTE_POSITIONS = {
    'frame_slots': [list(pos) for pos in SCENE4_FRAME_SLOT_TARGETS],
    'upper_shelf_slots': [
        [0.240, 0.060, SCENE4_SHELF_LEVEL1_Z_M],
        [0.240, 0.020, SCENE4_SHELF_LEVEL1_Z_M],
        [0.240, -0.020, SCENE4_SHELF_LEVEL1_Z_M],
        [0.240, -0.060, SCENE4_SHELF_LEVEL1_Z_M],
    ],
    'lower_shelf_slots': [
        [0.100, 0.060, SCENE4_SHELF_LEVEL2_Z_M],
        [0.100, 0.020, SCENE4_SHELF_LEVEL2_Z_M],
        [0.100, -0.020, SCENE4_SHELF_LEVEL2_Z_M],
        [0.100, -0.060, SCENE4_SHELF_LEVEL2_Z_M],
    ],
}
DEFAULT_BODY_KINEMATICS_PARAMS = [
    110.45,
    225.00,
    36.97,
    145.00,
    0.00,
    130.23,
    0.00,
    50.00,
    70.50,
]
DEFAULT_SCENE4_RAIL = {
    'enabled': True,
    'total_steps': 4200,
    'subdivision': 2,
    'calibration_abs_position': 4000,
    'place_abs_position': 700,
    'reset_wait_sec': 18.0,
    'speed_steps_per_sec': 1000.0,
}
DEFAULT_SCENE4_CALIBRATION_POSE = {
    'x': 145.0,
    'y': 0.0,
    'z': 290.0,
    'pitch': -90.0,
    'roll': 0.0,
    'claw': 0.0,
    'time_ms': 2000,
}
SCENE4_PICK_ZONE_LOWER = 'lower_board'
SCENE4_PICK_ZONE_UPPER = 'upper_shelf'
DEFAULT_SCENE4_PICK = {
    'active_zone': SCENE4_PICK_ZONE_LOWER,
    SCENE4_PICK_ZONE_LOWER: {
        'view_pose': dict(DEFAULT_SCENE4_CALIBRATION_POSE),
        'use_plane_calibration': True,
        'detection': {
            'min_v': 0,
            'max_v': 1080,
        },
    },
}
DEFAULT_SCENE4_SHELF = {
    'length_m': SCENE4_SHELF_LENGTH_M,
    'width_m': SCENE4_SHELF_WIDTH_M,
    'rail_slots': {
        '4': {'left': 3900, 'right': 1100, 'slot_1': 3900, 'slot_2': 2967, 'slot_3': 2033, 'slot_4': 1100},
        '8': {'left': 7800, 'right': 2200, 'slot_1': 7800, 'slot_2': 5933, 'slot_3': 4067, 'slot_4': 2200},
    },
    'level_match_tolerance_m': 0.06,
    'upper_z_m': SCENE4_SHELF_LEVEL1_Z_M,
    'lower_z_m': SCENE4_SHELF_LEVEL2_Z_M,
    'level_gap_m': abs(SCENE4_SHELF_LEVEL1_Z_M - SCENE4_SHELF_LEVEL2_Z_M),
    'levels': {
        SCENE4_SHELF_LEVEL1: {
            'target_z_m': SCENE4_SHELF_LEVEL1_Z_M,
            'approach_pose': {
                'x': 270.0,
                'y': 0.0,
                'z': 407.0,
                'pitch': 0.0,
                'roll': 0.0,
                'claw': 0.0,
                'time_ms': 2000,
            },
            'pose': {
                'x': 348.0,
                'y': 0.0,
                'z': 407.0,
                'pitch': 0.0,
                'roll': 0.0,
                'claw': 0.0,
                'time_ms': 2000,
            },
        },
        SCENE4_SHELF_LEVEL2: {
            'target_z_m': SCENE4_SHELF_LEVEL2_Z_M,
            'pose': {
                'x': 330.0,
                'y': 0.0,
                'z': 190.0,
                'pitch': 0.0,
                'roll': 0.0,
                'claw': 0.0,
                'time_ms': 2000,
            },
        },
    },
}
DEFAULT_SCENE4_PLACE = {
    'default_destination': SCENE4_PLACE_FRAME,
    'targets': {
        'red': SCENE4_PLACE_FRAME,
        'green': SCENE4_PLACE_FRAME,
        'yellow': SCENE4_PLACE_FRAME,
        'blue': SCENE4_PLACE_FRAME,
    },
}
DEFAULT_SCENE4_KINEMATICS = {
    'params': [182.45, 225.0, 36.97, 145.0, 0.0, 130.23, 0.0, 50.0, 142.5],
    'linkage1_mm': 110.45,
    'linkage1_delta_mm': 72.0,
    'base_high_mm': 70.5,
    'base_high_delta_mm': 72.0,
}
DEFAULT_SCENE5_CALIBRATION_POSE = {
    'x': 220.0,
    'y': 0.0,
    'z': 230.0,
    'pitch': -90.0,
    'roll': 0.0,
    'claw': 0.0,
    'time_ms': 1000,
}
DEFAULT_SCENE5_GRID = {
    'color_slots': list(SCENE5_COLOR_KEYS),
    'waste_slots': list(SCENE5_WASTE_KEYS),
}
SCENE5_PLACE_SLOT_TARGETS = [
    [-0.060, -0.130],
    [0.060, -0.130],
    [-0.060, -0.230],
    [0.060, -0.230],
    [-0.050, -0.360],
    [0.060, -0.360],
    [-0.050, -0.460],
    [0.060, -0.460],
]
SCENE5_ARM_A_NAMESPACE = '/arm_a'
SCENE5_ARM_B_NAMESPACE = '/arm_b'
SCENE5_ARM_A_PREFIX = f'{SCENE5_ARM_A_NAMESPACE}/ros_robot_controller'
SCENE5_ARM_B_PREFIX = f'{SCENE5_ARM_B_NAMESPACE}/ros_robot_controller'
SCENE5_CONVEYOR_TOPIC = f'{SCENE5_ARM_B_PREFIX}/conveyor/set'
DEFAULT_CONTROLLER_PREFIX = '/ros_robot_controller'
COLOR_BLOCK_SIZE_M = 0.030
WASTE_CARD_SIZE_M = 0.040
DEFAULT_SCENE5_DUAL_ARM = {
    'arm_a': {
        'label': 'A机械臂',
        'role': 'recognize_pick_and_fixed_place',
        'namespace': '/arm_a',
        'controller_prefix': SCENE5_ARM_A_PREFIX,
    },
    'arm_b': {
        'label': 'B机械臂',
        'role': 'recognize_pick_and_place',
        'namespace': '/arm_b',
        'controller_prefix': SCENE5_ARM_B_PREFIX,
    },
    'arm_a_vision': {
        'color_object_height_m': COLOR_BLOCK_SIZE_M,
        'garbage_object_height_m': WASTE_CARD_SIZE_M,
    },
    'conveyor': {
        'owner': 'arm_b',
        'topic': SCENE5_CONVEYOR_TOPIC,
        'speed': -50,
        'stop_speed': 0,
        'move_ms': 1200,
        'settle_ms': 500,
    },
    'arm_b_fixed_pick': {
        'enabled': True,
        'x': 200.0,
        'y': 15.0,
        'z': 80.0,
        'approach_z_offset': 10.0,
        'lift_z_offset': 30.0,
        'transfer_x': 200.0,
        'transfer_y': 0.0,
        'transfer_z': 200.0,
        'pitch': -90.0,
        'roll': 0.0,
        'pre_grab_roll': 0.0,
        'open_claw': -60.0,
        'close_claw': -35.0,
        'trigger_center_x': 320.0,
        'trigger_center_y': 280.0,
        'trigger_tolerance_x': 20.0,
        'trigger_tolerance_y': 120.0,
    },
    'arm_a_points': {
        'home': {'x': 220.0, 'y': 0.0, 'z': 230.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -75.0, 'time_ms': 1000},
        'pick': {'x': 180.0, 'y': 120.0, 'z': 65.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -75.0, 'time_ms': 1000},
        'pick_close': {'x': 180.0, 'y': 120.0, 'z': 65.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -28.0, 'time_ms': 600},
        'pick_lift': {'x': 180.0, 'y': 120.0, 'z': 160.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -28.0, 'time_ms': 900},
        'place_approach': {'x': 0.0, 'y': 200.0, 'z': 200.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -28.0, 'time_ms': 1000},
        'place': {'x': 0.0, 'y': 180.0, 'z': 100.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -28.0, 'time_ms': 900},
        'put_on_conveyor': {'x': 180.0, 'y': -80.0, 'z': 80.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -28.0, 'time_ms': 1200},
        'release': {'x': 0.0, 'y': 180.0, 'z': 100.0, 'pitch': -90.0, 'roll': 0.0, 'claw': -75.0, 'time_ms': 600},
    },
    'arm_b_place_targets': {
        'color': {
            'red': [-60.0, -230.0],
            'green': [60.0, -230.0],
            'blue': [60.0, -130.0],
            'yellow': [-60.0, -130.0],
        },
        'waste': {
            'food_waste': [-0.05, -0.360, 0.27],
            'hazardous_waste': [-0.05, -0.460, 0.27],
            'recyclable_waste': [0.06, -0.360, 0.27],
            'residual_waste': [0.06, -0.460, 0.27],
        },
    },
}
DEFAULT_PLACE_POLICY = {
    'only_left_y_positive': True,
    'min_place_z': 0.015,
}
PLACE_OFFSET_LIMIT_M = 0.010
DEFAULT_GLOBAL_PLACE_OFFSET = {'x': 0.0, 'y': 0.0, 'z': 0.0}
SCENE5_CONVEYOR_SPEED_PRESETS = (
    ('低速', -20),
    ('中速', -50),
    ('高速', -100),
)


def clamp_float(value, default=0.0, minimum=None, maximum=None):
    try:
        parsed = float(value)
    except Exception:
        parsed = float(default)
    if minimum is not None:
        parsed = max(float(minimum), parsed)
    if maximum is not None:
        parsed = min(float(maximum), parsed)
    return parsed


def normalize_global_place_offset(value):
    raw = value if isinstance(value, dict) else {}
    limit = PLACE_OFFSET_LIMIT_M
    return {
        'x': clamp_float(raw.get('x', raw.get('offset_x', 0.0)), 0.0, -limit, limit),
        'y': clamp_float(raw.get('y', raw.get('offset_y', 0.0)), 0.0, -limit, limit),
        'z': clamp_float(raw.get('z', raw.get('offset_z', 0.0)), 0.0, -limit, limit),
    }

DEFAULT_SCENE_PLACE_TARGETS = {
    'center': [0.235, 0.0, 0.015],
    'left_top': [0.285, 0.16, 0.015],
    'right_top': [0.285, -0.16, 0.015],
    'left_bottom': [0.115, 0.16, 0.015],
    'right_bottom': [0.115, -0.16, 0.015],
    'red': [0.087, 0.133, 0.015],
    'green': [0.017, 0.133, 0.015],
    'blue': [-0.053, 0.133, 0.015],
    'tag1': [-0.053, 0.063, 0.015],
    'tag2': [0.017, 0.063, 0.015],
    'tag3': [0.087, 0.063, 0.015],
    'residual_waste': [0.095, 0.214, 0.02],
    'food_waste': [0.040, 0.214, 0.02],
    'hazardous_waste': [-0.018, 0.214, 0.02],
    'recyclable_waste': [-0.070, 0.214, 0.02],
    'tag_stackup': [0.017, 0.063, 0.015],
}
DEFAULT_SCENE2_COLOR_TARGETS = {
    'yellow': [0.157, 0.133, 0.015],
    'red': list(DEFAULT_SCENE_PLACE_TARGETS['red']),
    'green': list(DEFAULT_SCENE_PLACE_TARGETS['green']),
    'blue': list(DEFAULT_SCENE_PLACE_TARGETS['blue']),
}
TARGET_KEY_MAP = {
    'Y': 'yellow',
    'R': 'red',
    'G': 'green',
    'B': 'blue',
    '1': 'tag1',
    '2': 'tag2',
    '3': 'tag3',
    'Residual Waste': 'residual_waste',
    'Food Waste': 'food_waste',
    'Hazardous Waste': 'hazardous_waste',
    'Recyclable Waste': 'recyclable_waste',
    'Center': 'center',
    'Left Top': 'left_top',
    'Right Top': 'right_top',
    'Left Bottom': 'left_bottom',
    'Right Bottom': 'right_bottom',
}
TARGET_KEY_TO_LABEL = {v: k for k, v in TARGET_KEY_MAP.items()}


def env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def scene_home_pose(prefix, x=110.0, y=0.0, z=220.0, pitch=-90.0, roll=0.0, claw=0.0, time_ms=None):
    pose = {
        'x': env_float(f'{prefix}_HOME_X', x),
        'y': env_float(f'{prefix}_HOME_Y', y),
        'z': env_float(f'{prefix}_HOME_Z', z),
        'pitch': env_float(f'{prefix}_HOME_PITCH', pitch),
        'roll': env_float(f'{prefix}_HOME_ROLL', roll),
        'claw': env_float(f'{prefix}_HOME_CLAW', claw),
    }
    if time_ms is not None:
        pose['time_ms'] = int(env_float(f'{prefix}_HOME_TIME_MS', time_ms))
    return pose


DEFAULT_SCENE_HOME_POSES = {
    SCENE0_ID: scene_home_pose('CALIBRATION_SCENE0', x=200.0, y=0.0, z=200.0, pitch=-90.0, time_ms=2000),
    DEFAULT_SCENE_ID: scene_home_pose('CALIBRATION_SCENE1', x=120.0, y=0.0, z=210.0, pitch=-90.0),
    SCENE2_ID: scene_home_pose('CALIBRATION_SCENE2', x=120.0, y=0.0, z=210.0, pitch=-90.0),
    SCENE3_ID: scene_home_pose('CALIBRATION_SCENE3', x=210.0, y=0.0, z=235.0, pitch=-90.0),
    SCENE4_ID: {
        'x': 145.0,
        'y': 0.0,
        'z': 290.0,
        'pitch': -90.0,
        'roll': 0.0,
        'claw': 0.0,
    },
    SCENE5_ID: dict(DEFAULT_SCENE5_CALIBRATION_POSE),
}


def normalize_scene5_arm_role(role):
    role = str(role or 'A').strip().upper()
    return role if role in ('A', 'B') else 'A'


def active_scene_id(default=SCENE0_ID):
    scene_id = configured_scene_env()
    if scene_id:
        return scene_id
    try:
        with open(SCENE_YAML_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        return str(cfg.get('current_scene', default))
    except Exception:
        return default


def scene5_controller_prefix(scene_id=None):
    if (scene_id or active_scene_id()) != SCENE5_ID:
        return DEFAULT_CONTROLLER_PREFIX
    role = normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE'))
    return SCENE5_ARM_A_PREFIX if role == 'A' else SCENE5_ARM_B_PREFIX


def scene5_calibration_controller_prefix(scene_id=None):
    if (scene_id or active_scene_id()) != SCENE5_ID:
        return DEFAULT_CONTROLLER_PREFIX
    return SCENE5_ARM_A_PREFIX


def scene5_calibration_allowed(scene_id=None):
    return (scene_id or active_scene_id()) != SCENE5_ID or normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE')) == 'A'


def controller_topic(prefix, suffix):
    return f'{prefix.rstrip("/")}/{suffix.lstrip("/")}'


def scene5_service_name(namespace, suffix):
    return f'{namespace.rstrip("/")}/{suffix.lstrip("/")}'


def scene5_arm_a_service(suffix):
    return scene5_service_name(SCENE5_ARM_A_NAMESPACE, suffix)


def scene5_arm_b_service(suffix):
    return scene5_service_name(SCENE5_ARM_B_NAMESPACE, suffix)


def scene5_namespaced_topic(namespace, suffix):
    return scene5_service_name(namespace, suffix)


def normalized_home_pose(value, default):
    if not isinstance(value, dict):
        value = {}
    pose = {
        'x': float(value.get('x', default['x'])),
        'y': float(value.get('y', default['y'])),
        'z': float(value.get('z', default['z'])),
        'pitch': float(value.get('pitch', default['pitch'])),
        'roll': float(value.get('roll', default['roll'])),
        'claw': float(value.get('claw', default['claw'])),
    }
    if 'time_ms' in value or 'time_ms' in default:
        pose['time_ms'] = int(float(value.get('time_ms', default.get('time_ms', 1500))))
    return pose


def normalized_position(value, default):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        value = default
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except Exception:
        return [float(default[0]), float(default[1]), float(default[2])]


def normalized_unique_slots(slots, keys):
    clean = []
    for key in slots if isinstance(slots, (list, tuple)) else list(keys):
        if key in keys and key not in clean:
            clean.append(key)
    for key in keys:
        if key not in clean:
            clean.append(key)
    return clean[:len(keys)]


def normalized_position_slot_targets(value, defaults):
    raw_slots = value if isinstance(value, (list, tuple)) else []
    clean = []
    for index, default in enumerate(defaults):
        raw = raw_slots[index] if index < len(raw_slots) else default
        clean.append(normalized_position(raw, default))
    return clean[:len(defaults)]


def scene3_keys_for_group(group):
    return list(SCENE2_COLOR_KEYS if group == SCENE3_GROUP_COLOR else WASTE_KEYS)


def scene3_labels_for_group(group):
    return SCENE2_COLOR_LABELS if group == SCENE3_GROUP_COLOR else WASTE_LABELS


def scene3_colors_for_group(group):
    return SCENE2_COLOR_QCOLORS if group == SCENE3_GROUP_COLOR else WASTE_QCOLORS


def scene3_default_targets_for_group(group):
    defaults = SCENE3_COLOR_TARGETS if group == SCENE3_GROUP_COLOR else SCENE3_WASTE_TARGETS
    return [list(defaults[key]) for key in scene3_keys_for_group(group)]


def normalized_scene3_grid(value):
    raw = value if isinstance(value, dict) else {}
    grid = {}
    for group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE):
        keys = scene3_keys_for_group(group)
        slot_field = SCENE3_GRID_SLOT_FIELDS[group]
        target_field = SCENE3_GRID_TARGET_FIELDS[group]
        grid[slot_field] = normalized_unique_slots(raw.get(slot_field), keys)
        grid[target_field] = normalized_position_slot_targets(
            raw.get(target_field),
            scene3_default_targets_for_group(group),
        )
    return grid


def normalized_calibration_pose(value, default):
    if not isinstance(value, dict):
        value = {}
    pose = {}
    for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw'):
        try:
            pose[key] = float(value.get(key, default[key]))
        except Exception:
            pose[key] = float(default[key])
    try:
        pose['time_ms'] = int(value.get('time_ms', default['time_ms']))
    except Exception:
        pose['time_ms'] = int(default['time_ms'])
    return pose


def normalized_pose_with_default(value, default):
    if not isinstance(value, dict):
        value = {}
    pose = {}
    for key in ('x', 'y', 'z', 'pitch', 'roll', 'claw'):
        pose[key] = _normalized_float(value.get(key), default[key])
    pose['time_ms'] = _normalized_int(value.get('time_ms'), default.get('time_ms', 2000), 1)
    return pose


def _merge_missing_dict(target, default):
    if not isinstance(target, dict):
        target = {}
    for key, value in default.items():
        if isinstance(value, dict):
            target[key] = _merge_missing_dict(target.get(key), value)
        elif key not in target:
            target[key] = value
    return target


def _normalized_int(value, default, minimum=None, maximum=None):
    try:
        parsed = int(value)
    except Exception:
        parsed = int(default)
    if minimum is not None:
        parsed = max(int(minimum), parsed)
    if maximum is not None:
        parsed = min(int(maximum), parsed)
    return parsed


def _normalized_float(value, default, minimum=None, maximum=None):
    try:
        parsed = float(value)
    except Exception:
        parsed = float(default)
    if minimum is not None:
        parsed = max(float(minimum), parsed)
    if maximum is not None:
        parsed = min(float(maximum), parsed)
    return parsed


def normalized_scene4_pick(value):
    pick = _merge_missing_dict(value, yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_PICK)))
    pick['active_zone'] = SCENE4_PICK_ZONE_LOWER

    lower = pick.setdefault(SCENE4_PICK_ZONE_LOWER, {})
    lower['view_pose'] = normalized_pose_with_default(
        lower.get('view_pose'),
        DEFAULT_SCENE4_PICK[SCENE4_PICK_ZONE_LOWER]['view_pose'],
    )
    lower['use_plane_calibration'] = bool(lower.get('use_plane_calibration', True))
    lower_default = DEFAULT_SCENE4_PICK[SCENE4_PICK_ZONE_LOWER]['detection']
    lower_detection = lower.setdefault('detection', {})
    min_v = _normalized_int(lower_detection.get('min_v'), lower_default['min_v'], 0)
    max_v = _normalized_int(lower_detection.get('max_v'), lower_default['max_v'], 0)
    if min_v > max_v:
        min_v, max_v = max_v, min_v
    lower['detection'] = {
        'min_v': min_v,
        'max_v': max_v,
    }
    return {
        'active_zone': SCENE4_PICK_ZONE_LOWER,
        SCENE4_PICK_ZONE_LOWER: lower,
    }


def normalized_scene4_shelf(value):
    raw = value if isinstance(value, dict) else {}
    raw_slots = raw.get('rail_slots', {}) if isinstance(raw.get('rail_slots'), dict) else {}
    rail_slots = {}
    for factor, defaults in DEFAULT_SCENE4_SHELF['rail_slots'].items():
        raw_factor = raw_slots.get(str(factor), raw_slots.get(factor, {}))
        raw_factor = raw_factor if isinstance(raw_factor, dict) else {}
        rail_slots[str(factor)] = {}
        for slot_key, default_value in defaults.items():
            rail_slots[str(factor)][slot_key] = _normalized_int(raw_factor.get(slot_key), default_value, 0)
    shelf = {
        'length_m': _normalized_float(raw.get('length_m'), DEFAULT_SCENE4_SHELF['length_m'], 0.001),
        'width_m': _normalized_float(raw.get('width_m'), DEFAULT_SCENE4_SHELF['width_m'], 0.001),
        'level_match_tolerance_m': _normalized_float(
            raw.get('level_match_tolerance_m'),
            DEFAULT_SCENE4_SHELF['level_match_tolerance_m'],
            0.0,
        ),
        'rail_slots': rail_slots,
        'levels': {},
    }
    for level_key in SCENE4_SHELF_LEVELS:
        default_level = DEFAULT_SCENE4_SHELF['levels'][level_key]
        level = {
            'target_z_m': _normalized_float(default_level.get('target_z_m'), default_level['target_z_m'], 0.0),
            'pose': normalized_pose_with_default(default_level.get('pose'), default_level['pose']),
        }
        default_approach = default_level.get('approach_pose')
        if default_approach is not None:
            level['approach_pose'] = normalized_pose_with_default(default_approach, default_approach)
        shelf['levels'][level_key] = level
    shelf['upper_z_m'] = shelf['levels'][SCENE4_SHELF_LEVEL1]['target_z_m']
    shelf['lower_z_m'] = shelf['levels'][SCENE4_SHELF_LEVEL2]['target_z_m']
    shelf['level_gap_m'] = abs(shelf['upper_z_m'] - shelf['lower_z_m'])
    return shelf

def normalized_scene4_place(value):
    raw = value if isinstance(value, dict) else {}
    valid = (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS)
    default_destination = str(raw.get('default_destination', DEFAULT_SCENE4_PLACE['default_destination']))
    if default_destination not in valid:
        default_destination = DEFAULT_SCENE4_PLACE['default_destination']

    targets = dict(DEFAULT_SCENE4_PLACE['targets'])
    raw_targets = raw.get('targets', {}) if isinstance(raw.get('targets'), dict) else {}
    for key, destination in raw_targets.items():
        if str(key) not in scene4_keys_for_mode(SCENE4_MODE_ALL):
            continue
        destination = str(destination)
        if destination in valid:
            targets[str(key)] = destination
    return {
        'default_destination': default_destination,
        'targets': targets,
    }

def scene4_keys_for_mode(mode):
    if mode == SCENE4_MODE_WASTE:
        return []
    return list(SCENE4_COLOR_KEYS)


def scene4_labels_for_mode(mode):
    if mode == SCENE4_MODE_WASTE:
        return {}
    return SCENE2_COLOR_LABELS


def scene4_colors_for_mode(mode):
    if mode == SCENE4_MODE_WASTE:
        return {}
    return SCENE2_COLOR_QCOLORS


def scene4_default_position(key):
    if key in SCENE4_COLOR_TARGETS:
        return list(SCENE4_COLOR_TARGETS[key])
    if key in SCENE4_WASTE_TARGETS:
        return list(SCENE4_WASTE_TARGETS[key])
    return list(DEFAULT_SCENE_PLACE_TARGETS.get(key, [0.0, 0.0, 0.015]))


def normalized_scene4_frame_slots(slots, keys):
    keys = list(keys)
    slot_count = SCENE4_FRAME_SLOT_COUNT
    raw_slots = slots if isinstance(slots, (list, tuple)) else []
    clean = []
    used = set()
    for raw in raw_slots[:slot_count]:
        key = str(raw) if raw is not None else ''
        if not key:
            clean.append('')
        elif key in keys and key not in used:
            clean.append(key)
            used.add(key)
        else:
            clean.append('')
    while len(clean) < slot_count:
        clean.append('')
    for key in keys:
        if key in used:
            continue
        try:
            empty_index = clean.index('')
            clean[empty_index] = key
        except ValueError:
            clean.append(key)
        used.add(key)
    while len(clean) < slot_count:
        clean.append('')
    return clean[:slot_count]


def scene4_shelf_slot_count(keys):
    return SCENE4_SHELF_SLOT_COUNT


def scene4_default_shelf_slots(keys, destination):
    slot_count = scene4_shelf_slot_count(keys)
    slots = [''] * slot_count
    y_values = SCENE4_SHELF_SLOT_Y_VALUES
    for key in keys:
        pos = scene4_default_position(key)
        if destination == SCENE4_SHELF_LEVEL1 and abs(float(pos[2]) - SCENE4_SHELF_LEVEL1_Z_M) > 1e-6:
            continue
        if destination == SCENE4_SHELF_LEVEL2 and abs(float(pos[2]) - SCENE4_SHELF_LEVEL2_Z_M) > 1e-6:
            continue
        y = float(pos[1])
        index = min(range(len(y_values)), key=lambda i: abs(y_values[i] - y))
        if index < slot_count:
            slots[index] = key
    return slots


def expanded_legacy_scene4_shelf_list(raw_slots, middle_value):
    if SCENE4_SHELF_SLOT_COUNT == 4 and len(raw_slots) == 2:
        return [raw_slots[0], middle_value, middle_value, raw_slots[1]]
    return raw_slots


def normalized_scene4_shelf_slots(slots, keys, destination):
    keys = list(keys)
    slot_count = scene4_shelf_slot_count(keys)
    raw_slots = slots if isinstance(slots, (list, tuple)) else scene4_default_shelf_slots(keys, destination)
    raw_slots = expanded_legacy_scene4_shelf_list(raw_slots, '')
    clean = []
    used = set()
    for raw in raw_slots[:slot_count]:
        key = str(raw) if raw is not None else ''
        if key in keys and key not in used:
            clean.append(key)
            used.add(key)
        else:
            clean.append('')
    while len(clean) < slot_count:
        clean.append('')
    return clean[:slot_count]


def normalized_scene4_position(value, default):
    raw = value if isinstance(value, (list, tuple)) else []
    pos = []
    for idx, fallback in enumerate(default):
        try:
            pos.append(round(float(raw[idx]), 3))
        except Exception:
            pos.append(round(float(fallback), 3))
    return pos[:3]


def normalized_scene4_position_slots(value, defaults, expand_legacy_shelf=False):
    raw_slots = value if isinstance(value, (list, tuple)) else []
    if expand_legacy_shelf:
        raw_slots = expanded_legacy_scene4_shelf_list(raw_slots, None)
    slots = []
    for index, default in enumerate(defaults):
        raw = raw_slots[index] if index < len(raw_slots) else default
        slots.append(normalized_scene4_position(raw, default))
    return slots


def normalized_scene4_absolute_positions(value):
    raw = value if isinstance(value, dict) else {}
    return {
        'frame_slots': normalized_scene4_position_slots(raw.get('frame_slots'), DEFAULT_SCENE4_ABSOLUTE_POSITIONS['frame_slots']),
        'upper_shelf_slots': normalized_scene4_position_slots(
            raw.get('upper_shelf_slots'),
            DEFAULT_SCENE4_ABSOLUTE_POSITIONS['upper_shelf_slots'],
            expand_legacy_shelf=True,
        ),
        'lower_shelf_slots': normalized_scene4_position_slots(
            raw.get('lower_shelf_slots'),
            DEFAULT_SCENE4_ABSOLUTE_POSITIONS['lower_shelf_slots'],
            expand_legacy_shelf=True,
        ),
    }


def scene4_frame_slot_target(index, absolute_positions=None):
    absolute_positions = normalized_scene4_absolute_positions(absolute_positions)
    index = max(0, min(SCENE4_FRAME_SLOT_COUNT - 1, int(index)))
    return list(absolute_positions['frame_slots'][index])


def scene4_shelf_slot_target(destination, index, absolute_positions=None):
    absolute_positions = normalized_scene4_absolute_positions(absolute_positions)
    field = 'upper_shelf_slots' if destination == SCENE4_SHELF_LEVEL1 else 'lower_shelf_slots'
    index = max(0, min(SCENE4_SHELF_SLOT_COUNT - 1, int(index)))
    return list(absolute_positions[field][index])


def scene4_target_side(key):
    color_key = SCENE4_WASTE_TARGET_MAP.get(key, key)
    return 'right' if color_key in ('green', 'blue') else 'left'


def scene4_shelf_fixed_position(key, destination, slot_index=None, absolute_positions=None):
    destination = destination if destination in SCENE4_SHELF_LEVELS else SCENE4_SHELF_LEVEL1
    if slot_index is None:
        slot_index = SCENE4_SHELF_SLOT_COUNT - 1 if scene4_target_side(key) == 'right' else 0
    return scene4_shelf_slot_target(destination, slot_index, absolute_positions)


def scene4_fixed_position(key, destination, frame_slot_index=None, shelf_slot_index=None, absolute_positions=None):
    if destination in SCENE4_SHELF_LEVELS:
        return scene4_shelf_fixed_position(key, destination, shelf_slot_index, absolute_positions)
    if destination == SCENE4_PLACE_FRAME:
        keys = list(WASTE_KEYS) if key in WASTE_KEYS else list(SCENE4_COLOR_KEYS)
        fixed_slot_index = frame_slot_index
        if fixed_slot_index is None:
            fixed_slot_index = keys.index(key) if key in keys else 0
        return scene4_frame_slot_target(fixed_slot_index, absolute_positions)
    return scene4_default_position(key)


def read_yaml_dict(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def write_yaml_dict(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data or {}, f, sort_keys=False, allow_unicode=True)


def scene_play_id(scene_id):
    return SCENE_PLAY_IDS.get(scene_id, scene_id)


def scene_play_config_path(scene_id):
    return SCENE_PLAY_CONFIG_PATHS.get(scene_id)


def extract_scene_play_config(scene_id, scene):
    play = {'play_id': scene_play_id(scene_id)}
    if not isinstance(scene, dict):
        return play
    for key in SCENE_PLAY_CONFIG_KEYS.get(scene_id, ()):
        if key in scene:
            play[key] = copy.deepcopy(scene[key])
    return play


def apply_scene_play_config(scene_id, scene):
    path = scene_play_config_path(scene_id)
    if not path or not isinstance(scene, dict):
        return scene
    play = read_yaml_dict(path)
    if not play:
        return scene
    for key in SCENE_PLAY_CONFIG_KEYS.get(scene_id, ()):
        if key in play:
            scene[key] = copy.deepcopy(play[key])
    scene['play_id'] = play.get('play_id', scene_play_id(scene_id))
    return scene


def strip_scene_play_config(scene_id, scene):
    clean = copy.deepcopy(scene) if isinstance(scene, dict) else {}
    for key in SCENE_PLAY_CONFIG_KEYS.get(scene_id, ()):
        clean.pop(key, None)
    clean['play_id'] = scene_play_id(scene_id)
    return clean


DEFAULT_SCENE_CONFIG = {
    'current_scene': DEFAULT_CURRENT_SCENE,
    'global_place_offset': dict(DEFAULT_GLOBAL_PLACE_OFFSET),
    'scenes': {
        SCENE0_ID: {
            'name': BUILTIN_SCENE_NAMES[SCENE0_ID],
            'mode': 'arm_body',
            'length_m': 0.130,
            'width_m': 0.167,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'yaw_deg': 0.0,
                'center_in_map_m': {
                    'x': -0.045,
                    'y': 0.0635,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[SCENE0_ID]),
            'kinematics': {
                'params': list(DEFAULT_BODY_KINEMATICS_PARAMS),
            },
            'place_policy': {
                'only_left_y_positive': False,
                'min_place_z': DEFAULT_PLACE_POLICY['min_place_z'],
            },
            'place_targets': dict(DEFAULT_SCENE_PLACE_TARGETS, yellow=list(DEFAULT_SCENE2_COLOR_TARGETS['yellow'])),
        },
        DEFAULT_SCENE_ID: {
            'name': BUILTIN_SCENE_NAMES[DEFAULT_SCENE_ID],
            'length_m': 0.170,
            'width_m': 0.158,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'center_in_map_m': {
                    'x': -0.065,
                    'y': 0.064,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[DEFAULT_SCENE_ID]),
            'place_policy': dict(DEFAULT_PLACE_POLICY),
            'place_targets': dict(DEFAULT_SCENE_PLACE_TARGETS),
        },
        SCENE2_ID: {
            'name': BUILTIN_SCENE_NAMES[SCENE2_ID],
            'length_m': 0.170,
            'width_m': 0.158,
            'use_calibration_scene': DEFAULT_SCENE_ID,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'center_in_map_m': {
                    'x': -0.065,
                    'y': 0.064,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[SCENE2_ID]),
            'place_policy': dict(DEFAULT_PLACE_POLICY),
            'place_targets': dict(DEFAULT_SCENE_PLACE_TARGETS, yellow=list(DEFAULT_SCENE2_COLOR_TARGETS['yellow'])),
            'color_grid': {
                'slots': list(SCENE2_COLOR_KEYS),
                'slot_targets': [list(DEFAULT_SCENE2_COLOR_TARGETS[key]) for key in SCENE2_SLOT_TARGET_KEYS],
            },
        },
        SCENE3_ID: {
            'name': BUILTIN_SCENE_NAMES[SCENE3_ID],
            'length_m': 0.263,
            'width_m': 0.263,
            'use_calibration_scene': DEFAULT_SCENE_ID,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'center_in_map_m': {
                    'x': -0.1115,
                    'y': 0.1115,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[SCENE3_ID]),
            'place_policy': dict(DEFAULT_PLACE_POLICY),
            'place_targets': dict(
                DEFAULT_SCENE_PLACE_TARGETS,
                **{k: list(v) for k, v in SCENE3_COLOR_TARGETS.items()},
                **{k: list(v) for k, v in SCENE3_WASTE_TARGETS.items()},
            ),
            'place_pitch': dict(SCENE3_PLACE_PITCH),
            'scene3_grid': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE3_GRID)),
        },
        SCENE4_ID: {
            'name': BUILTIN_SCENE_NAMES[SCENE4_ID],
            'length_m': SCENE4_CALIBRATION_LENGTH_M,
            'width_m': SCENE4_CALIBRATION_WIDTH_M,
            'use_calibration_scene': SCENE4_ID,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'yaw_deg': 0.0,
                'center_in_map_m': {
                    'x': -0.1115,
                    'y': 0.1115,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[SCENE4_ID]),
            'place_policy': {
                'only_left_y_positive': False,
                'min_place_z': DEFAULT_PLACE_POLICY['min_place_z'],
            },
            'place_targets': {
                **{k: list(v) for k, v in DEFAULT_SCENE_PLACE_TARGETS.items() if k not in WASTE_KEYS},
                **{k: list(v) for k, v in SCENE4_DEFAULT_ALL_TARGETS.items()},
            },
            'rail': dict(DEFAULT_SCENE4_RAIL),
            'calibration_pose': dict(DEFAULT_SCENE4_CALIBRATION_POSE),
            'scene4_pick': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_PICK)),
            'scene4_place': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_PLACE)),
            'scene4_absolute_positions': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_ABSOLUTE_POSITIONS)),
            'kinematics': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_KINEMATICS)),
            'scene4_grid': {
                'color_slots': list(SCENE4_COLOR_KEYS),
                'all_slots': list(SCENE4_COLOR_KEYS),
                'color_upper_slots': ['red', '', '', 'green'],
                'color_lower_slots': ['yellow', '', '', 'blue'],
                'all_upper_slots': ['red', '', '', 'green'],
                'all_lower_slots': ['yellow', '', '', 'blue'],
                'color_slot_targets': [list(SCENE4_COLOR_TARGETS[key]) for key in SCENE4_COLOR_KEYS],
            },
            'scene4_shelf': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_SHELF)),
        },
        SCENE5_ID: {
            'name': BUILTIN_SCENE_NAMES[SCENE5_ID],
            'mode': 'dual_arm_single_conveyor',
            'length_m': 0.263,
            'width_m': 0.263,
            'use_calibration_scene': SCENE4_ID,
            'calibration_tag': {
                'id': 1,
                'size_m': 0.04,
                'effective_size_m': 0.033,
                'yaw_deg': 0.0,
                'center_in_map_m': {
                    'x': -0.1115,
                    'y': 0.1115,
                    'z': 0.0,
                },
            },
            'home_pose': dict(DEFAULT_SCENE_HOME_POSES[SCENE5_ID]),
            'calibration_pose': dict(DEFAULT_SCENE5_CALIBRATION_POSE),
            'place_policy': {
                'only_left_y_positive': False,
                'min_place_z': DEFAULT_PLACE_POLICY['min_place_z'],
            },
            'place_targets': {
                **DEFAULT_SCENE_PLACE_TARGETS,
                **{k: list(DEFAULT_SCENE_PLACE_TARGETS[k]) for k in SCENE2_COLOR_KEYS if k in DEFAULT_SCENE_PLACE_TARGETS},
                'yellow': list(DEFAULT_SCENE2_COLOR_TARGETS['yellow']),
                **{k: list(DEFAULT_SCENE_PLACE_TARGETS[k]) for k in WASTE_KEYS},
            },
            'scene5_grid': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_GRID)),
            'scene5_dual_arm': yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM)),
        },
    },
}


TARGET_POSITION_MAP = {
    'Center': [0.235, 0.0, 0.015],
    'Left Top': [0.285, 0.16, 0.015],
    'Right Top': [0.285, -0.16, 0.015],
    'Left Bottom': [0.115, 0.16, 0.015],
    'Right Bottom': [0.115, -0.16, 0.015],
    'Y': [0.157, 0.133, 0.015],
    'R': [0.087, 0.133, 0.015],
    'G': [0.017, 0.133, 0.015],
    'B': [-0.053, 0.133, 0.015],
    '1': [-0.053, 0.063, 0.015],
    '2': [0.017, 0.063, 0.015],
    '3': [0.087, 0.063, 0.015],
    'Residual Waste': [0.112, -0.125, 0.015],
    'Food Waste': [0.05, -0.125, 0.015],
    'Hazardous Waste': [0.017, -0.125, 0.015],
    'Recyclable Waste': [-0.045, -0.125, 0.015],
}


TARGET_LABEL_ALIASES = {
    '中心': 'Center',
    '左上': 'Left Top',
    '右上': 'Right Top',
    '左下': 'Left Bottom',
    '右下': 'Right Bottom',
    '其他垃圾': 'Residual Waste',
    '厨余垃圾': 'Food Waste',
    '有害垃圾': 'Hazardous Waste',
    '可回收垃圾': 'Recyclable Waste',
}


chassis_type = os.environ.get('CHASSIS_TYPE', '')
APP_SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
APP_POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration.yaml"
STEPPER_POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration.yaml"
if chassis_type == 'Slide_Rails' and DEFAULT_CURRENT_SCENE != SCENE5_ID:
    POSITIONS_YAML_PATH = STEPPER_POSITIONS_YAML_PATH
    SCENE_YAML_PATH = STEPPER_SCENE_YAML_PATH
else:
    POSITIONS_YAML_PATH = APP_POSITIONS_YAML_PATH
    SCENE_YAML_PATH = APP_SCENE_YAML_PATH


def save_positions_yaml(data):
    paths = [POSITIONS_YAML_PATH]
    if active_scene_id() == SCENE4_ID:
        paths.extend([APP_POSITIONS_YAML_PATH, STEPPER_POSITIONS_YAML_PATH])
    for path in dict.fromkeys(paths):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)


init_finish = False
SCENE2_RESULT_IMAGE_TOPIC = os.environ.get('SCENE2_RESULT_IMAGE_TOPIC', '/object_sorting/image_result')
SCENE2_RGB_IMAGE_TOPIC = os.environ.get('SCENE2_RGB_IMAGE_TOPIC', '/depth_cam/rgb/image_raw')
_calib_ns = (
    ('/arm_a' if normalize_scene5_arm_role(os.environ.get('SCENE5_ARM_ROLE')) == 'A' else '/arm_b')
    if active_scene_id() == SCENE5_ID else ''
)
CALIB_DISPLAY_IMAGE_TOPIC = os.environ.get('CALIB_DISPLAY_IMAGE_TOPIC', f'{_calib_ns}/calibration/display_image' if _calib_ns else '/calibration/display_image')
CALIB_DEPTH_IMAGE_TOPIC = os.environ.get('CALIB_DEPTH_IMAGE_TOPIC', f'{_calib_ns}/calibration/depth_image' if _calib_ns else '/calibration/depth_image')
CALIB_RGB_IMAGE_TOPIC = os.environ.get('CALIB_RGB_IMAGE_TOPIC', f'{_calib_ns}/depth_cam/rgb/image_raw' if _calib_ns else '/depth_cam/rgb/image_raw')
SCENE3_COLOR_IMAGE_TOPIC = os.environ.get('SCENE3_COLOR_IMAGE_TOPIC', '/object_sorting/image_result')
SCENE3_WASTE_IMAGE_TOPIC = os.environ.get('SCENE3_WASTE_IMAGE_TOPIC', '/waste_classification/image_result')
SCENE5_WASTE_IMAGE_TOPIC = os.environ.get('SCENE5_WASTE_IMAGE_TOPIC', '/waste_classification_motor_depth/result_image')
SCENE5_ARM_A_IMAGE_TOPIC = os.environ.get(
    'SCENE5_ARM_A_IMAGE_TOPIC',
    (
        '/arm_a/scene5_arm_a_loader/image_result'
        if active_scene_id() == SCENE5_ID
        else '/scene5_arm_a_loader/image_result'
    ),
)
SCENE5_ARM_B_IMAGE_TOPIC = os.environ.get(
    'SCENE5_ARM_B_IMAGE_TOPIC',
    (
        '/arm_b/waste_classification_motor_depth/result_image'
        if active_scene_id() == SCENE5_ID
        else SCENE5_WASTE_IMAGE_TOPIC
    ),
)
SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC = os.environ.get(
    'SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC',
    '/arm_b/waste_classification_motor_depth/result_image/compressed'
    if active_scene_id() == SCENE5_ID
    else f'{SCENE5_WASTE_IMAGE_TOPIC}/compressed',
)

_image_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class ArmControlNode(Node):
    def __init__(self, name):
        global init_finish
        if not rclpy.ok():
            rclpy.init()
        super().__init__(name)
        self.controller_prefix = scene5_calibration_controller_prefix()
        self.arm_pub = self.create_publisher(
            ArmCoords,
            controller_topic(self.controller_prefix, 'arm/set_coords'),
            5,
        )
        self.calibration_enable_client = self.create_client(SetBool, 'calibration/start_calibration')
        self.enter_calibration_client = self.create_client(Trigger, 'calibration/enter')
        self.start_calibration_client = self.create_client(Trigger, 'calibration/start')
        self.exit_calibration_client = self.create_client(Trigger, 'calibration/exit')
        self.object_sort_enter_client = self.create_client(Trigger, 'object_sorting/enter')
        self.object_sort_exit_client = self.create_client(Trigger, 'object_sorting/exit')
        self.object_sort_enable_client = self.create_client(SetBool, 'object_sorting/enable_sorting')
        self.object_sort_target_client = self.create_client(SetStringBool, 'object_sorting/set_target')
        self.waste_enter_client = self.create_client(Trigger, 'waste_classification/enter')
        self.waste_exit_client = self.create_client(Trigger, 'waste_classification/exit')
        self.waste_enable_client = self.create_client(SetBool, 'waste_classification/enable_transport')
        self.waste_target_client = self.create_client(SetStringList, 'waste_classification/set_target')
        self.scene_runtime_prepare_client = self.create_client(Trigger, '/ros_robot_controller/scene_runtime/prepare')
        self.grab_calib_client = self.create_client(Trigger, 'calibration/grab_calibration')
        self.clear_grab_calib_client = self.create_client(Trigger, 'calibration/clear_grab_calibration')
        self.scene5_waste_slot_order_client = self.create_client(
            SetStringList,
            scene5_arm_b_service('scene5_waste_classification/set_slot_order'),
        )
        self.scene5_cycle_client = self.create_client(Trigger, scene5_arm_a_service('scene5/arm_a/load_once_then_b'))
        self.scene5_arm_a_home_client = self.create_client(Trigger, scene5_arm_a_service('scene5/arm_a/home'))
        self.scene5_arm_a_start_client = self.create_client(Trigger, scene5_arm_a_service('scene5/arm_a/start'))
        self.scene5_arm_a_stop_client = self.create_client(Trigger, scene5_arm_a_service('scene5/arm_a/stop'))
        self.scene5_arm_a_load_client = self.create_client(Trigger, scene5_arm_a_service('scene5/arm_a/load_once'))
        self.scene5_conveyor_pub = self.create_publisher(Int8, SCENE5_CONVEYOR_TOPIC, 1)
        _click_topic = f'{_calib_ns}/calibration/ui_click' if _calib_ns else '/calibration/ui_click'
        self.calib_click_pub = self.create_publisher(RosString, _click_topic, 1)
        self.scene5_conveyor_speed = int(DEFAULT_SCENE5_DUAL_ARM['conveyor']['speed'])
        self.scene5_conveyor_enabled = False
        self.scene5_place_targets_client = self.create_client(
            SetString,
            scene5_arm_b_service('scene5_waste_classification/set_place_targets'),
        )
        self.scene5_b_enter_client = self.create_client(Trigger, scene5_arm_b_service('scene5_waste_classification/enter'))
        self.scene5_b_enable_client = self.create_client(SetBool, scene5_arm_b_service('scene5_waste_classification/enable_transport'))
        self.scene5_b_exit_client = self.create_client(Trigger, scene5_arm_b_service('scene5_waste_classification/exit'))
        self.scene5_b_fixed_pick_client = self.create_client(
            SetString,
            scene5_arm_b_service('scene5_waste_classification/set_fixed_pick'),
        )
        self.scene1_processed_image_callback = None
        self.scene1_rgb_image_callback = None
        self.scene2_result_image_callback = None
        self.scene2_waste_image_callback = None
        self.scene2_rgb_image_callback = None
        self.scene3_color_image_callback = None
        self.scene3_waste_image_callback = None
        self.scene3_rgb_image_callback = None
        self.scene4_result_image_callback = None
        self.scene4_waste_image_callback = None
        self.scene4_rgb_image_callback = None
        self.scene5_arm_a_image_callback = None
        self.scene5_arm_b_image_callback = None
        self.scene5_waste_image_callback = None
        self.scene5_rgb_image_callback = None
        self.calib_result_image_callback = None
        self.calib_depth_image_callback = None
        self.calib_rgb_image_callback = None
        self.result_image_sub = self.create_subscription(
            Image,
            SCENE2_RESULT_IMAGE_TOPIC,
            lambda msg: self._emit_scene2_image('result', msg),
            _image_qos,
        )
        self.rgb_image_sub = self.create_subscription(
            Image,
            SCENE2_RGB_IMAGE_TOPIC,
            lambda msg: self._emit_scene2_image('rgb', msg),
            _image_qos,
        )
        self.waste_image_sub = self.create_subscription(
            Image,
            SCENE3_WASTE_IMAGE_TOPIC,
            lambda msg: self._emit_scene3_image('waste', msg),
            _image_qos,
        )
        self.scene5_arm_a_image_sub = self.create_subscription(
            Image,
            SCENE5_ARM_A_IMAGE_TOPIC,
            lambda msg: self._emit_scene5_image('arm_a', msg),
            _image_qos,
        )
        self.scene5_arm_b_image_sub = self.create_subscription(
            CompressedImage,
            SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC,
            lambda msg: self._emit_scene5_image('arm_b', msg),
            _image_qos,
        )
        self.calib_result_image_sub = self.create_subscription(
            Image,
            CALIB_DISPLAY_IMAGE_TOPIC,
            lambda msg: self._emit_calib_image('result', msg),
            _image_qos,
        )
        self.calib_depth_image_sub = self.create_subscription(
            Image,
            CALIB_DEPTH_IMAGE_TOPIC,
            lambda msg: self._emit_calib_image('depth', msg),
            _image_qos,
        )
        if CALIB_RGB_IMAGE_TOPIC != SCENE2_RGB_IMAGE_TOPIC:
            self.calib_rgb_image_sub = self.create_subscription(
                Image,
                CALIB_RGB_IMAGE_TOPIC,
                lambda msg: self._emit_calib_image('rgb', msg),
                _image_qos,
            )
        wait_start = time.time()
        while self.arm_pub.get_subscription_count() == 0:
            if time.time() - wait_start > 5.0:
                self.get_logger().error('等待 ros_robot_controller 订阅超时，请确认底层控制节点已启动')
                break
            self.get_logger().info('等待 ros_robot_controller 订阅...')
            time.sleep(0.5)
        init_finish = True

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

    def send_request(self, client, msg, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'服务不可用: {client.srv_name}')
            return None
        future = client.call_async(msg)
        deadline = time.time() + max(0.1, float(timeout_sec))
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f'服务调用超时: {client.srv_name}')
            return None
        return future.result()

    def enter_calibration(self):
        return self.send_request(self.enter_calibration_client, Trigger.Request(), timeout_sec=90.0)

    def exit_calibration(self):
        return self.send_request(self.exit_calibration_client, Trigger.Request())

    def start_calibration(self):
        # scene5 UI always controls arm A calibration, even when it runs on arm B.
        return self.send_request(self.start_calibration_client, Trigger.Request())

    def enable_calibration(self, enable):
        msg = SetBool.Request()
        msg.data = bool(enable)
        return self.send_request(self.calibration_enable_client, msg)

    def grab_calibration(self):
        return self.send_request(self.grab_calib_client, Trigger.Request(), timeout_sec=5.0)

    def clear_grab_calibration(self):
        return self.send_request(self.clear_grab_calib_client, Trigger.Request(), timeout_sec=5.0)

    def prepare_scene_runtime(self):
        return self.send_request(self.scene_runtime_prepare_client, Trigger.Request(), timeout_sec=45.0)

    def init_pose(self, home):
        time_ms = int(float(home.get('time_ms', 1000)))
        self.publish_arm(home['x'], home['y'], home['z'], home['pitch'], home['roll'], home['claw'], time_ms)
        time.sleep(max(0.0, time_ms / 1000.0))
        self.publish_arm(home['x'], home['y'], home['z'], home['pitch'], home['roll'], home['claw'], time_ms)
        time.sleep(max(0.0, time_ms / 1000.0))

    def set_position(self, position, roll_deg, home, time_ms=1500):
        x_mm = float(position[0]) * 1000.0
        y_mm = float(position[1]) * 1000.0
        z_mm = float(position[2]) * 1000.0
        self.publish_arm(x_mm, y_mm, z_mm, home['pitch'], float(roll_deg), home['claw'], time_ms)

    def set_scene1_image_callbacks(self, processed_callback=None, rgb_callback=None):
        self.scene1_processed_image_callback = processed_callback
        self.scene1_rgb_image_callback = rgb_callback

    def set_scene2_image_callbacks(self, result_callback=None, waste_callback=None, rgb_callback=None):
        self.scene2_result_image_callback = result_callback
        self.scene2_waste_image_callback = waste_callback
        self.scene2_rgb_image_callback = rgb_callback

    def set_scene3_image_callbacks(self, color_callback=None, waste_callback=None, rgb_callback=None):
        self.scene3_color_image_callback = color_callback
        self.scene3_waste_image_callback = waste_callback
        self.scene3_rgb_image_callback = rgb_callback

    def set_scene4_image_callbacks(self, result_callback=None, waste_callback=None, rgb_callback=None):
        self.scene4_result_image_callback = result_callback
        self.scene4_waste_image_callback = waste_callback
        self.scene4_rgb_image_callback = rgb_callback

    def set_scene5_image_callbacks(self, arm_a_callback=None, arm_b_callback=None, waste_callback=None, rgb_callback=None):
        self.scene5_arm_a_image_callback = arm_a_callback or rgb_callback
        self.scene5_arm_b_image_callback = arm_b_callback or waste_callback
        self.scene5_waste_image_callback = self.scene5_arm_b_image_callback
        self.scene5_rgb_image_callback = self.scene5_arm_a_image_callback

    def publish_calib_click(self, mode, x, y):
        msg = RosString()
        msg.data = _json.dumps({'mode': mode, 'x': float(x), 'y': float(y)})
        self.calib_click_pub.publish(msg)

    def set_calib_image_callbacks(self, result_callback=None, depth_callback=None, rgb_callback=None):
        self.calib_result_image_callback = result_callback
        self.calib_depth_image_callback = depth_callback
        self.calib_rgb_image_callback = rgb_callback

    def _emit_calib_image(self, image_type, msg):
        if image_type == 'result' and self.calib_result_image_callback is not None:
            self.calib_result_image_callback(msg)
        elif image_type == 'depth' and self.calib_depth_image_callback is not None:
            self.calib_depth_image_callback(msg)
        elif image_type == 'rgb' and self.calib_rgb_image_callback is not None:
            self.calib_rgb_image_callback(msg)

    def _emit_scene2_image(self, image_type, msg):
        if image_type == 'result':
            if self.scene1_processed_image_callback is not None:
                self.scene1_processed_image_callback(('color', msg))
            if self.scene2_result_image_callback is not None:
                self.scene2_result_image_callback(msg)
            if self.scene3_color_image_callback is not None:
                self.scene3_color_image_callback(msg)
            if self.scene4_result_image_callback is not None:
                self.scene4_result_image_callback(msg)
        elif image_type == 'rgb':
            if self.scene1_rgb_image_callback is not None:
                self.scene1_rgb_image_callback(msg)
            if self.scene2_rgb_image_callback is not None:
                self.scene2_rgb_image_callback(msg)
            if self.scene3_rgb_image_callback is not None:
                self.scene3_rgb_image_callback(msg)
            if self.scene4_rgb_image_callback is not None:
                self.scene4_rgb_image_callback(msg)
            if self.scene5_rgb_image_callback is not None:
                self.scene5_rgb_image_callback(msg)
            if self.calib_rgb_image_callback is not None and CALIB_RGB_IMAGE_TOPIC == SCENE2_RGB_IMAGE_TOPIC:
                self.calib_rgb_image_callback(msg)

    def _emit_scene3_image(self, image_type, msg):
        if image_type == 'waste':
            if self.scene1_processed_image_callback is not None:
                self.scene1_processed_image_callback(('waste', msg))
            if self.scene2_waste_image_callback is not None:
                self.scene2_waste_image_callback(msg)
            if self.scene3_waste_image_callback is not None:
                self.scene3_waste_image_callback(msg)
            if self.scene4_waste_image_callback is not None:
                self.scene4_waste_image_callback(msg)

    def _emit_scene5_image(self, image_type, msg):
        if image_type in ('arm_a', 'rgb') and self.scene5_arm_a_image_callback is not None:
            self.scene5_arm_a_image_callback(msg)
        elif image_type in ('arm_b', 'waste') and self.scene5_arm_b_image_callback is not None:
            self.scene5_arm_b_image_callback(msg)

    def stop_color_sorting(self):
        feedback = []
        req = SetBool.Request()
        req.data = False
        res = self.send_request(self.object_sort_enable_client, req)
        if res is not None:
            feedback.append('enable_sorting=false')
        for key in OBJECT_SORT_KEYS:
            msg = SetStringBool.Request()
            msg.data_str = key
            msg.data_bool = False
            res = self.send_request(self.object_sort_target_client, msg)
            if res is not None:
                feedback.append(f'{key}=false')
        res = self.send_request(self.object_sort_exit_client, Trigger.Request())
        if res is not None:
            feedback.append('exit')
        return bool(feedback), ' | '.join(feedback) if feedback else '物体分拣服务无响应'

    def stop_waste_classification(self):
        feedback = []
        req = SetBool.Request()
        req.data = False
        res = self.send_request(self.waste_enable_client, req)
        if res is not None:
            feedback.append('waste_enable=false')
        res = self.send_request(self.waste_exit_client, Trigger.Request())
        if res is not None:
            feedback.append('waste_exit')
        return bool(feedback), ' | '.join(feedback) if feedback else '垃圾分类服务无响应'

    def stop_scene3_tasks(self):
        color_ok, color_msg = self.stop_color_sorting()
        waste_ok, waste_msg = self.stop_waste_classification()
        return color_ok or waste_ok, f'{color_msg} | {waste_msg}'

    def start_color_sorting(self, target_key=None, stop_all=False):
        label = '全部颜色' if target_key is None else SCENE2_COLOR_LABELS.get(target_key, target_key)
        self.get_logger().info(f'[Scene4ColorStart] request: target={label}, stop_all={stop_all}')
        if stop_all:
            self.stop_scene3_tasks()
        else:
            self.stop_color_sorting()
        res = self.send_request(self.object_sort_enter_client, Trigger.Request())
        if res is None or not bool(getattr(res, 'success', False)):
            self.get_logger().warn('[Scene4ColorStart] object_sorting/enter failed')
            return False, 'object_sorting/enter failed'
        self.get_logger().info('[Scene4ColorStart] object_sorting/enter ok')
        active = set(SCENE2_COLOR_KEYS if target_key is None else [target_key])
        for key in OBJECT_SORT_KEYS:
            msg = SetStringBool.Request()
            msg.data_str = key
            msg.data_bool = key in active
            res = self.send_request(self.object_sort_target_client, msg)
            if res is None or not bool(getattr(res, 'success', False)):
                self.get_logger().warn(f'[Scene4ColorStart] object_sorting/set_target failed: {key}')
                return False, f'object_sorting/set_target failed: {key}'
        self.get_logger().info(f'[Scene4ColorStart] targets ok: {sorted(active)}')
        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.object_sort_enable_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            self.get_logger().warn('[Scene4ColorStart] object_sorting/enable_sorting failed')
            return False, 'object_sorting/enable_sorting failed'
        self.get_logger().info(f'[Scene4ColorStart] enabled ok: {label}')
        return True, f'已开启颜色分拣: {label}'

    def start_tag_sorting(self, target_key=None):
        self.stop_scene3_tasks()
        res = self.send_request(self.object_sort_enter_client, Trigger.Request())
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'object_sorting/enter failed'
        active = set(TAG_KEYS if target_key is None else [target_key])
        for key in OBJECT_SORT_KEYS:
            msg = SetStringBool.Request()
            msg.data_str = key
            msg.data_bool = key in active
            res = self.send_request(self.object_sort_target_client, msg)
            if res is None or not bool(getattr(res, 'success', False)):
                return False, f'object_sorting/set_target failed: {key}'
        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.object_sort_enable_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'object_sorting/enable_sorting failed'
        label = '全部标签' if target_key is None else TAG_LABELS.get(target_key, target_key)
        return True, f'已开启标签夹取: {label}'

    def start_waste_classification(self, target_key=None):
        self.stop_scene3_tasks()
        res = self.send_request(self.waste_enter_client, Trigger.Request())
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/enter failed'
        req_target = SetStringList.Request()
        req_target.data = list(WASTE_KEYS if target_key is None else [target_key])
        res = self.send_request(self.waste_target_client, req_target)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/set_target failed'
        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.waste_enable_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/enable_transport failed'
        label = '全部垃圾' if target_key is None else WASTE_LABELS.get(target_key, target_key)
        return True, f'已开启垃圾分类: {label}'

    def start_color_and_waste_sorting(self):
        self.stop_scene3_tasks()
        res = self.send_request(self.object_sort_enter_client, Trigger.Request())
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'object_sorting/enter failed'
        for key in OBJECT_SORT_KEYS:
            msg = SetStringBool.Request()
            msg.data_str = key
            msg.data_bool = key in SCENE2_COLOR_KEYS
            res = self.send_request(self.object_sort_target_client, msg)
            if res is None or not bool(getattr(res, 'success', False)):
                return False, f'object_sorting/set_target failed: {key}'
        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.object_sort_enable_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'object_sorting/enable_sorting failed'

        res = self.send_request(self.waste_enter_client, Trigger.Request())
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/enter failed'
        req_target = SetStringList.Request()
        req_target.data = list(WASTE_KEYS)
        res = self.send_request(self.waste_target_client, req_target)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/set_target failed'
        req = SetBool.Request()
        req.data = True
        res = self.send_request(self.waste_enable_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, 'waste_classification/enable_transport failed'
        return True, '已开启色块和垃圾一起夹取'

    def stop_scene5_tasks(self):
        return self.stop_scene5_pipeline()

    def call_scene5_trigger(self, client, label, timeout_sec=8.0):
        res = self.send_request(client, Trigger.Request(), timeout_sec=timeout_sec)
        if res is None or not bool(getattr(res, 'success', False)):
            msg = getattr(res, 'message', 'no response') if res is not None else 'no response'
            return False, f'{label} failed: {msg}'
        return True, getattr(res, 'message', label)

    def call_scene5_set_bool(self, client, value, label, timeout_sec=8.0):
        req = SetBool.Request()
        req.data = bool(value)
        res = self.send_request(client, req, timeout_sec=timeout_sec)
        if res is None or not bool(getattr(res, 'success', False)):
            msg = getattr(res, 'message', 'no response') if res is not None else 'no response'
            return False, f'{label} failed: {msg}'
        return True, getattr(res, 'message', label)

    def scene5_b_services_ready(self, timeout_sec=1.0):
        required = (
            (self.scene5_b_enter_client, 'B进入准备服务'),
            (self.scene5_b_enable_client, 'B启停夹取服务'),
        )
        missing = [
            label for client, label in required
            if not client.wait_for_service(timeout_sec=timeout_sec)
        ]
        if missing:
            return False, 'B机械臂服务未就绪，请确认B机启动并且scene5桥接已运行: ' + ', '.join(missing)
        return True, ''

    def _prepare_scene5_b_settings(self, slot_order=None, place_targets=None):
        _ok, place_msg = self.set_scene5_b_place_targets(place_targets)
        if not _ok:
            self.get_logger().warn(f'[scene5_b] set_place_targets (non-fatal): {place_msg}')
        _ok, slot_msg = self._set_scene5_slot_order(
            self.scene5_waste_slot_order_client,
            slot_order or list(SCENE5_WASTE_KEYS),
            'scene5_waste_classification',
        )
        if not _ok:
            self.get_logger().warn(f'[scene5_b] set_slot_order (non-fatal): {slot_msg}')
        return True, ' | '.join(msg for msg in (place_msg, slot_msg) if msg)

    def start_scene5_pipeline(self, slot_order=None, place_targets=None):
        ok, ready_msg = self.scene5_b_services_ready(timeout_sec=1.5)
        if not ok:
            return False, ready_msg
        ok, slot_msg = self._prepare_scene5_b_settings(slot_order, place_targets)
        if not ok:
            return False, slot_msg
        ok, a_msg = self.call_scene5_trigger(self.scene5_arm_a_start_client, 'arm_a start', timeout_sec=30.0)
        if not ok:
            return False, a_msg
        ok, b_enter_msg = self.call_scene5_trigger(self.scene5_b_enter_client, 'arm_b enter', timeout_sec=30.0)
        if not ok:
            self.call_scene5_trigger(self.scene5_arm_a_stop_client, 'arm_a stop', timeout_sec=5.0)
            return False, b_enter_msg
        ok, b_start_msg = self.call_scene5_set_bool(
            self.scene5_b_enable_client,
            True,
            'arm_b enable_transport',
            timeout_sec=30.0,
        )
        if not ok:
            self.call_scene5_trigger(self.scene5_arm_a_stop_client, 'arm_a stop', timeout_sec=5.0)
            return False, b_start_msg
        self.scene5_conveyor_enabled = True
        self.publish_scene5_conveyor(self.scene5_conveyor_speed)
        suffix = f' ({slot_msg})' if slot_msg else ''
        return True, f'{a_msg} | {b_enter_msg} | {b_start_msg} | conveyor speed={self.scene5_conveyor_speed}{suffix}'

    def run_scene5_one_cycle(self, slot_order=None, place_targets=None):
        ok, slot_msg = self._prepare_scene5_b_settings(slot_order, place_targets)
        if not ok:
            return False, slot_msg
        ok, msg = self.call_scene5_trigger(self.scene5_cycle_client, 'scene5 one cycle')
        suffix = f' ({slot_msg})' if slot_msg else ''
        return ok, f'{msg}{suffix}'

    def stop_scene5_pipeline(self):
        self.scene5_conveyor_enabled = False
        self.publish_scene5_conveyor(0)
        b_ok, b_msg = self.scene5_arm_b_stop()
        a_ok, a_msg = self.scene5_arm_a_stop()
        ok = bool(b_ok and a_ok)
        return ok, ' | '.join(m for m in (b_msg, a_msg, 'conveyor stopped on arm_b') if m)

    def scene5_arm_a_home(self):
        return self.call_scene5_trigger(self.scene5_arm_a_home_client, 'arm_a home')

    def scene5_arm_a_start(self):
        return self.call_scene5_trigger(self.scene5_arm_a_start_client, 'arm_a start')

    def scene5_arm_a_stop(self):
        return self.call_scene5_trigger(self.scene5_arm_a_stop_client, 'arm_a stop')

    def scene5_arm_a_load_once(self):
        return self.call_scene5_trigger(self.scene5_arm_a_load_client, 'arm_a load_once', timeout_sec=30.0)

    def scene5_arm_b_enter(self, slot_order=None, place_targets=None):
        ok, ready_msg = self.scene5_b_services_ready(timeout_sec=1.5)
        if not ok:
            return False, ready_msg
        ok, slot_msg = self._prepare_scene5_b_settings(slot_order, place_targets)
        if not ok:
            return False, slot_msg
        ok, msg = self.call_scene5_trigger(self.scene5_b_enter_client, 'arm_b enter', timeout_sec=30.0)
        suffix = f' ({slot_msg})' if slot_msg else ''
        return ok, f'{msg}{suffix}'

    def scene5_arm_b_start(self, slot_order=None, place_targets=None):
        ok, msg = self.scene5_arm_b_enter(slot_order, place_targets)
        if not ok:
            return False, msg
        ok, start_msg = self.call_scene5_set_bool(
            self.scene5_b_enable_client,
            True,
            'arm_b enable_transport',
            timeout_sec=30.0,
        )
        return ok, f'{msg} | {start_msg}'

    def scene5_arm_b_stop(self):
        ok, disable_msg = self.call_scene5_set_bool(
            self.scene5_b_enable_client,
            False,
            'arm_b disable_transport',
            timeout_sec=5.0,
        )
        if not ok:
            self.get_logger().warn(f'[scene5_b_stop] disable_transport (non-fatal): {disable_msg}')
        exit_ok, exit_msg = self.call_scene5_trigger(
            self.scene5_b_exit_client,
            'arm_b exit',
            timeout_sec=5.0,
        )
        if not exit_ok:
            self.get_logger().warn(f'[scene5_b_stop] exit (non-fatal): {exit_msg}')
        return True, ' | '.join(m for m in (disable_msg, exit_msg) if m)

    def start_scene5_conveyor(self):
        self.scene5_conveyor_enabled = True
        self.publish_scene5_conveyor(self.scene5_conveyor_speed)
        return True, f'conveyor started on arm_b speed={self.scene5_conveyor_speed}'

    def stop_scene5_conveyor(self):
        self.scene5_conveyor_enabled = False
        self.publish_scene5_conveyor(0)
        return True, 'conveyor stopped on arm_b'

    def set_scene5_conveyor_speed(self, speed):
        self.scene5_conveyor_speed = int(max(-127, min(127, int(speed))))
        if self.scene5_conveyor_enabled:
            self.publish_scene5_conveyor(self.scene5_conveyor_speed)
        return True, f'conveyor speed={self.scene5_conveyor_speed}'

    def publish_scene5_conveyor(self, speed):
        msg = Int8()
        msg.data = int(max(-127, min(127, int(speed))))
        self.scene5_conveyor_pub.publish(msg)

    def set_scene5_b_place_targets(self, place_targets):
        if not place_targets:
            return True, ''
        req = SetString.Request()
        req.data = json.dumps(place_targets, ensure_ascii=False)
        res = self.send_request(self.scene5_place_targets_client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            msg = getattr(res, 'message', 'no response') if res is not None else 'no response'
            return False, f'scene5_waste_classification/set_place_targets failed: {msg}'
        return True, getattr(res, 'message', 'set place targets')

    def set_scene5_b_fixed_pick(self, params):
        req = SetString.Request()
        req.data = json.dumps(params, ensure_ascii=False)
        res = self.send_request(self.scene5_b_fixed_pick_client, req, timeout_sec=5.0)
        if res is None or not bool(getattr(res, 'success', False)):
            msg = getattr(res, 'message', 'no response') if res is not None else 'no response'
            return False, f'set_fixed_pick failed: {msg}'
        return True, getattr(res, 'message', 'fixed_pick updated')

    def _set_scene5_slot_order(self, client, slots, label):
        if not slots:
            return True, ''
        req = SetStringList.Request()
        req.data = list(slots)
        res = self.send_request(client, req)
        if res is None or not bool(getattr(res, 'success', False)):
            return False, f'{label}/set_slot_order failed'
        return True, f'{label}_slot_order={",".join(slots)}'

class ClickablePreviewLabel(QLabel):
    """支持鼠标点击的预览标签，点击时发出归一化坐标 (fx, fy) 信号 [0,1]."""
    clicked_at = pyqtSignal(float, float)

    def __init__(self, text='', parent=None):
        super().__init__(text, parent)
        self.setCursor(Qt.CrossCursor)
        self._pixmap_actual = None

    def setPixmap(self, pixmap):
        self._pixmap_actual = pixmap
        super().setPixmap(pixmap)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self._pixmap_actual and not self._pixmap_actual.isNull():
            lw, lh = float(self.width()), float(self.height())
            pw = float(self._pixmap_actual.width())
            ph = float(self._pixmap_actual.height())
            if pw > 0 and ph > 0:
                scale = min(lw / pw, lh / ph)
                iw, ih = pw * scale, ph * scale
                ox = (lw - iw) / 2.0
                oy = (lh - ih) / 2.0
                cx = event.x() - ox
                cy = event.y() - oy
                if 0.0 <= cx <= iw and 0.0 <= cy <= ih:
                    self.clicked_at.emit(cx / iw, cy / ih)
        super().mousePressEvent(event)


class ColorGridWidget(QWidget):
    gridChanged = pyqtSignal(list)
    colorClicked = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 190)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.item_keys = tuple(SCENE2_COLOR_KEYS)
        self.item_labels = dict(SCENE2_COLOR_LABELS)
        self.item_colors = dict(SCENE2_COLOR_QCOLORS)
        self.draggable = True
        self.slots = list(SCENE2_COLOR_KEYS)
        self.drag_index = None
        self.hover_index = None
        self.drag_pos = QPointF(0.0, 0.0)
        self.press_pos = QPointF(0.0, 0.0)
        self.drag_moved = False
        self.fixed_layout = True

    def _cell_rect(self, index):
        cols = 2
        rows = max(1, int(math.ceil(len(self.slots) / float(cols))))
        cell_w = self.width() / float(cols)
        cell_h = self.height() / float(rows)
        row = index // cols
        col = index % cols
        x = col * cell_w
        y = row * cell_h
        card_w = max(70.0, min(cell_w - 26.0, cell_w * 0.68))
        card_h = max(40.0, min(cell_h - 24.0, cell_h * 0.52))
        return QRectF(x + (cell_w - card_w) * 0.5, y + (cell_h - card_h) * 0.5, card_w, card_h)

    def set_items(self, keys, labels, colors, draggable=False):
        clean = list(keys)
        self.item_keys = tuple(clean)
        self.item_labels = dict(labels)
        self.item_colors = dict(colors)
        self.draggable = bool(draggable)
        self.drag_index = None
        self.hover_index = None
        self.set_slots(clean)

    def set_slots(self, slots):
        clean = []
        for key in slots:
            if key in self.item_keys and key not in clean:
                clean.append(key)
        for key in self.item_keys:
            if key not in clean:
                clean.append(key)
        self.slots = clean[:len(self.item_keys)]
        self.update()

    def cell_index_at(self, pos):
        cols = 2
        rows = max(1, int(math.ceil(len(self.slots) / float(cols))))
        cell_w = max(1, self.width() // cols)
        cell_h = max(1, self.height() // rows)
        col = min(1, max(0, int(pos.x() // cell_w)))
        row = min(rows - 1, max(0, int(pos.y() // cell_h)))
        return min(len(self.slots) - 1, row * cols + col)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_index = self.cell_index_at(event.pos())
            self.drag_pos = QPointF(event.pos())
            self.press_pos = QPointF(event.pos())
            self.drag_moved = False
            self.setCursor(Qt.ClosedHandCursor)
            self.update()

    def mouseMoveEvent(self, event):
        index = self.cell_index_at(event.pos())
        if self.drag_index is None:
            if index != self.hover_index:
                self.hover_index = index
                self.update()
            self.setCursor(Qt.OpenHandCursor)
            return
        self.drag_pos = QPointF(event.pos())
        self.hover_index = index
        if (self.drag_pos - self.press_pos).manhattanLength() > 4.0:
            self.drag_moved = True
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.drag_index is not None:
            drop_index = self.cell_index_at(event.pos())
            if self.draggable and drop_index != self.drag_index:
                self.slots[self.drag_index], self.slots[drop_index] = self.slots[drop_index], self.slots[self.drag_index]
                self.gridChanged.emit(list(self.slots))
            elif not self.drag_moved:
                self.colorClicked.emit(self.slots[self.drag_index])
            self.drag_index = None
            self.setCursor(Qt.OpenHandCursor)
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        font = painter.font()
        font.setPointSize(16)
        font.setBold(True)
        painter.setFont(font)
        painter.fillRect(self.rect(), QColor(30, 31, 49))
        painter.setPen(QPen(QColor(74, 77, 94), 1))
        painter.drawLine(int(self.width() * 0.5), 12, int(self.width() * 0.5), self.height() - 12)
        if len(self.slots) > 2:
            rows = int(math.ceil(len(self.slots) / 2.0))
            for row in range(1, rows):
                y = int(self.height() * row / float(rows))
                painter.drawLine(12, y, self.width() - 12, y)

        for idx, color_key in enumerate(self.slots):
            if idx == self.drag_index and self.drag_moved:
                continue
            self._draw_color_card(painter, self._cell_rect(idx), color_key, idx == self.hover_index)
        if self.drag_index is not None and self.drag_moved:
            rect = self._cell_rect(self.drag_index)
            rect.moveCenter(self.drag_pos)
            self._draw_color_card(painter, rect, self.slots[self.drag_index], True, dragging=True)

    def _draw_color_card(self, painter, rect, color_key, active=False, dragging=False):
        fill = self.item_colors.get(color_key, QColor(180, 180, 180))
        shadow = rect.translated(0, 3)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 35 if active or dragging else 18))
        painter.drawRoundedRect(shadow, 10, 10)
        painter.setBrush(QBrush(fill))
        painter.setPen(QPen(QColor(15, 23, 42), 2 if active or dragging else 1))
        painter.drawRoundedRect(rect, 7, 7)
        font = painter.font()
        font.setPointSize(16)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(15, 23, 42))
        painter.drawText(rect, Qt.AlignCenter, self.item_labels.get(color_key, color_key))


class UprightWestTabBar(QTabBar):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setExpanding(False)

    def tabSizeHint(self, index):
        base = super().tabSizeHint(index)
        return QSize(max(116, base.width() + 20), max(50, base.height() + 12))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#1E1F31'))
        for index in range(self.count()):
            rect = self.tabRect(index).adjusted(0, 3, 0, -3)
            selected = index == self.currentIndex()
            painter.fillRect(rect, QColor('#343645') if selected else QColor('#1E1F31'))
            if selected:
                painter.fillRect(rect.left(), rect.top(), 4, rect.height(), QColor('#FA8F01'))
            font = painter.font()
            font.setBold(selected)
            painter.setFont(font)
            painter.setPen(QColor('#FFFFFF') if selected else QColor('#B0BEC5'))
            painter.drawText(rect.adjusted(10, 4, -8, -4), Qt.AlignCenter | Qt.TextWordWrap, self.tabText(index))


class Scene5PlaceMapWidget(QWidget):
    placeTargetsChanged = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 160)
        self.setMaximumHeight(340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setMouseTracking(True)
        self.place_targets = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets']))
        self.place_slots = self._slots_from_targets()
        self.drag_key = None
        self.drag_slot = None
        self.hover_key = None
        self.hover_slot = None
        self.drag_pos = QPointF(0.0, 0.0)
        self.press_pos = QPointF(0.0, 0.0)
        self.drag_moved = False

    def set_scene_targets(self, place_targets):
        if isinstance(place_targets, dict):
            self.place_targets = _merge_missing_dict(
                yaml.safe_load(yaml.safe_dump(place_targets)),
                yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'])),
            )
        else:
            self.place_targets = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets']))
        self.place_slots = self._slots_from_targets()
        self.update()

    def _keys(self):
        return list(SCENE5_COLOR_KEYS) + list(SCENE5_WASTE_KEYS)

    def _default_slots(self):
        return ['yellow', 'blue', 'red', 'green', 'food_waste', 'recyclable_waste', 'hazardous_waste', 'residual_waste']

    def _group_for_key(self, key):
        return 'color' if key in SCENE5_COLOR_KEYS else 'waste'

    def _labels_for_key(self, key):
        if key in SCENE5_COLOR_KEYS:
            return SCENE2_COLOR_LABELS.get(key, key), SCENE2_COLOR_QCOLORS.get(key, QColor(180, 180, 180))
        return WASTE_LABELS.get(key, key), WASTE_QCOLORS.get(key, QColor(180, 180, 180))

    def _raw_target(self, group, key):
        defaults = DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'].get(group, {})
        return self.place_targets.get(group, {}).get(key, defaults.get(key, [0.0, 0.0]))

    def _target_xy_m(self, key):
        group = self._group_for_key(key)
        raw = self._raw_target(group, key)
        try:
            x = float(raw[0])
            y = float(raw[1])
        except Exception:
            fallback = DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'][group][key]
            x = float(fallback[0])
            y = float(fallback[1])
        if group == 'color' or abs(x) > 2.0 or abs(y) > 2.0:
            x /= 1000.0
            y /= 1000.0
        return x, y

    def _target_z_m(self, key):
        group = self._group_for_key(key)
        if group == 'color':
            return 0.0
        raw = self._raw_target(group, key)
        try:
            return float(raw[2])
        except Exception:
            return float(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'][group][key][2])

    def _set_target_xy_m(self, key, x_m, y_m):
        group = self._group_for_key(key)
        targets = self.place_targets.setdefault(group, {})
        if group == 'color':
            targets[key] = [round(float(x_m) * 1000.0, 1), round(float(y_m) * 1000.0, 1)]
        else:
            targets[key] = [round(float(x_m), 3), round(float(y_m), 3), round(self._target_z_m(key), 3)]

    def _swap_key_positions(self, source_key, target_key):
        source_xy = self._target_xy_m(source_key)
        target_xy = self._target_xy_m(target_key)
        self._set_target_xy_m(source_key, target_xy[0], target_xy[1])
        self._set_target_xy_m(target_key, source_xy[0], source_xy[1])

    def _slots_from_targets(self):
        slots = [None] * len(SCENE5_PLACE_SLOT_TARGETS)
        used = set()
        keys = self._keys()
        for key in keys:
            x_m, y_m = self._target_xy_m(key)
            distances = sorted(
                (
                    ((x_m - target[0]) ** 2 + (y_m - target[1]) ** 2, index)
                    for index, target in enumerate(SCENE5_PLACE_SLOT_TARGETS)
                    if index not in used
                ),
                key=lambda item: item[0],
            )
            if not distances:
                break
            slot_index = distances[0][1]
            slots[slot_index] = key
            used.add(slot_index)
        remaining = [key for key in self._default_slots() if key in keys and key not in slots]
        for index, key in enumerate(slots):
            if key is None and remaining:
                slots[index] = remaining.pop(0)
        return slots

    def _sync_targets_from_slots(self):
        for index, key in enumerate(self.place_slots):
            if key not in self._keys() or index >= len(SCENE5_PLACE_SLOT_TARGETS):
                continue
            x_m, y_m = SCENE5_PLACE_SLOT_TARGETS[index]
            self._set_target_xy_m(key, x_m, y_m)

    def _map_rect(self):
        return QRectF(self.rect()).adjusted(10, 10, -10, -10)

    def _cell_rects(self):
        inner = self._map_rect()
        gap = 6.0
        cell_w = (inner.width() - gap) * 0.5
        cell_h = (inner.height() - gap * 3.0) * 0.25
        rects = []
        for row in range(4):
            for col in range(2):
                rects.append(QRectF(
                    inner.left() + col * (cell_w + gap),
                    inner.top() + row * (cell_h + gap),
                    cell_w,
                    cell_h,
                ))
        return rects

    def place_slot_index(self, pos):
        point = QPointF(pos)
        cells = self._cell_rects()
        for index, rect in enumerate(cells):
            if rect.contains(point):
                return index
        inner = self._map_rect()
        if not inner.contains(point):
            return None
        col = 0 if point.x() < inner.center().x() else 1
        row_h = inner.height() / 4.0
        row = max(0, min(3, int((point.y() - inner.top()) / max(row_h, 1.0))))
        return row * 2 + col

    def _slot_for_key(self, key):
        for index, slot_key in enumerate(self.place_slots):
            if slot_key == key:
                return index
        return None

    def _marker_rects(self):
        cells = self._cell_rects()
        rects = {}
        for index, key in enumerate(self.place_slots):
            if key not in self._keys() or index >= len(cells):
                continue
            label, _color = self._labels_for_key(key)
            cell = cells[index]
            marker_w = min(cell.width() - 8.0, 66.0 if len(label) > 2 else 48.0)
            marker_h = min(cell.height() - 4.0, 20.0)
            center = cell.center()
            if key == self.drag_key and self.drag_moved:
                center = self.drag_pos
            rects[key] = QRectF(center.x() - marker_w / 2.0, center.y() - marker_h / 2.0, marker_w, marker_h)
        return rects

    def _hit_test(self, pos):
        p = QPointF(pos)
        for key, rect in reversed(list(self._marker_rects().items())):
            if rect.contains(p):
                return key
        return None

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        key = self._hit_test(event.pos())
        if key is None:
            return
        self.drag_key = key
        self.drag_slot = self._slot_for_key(key)
        self.hover_key = key
        self.hover_slot = self.drag_slot
        self.drag_pos = QPointF(event.pos())
        self.press_pos = QPointF(event.pos())
        self.drag_moved = False
        self.setCursor(Qt.ClosedHandCursor)
        self.update()

    def mouseMoveEvent(self, event):
        self.drag_pos = QPointF(event.pos())
        hover_key = self._hit_test(event.pos())
        hover_slot = self.place_slot_index(event.pos())
        if self.drag_key is None:
            if hover_key != self.hover_key:
                self.hover_key = hover_key
                self.hover_slot = hover_slot
                self.update()
            self.setCursor(Qt.OpenHandCursor if hover_key else Qt.ArrowCursor)
            return
        if (self.drag_pos - self.press_pos).manhattanLength() > 4.0:
            self.drag_moved = True
        if hover_key != self.hover_key:
            self.hover_key = hover_key
        self.hover_slot = hover_slot
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.LeftButton or self.drag_key is None:
            return
        source_key = self.drag_key
        source_slot = self.drag_slot
        self.drag_key = None
        self.drag_slot = None
        drop_slot = self.place_slot_index(event.pos())
        if drop_slot is None:
            drop_slot = source_slot
        if (
            self.drag_moved
            and source_slot is not None
            and drop_slot is not None
            and drop_slot != source_slot
            and 0 <= int(drop_slot) < len(self.place_slots)
        ):
            self.place_slots[source_slot], self.place_slots[drop_slot] = self.place_slots[drop_slot], self.place_slots[source_slot]
            self._sync_targets_from_slots()
            self.placeTargetsChanged.emit(yaml.safe_load(yaml.safe_dump(self.place_targets)))
        self.drag_moved = False
        self.hover_key = self._hit_test(event.pos())
        self.hover_slot = drop_slot
        self.setCursor(Qt.OpenHandCursor if self.hover_key else Qt.ArrowCursor)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(30, 31, 49))

        inner = self._map_rect()
        painter.setPen(QPen(QColor(74, 77, 94), 1))
        painter.setBrush(QBrush(QColor(45, 47, 63)))
        painter.drawRoundedRect(inner, 4, 4)

        cells = self._cell_rects()
        for index, rect in enumerate(cells):
            active = index == self.hover_slot or (self.drag_key is not None and index == self.drag_slot)
            painter.setPen(QPen(QColor(250, 143, 1) if active else QColor(90, 93, 110), 1))
            painter.setBrush(QBrush(QColor(52, 54, 69) if active else QColor(37, 39, 53)))
            painter.drawRoundedRect(rect, 3, 3)

        rects = self._marker_rects()
        for key in self.place_slots:
            if key == self.drag_key and self.drag_moved:
                continue
            if key in rects:
                self._draw_marker(painter, rects[key], key, key == self.hover_key)
        if self.drag_key is not None and self.drag_moved:
            self._draw_marker(painter, rects[self.drag_key], self.drag_key, True, dragging=True)

    def _draw_marker(self, painter, rect, key, active=False, dragging=False):
        label, color = self._labels_for_key(key)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 40 if active or dragging else 24))
        painter.drawRoundedRect(rect.translated(0, 1), 5, 5)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(15, 23, 42), 2 if active or dragging else 1))
        painter.drawRoundedRect(rect, 5, 5)
        font = painter.font()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(17, 24, 39))
        painter.drawText(rect, Qt.AlignCenter, label)


class Scene3BoardWidget(QWidget):
    colorClicked = pyqtSignal(str)
    wasteClicked = pyqtSignal(str)
    colorGridChanged = pyqtSignal(list)
    wasteGridChanged = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.color_slots = list(SCENE2_COLOR_KEYS)
        self.waste_slots = list(WASTE_KEYS)
        self.hover_group = None
        self.hover_key = None
        self.hover_index = None
        self.drag_group = None
        self.drag_key = None
        self.drag_index = None
        self.press_pos = QPointF(0.0, 0.0)
        self.drag_pos = QPointF(0.0, 0.0)
        self.drag_offset = QPointF(0.0, 0.0)
        self.drag_moved = False
        self.interactive = True

    def set_interactive(self, enabled):
        self.interactive = bool(enabled)
        if not self.interactive:
            self.drag_group = None
            self.drag_key = None
            self.drag_index = None
            self.hover_group = None
            self.hover_key = None
            self.hover_index = None
            self.setCursor(Qt.ArrowCursor)
            self.update()

    def set_slots(self, color_slots=None, waste_slots=None):
        self.color_slots = normalized_unique_slots(
            color_slots if color_slots is not None else self.color_slots,
            SCENE2_COLOR_KEYS,
        )
        self.waste_slots = normalized_unique_slots(
            waste_slots if waste_slots is not None else self.waste_slots,
            WASTE_KEYS,
        )
        self.update()

    def _slots_for_group(self, group):
        return self.color_slots if group == SCENE3_GROUP_COLOR else self.waste_slots

    def _set_slots_for_group(self, group, slots):
        if group == SCENE3_GROUP_COLOR:
            self.color_slots = normalized_unique_slots(slots, SCENE2_COLOR_KEYS)
            self.colorGridChanged.emit(list(self.color_slots))
        else:
            self.waste_slots = normalized_unique_slots(slots, WASTE_KEYS)
            self.wasteGridChanged.emit(list(self.waste_slots))

    def _labels_for_group(self, group):
        return scene3_labels_for_group(group)

    def _colors_for_group(self, group):
        return scene3_colors_for_group(group)

    def _areas(self):
        gap = 18.0
        margin = 12.0
        w = max(1.0, float(self.width()) - margin * 2.0)
        h = max(1.0, float(self.height()) - margin * 2.0)
        left_w = (w - gap) * 0.5
        right_w = w - gap - left_w
        left = QRectF(margin, margin, left_w, h)
        right = QRectF(margin + left_w + gap, margin, right_w, h)
        return left, right

    def _group_area(self, group):
        left, right = self._areas()
        return left if group == SCENE3_GROUP_COLOR else right

    def _card_rects(self, area, slots):
        rects = []
        gap = 9.0
        card_area = area.adjusted(0, 28, 0, 0)
        cell_w = card_area.width()
        cell_h = (card_area.height() - gap * (len(slots) - 1)) / max(1, len(slots))
        for idx, _key in enumerate(slots):
            card_w = max(96.0, cell_w * 0.80)
            card_h = max(30.0, cell_h * 0.66)
            x = card_area.left() + (cell_w - card_w) * 0.5
            y = card_area.top() + idx * (cell_h + gap)
            rects.append(QRectF(x, y + (cell_h - card_h) * 0.5, card_w, card_h))
        return rects

    def _card_rect(self, group, index):
        slots = self._slots_for_group(group)
        rects = self._card_rects(self._group_area(group), slots)
        if index is None or index < 0 or index >= len(rects):
            return QRectF()
        return QRectF(rects[index])

    def _hit_test(self, pos):
        p = QPointF(pos)
        for group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE):
            slots = self._slots_for_group(group)
            rects = self._card_rects(self._group_area(group), slots)
            for index, rect in enumerate(rects):
                if rect.contains(p):
                    return group, slots[index], index
        return None, None, None

    def mouseMoveEvent(self, event):
        if not self.interactive:
            return
        pos = QPointF(event.pos())
        if self.drag_key is not None:
            self.drag_pos = pos
            if (self.drag_pos - self.press_pos).manhattanLength() > 4.0:
                self.drag_moved = True
            self.update()
            self.setCursor(Qt.PointingHandCursor)
            return
        group, key, index = self._hit_test(event.pos())
        if group != self.hover_group or key != self.hover_key or index != self.hover_index:
            self.hover_group = group
            self.hover_key = key
            self.hover_index = index
            self.update()
        self.setCursor(Qt.PointingHandCursor if key else Qt.ArrowCursor)

    def mousePressEvent(self, event):
        if not self.interactive:
            return
        if event.button() != Qt.LeftButton:
            return
        group, key, index = self._hit_test(event.pos())
        if key is None:
            return
        self.drag_group = group
        self.drag_key = key
        self.drag_index = index
        self.hover_group = group
        self.hover_key = key
        self.hover_index = index
        self.press_pos = QPointF(event.pos())
        self.drag_pos = QPointF(event.pos())
        rect = self._card_rect(group, index)
        self.drag_offset = QPointF(event.pos()) - rect.center()
        self.drag_moved = False
        self.update()

    def mouseReleaseEvent(self, event):
        if not self.interactive:
            return
        if event.button() != Qt.LeftButton or self.drag_key is None:
            return
        group = self.drag_group
        key = self.drag_key
        source_index = self.drag_index
        drop_group, _drop_key, drop_index = self._hit_test(event.pos())
        self.drag_group = None
        self.drag_key = None
        self.drag_index = None
        if self.drag_moved and drop_group == group and drop_index is not None and drop_index != source_index:
            slots = list(self._slots_for_group(group))
            if 0 <= int(source_index) < len(slots) and 0 <= int(drop_index) < len(slots):
                slots[source_index], slots[drop_index] = slots[drop_index], slots[source_index]
                self._set_slots_for_group(group, slots)
        elif not self.drag_moved:
            if group == SCENE3_GROUP_COLOR:
                self.colorClicked.emit(key)
            elif group == SCENE3_GROUP_WASTE:
                self.wasteClicked.emit(key)
        self.drag_moved = False
        self.hover_group, self.hover_key, self.hover_index = self._hit_test(event.pos())
        self.setCursor(Qt.PointingHandCursor if self.hover_key else Qt.ArrowCursor)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(30, 31, 49))
        left, right = self._areas()
        self._draw_group(painter, left, '色块位置', self.color_slots, SCENE2_COLOR_LABELS, SCENE2_COLOR_QCOLORS, SCENE3_GROUP_COLOR)
        self._draw_group(painter, right, '垃圾分类', self.waste_slots, WASTE_LABELS, WASTE_QCOLORS, SCENE3_GROUP_WASTE)
        if self.drag_key is not None and self.drag_moved:
            rect = self._card_rect(self.drag_group, self.drag_index)
            rect.moveCenter(self.drag_pos - self.drag_offset)
            labels = self._labels_for_group(self.drag_group)
            colors = self._colors_for_group(self.drag_group)
            self._draw_card(painter, rect, self.drag_key, labels, colors, active=True, dragging=True)

    def _draw_group(self, painter, area, title, slots, labels, colors, group):
        painter.setPen(QPen(QColor(74, 77, 94), 1))
        painter.setBrush(QBrush(QColor(45, 47, 63)))
        painter.drawRoundedRect(area.adjusted(-6, -6, 6, 6), 6, 6)
        font = painter.font()
        font.setPointSize(12)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(250, 143, 1))
        painter.drawText(area.adjusted(4, -4, -4, 0), Qt.AlignTop | Qt.AlignLeft, title)
        rects = self._card_rects(area, slots)
        for index, key in enumerate(slots):
            if key == self.drag_key and group == self.drag_group and self.drag_moved:
                continue
            active = self.hover_group == group and self.hover_key == key and self.hover_index == index
            self._draw_card(painter, rects[index], key, labels, colors, active=active)

    def _draw_card(self, painter, rect, key, labels, colors, active=False, dragging=False):
        shadow = rect.translated(0, 3)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 36 if active or dragging else 16))
        painter.drawRoundedRect(shadow, 9, 9)
        painter.setBrush(QBrush(colors.get(key, QColor(180, 180, 180))))
        painter.setPen(QPen(QColor(15, 23, 42), 2 if active or dragging else 1))
        painter.drawRoundedRect(rect, 7, 7)
        label = labels.get(key, key)
        font = painter.font()
        font.setPointSize(13 if len(label) <= 2 else 9)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(17, 24, 39))
        painter.drawText(rect, Qt.AlignCenter | Qt.TextWordWrap, label)


class Scene4BoardWidget(QWidget):
    frameMoved = pyqtSignal(str, float, float)
    frameClicked = pyqtSignal(str)
    destinationChanged = pyqtSignal(str, str)
    frameGridChanged = pyqtSignal(str, list)
    shelfGridChanged = pyqtSignal(str, str, list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.scene_mode = SCENE4_MODE_COLOR
        self.item_keys = list(SCENE4_COLOR_KEYS)
        self.item_labels = dict(SCENE2_COLOR_LABELS)
        self.item_colors = dict(SCENE2_COLOR_QCOLORS)
        self.board_length_m = SCENE4_SHELF_LENGTH_M
        self.board_width_m = SCENE4_SHELF_WIDTH_M
        self.absolute_positions = normalized_scene4_absolute_positions(None)
        self.targets = {key: scene4_default_position(key) for key in self.item_keys}
        self.destinations = dict(DEFAULT_SCENE4_PLACE['targets'])
        self.frame_slots = normalized_scene4_frame_slots(self.item_keys, self.item_keys)
        self.shelf_slots = {
            destination: normalized_scene4_shelf_slots(None, self.item_keys, destination)
            for destination in SCENE4_SHELF_LEVELS
        }
        self.hover_key = None
        self.active_key = None
        self.drag_key = None
        self.drag_origin_destination = None
        self.drag_origin_slot = None
        self.drag_destination = None
        self.drag_pos = QPointF(0.0, 0.0)
        self.drop_slot = None
        self.drag_offset = QPointF(0.0, 0.0)
        self.press_pos = QPointF(0.0, 0.0)
        self.drag_moved = False
        self.fixed_layout = False

    def set_scene_mode(self, mode, keys=None, labels=None, colors=None):
        mode = mode if mode in SCENE4_MODES else SCENE4_MODE_COLOR
        self.scene_mode = mode
        self.item_keys = list(keys or scene4_keys_for_mode(mode))
        self.item_labels = dict(labels or scene4_labels_for_mode(mode))
        self.item_colors = dict(colors or scene4_colors_for_mode(mode))
        for key in self.item_keys:
            self.targets.setdefault(key, scene4_default_position(key))
            self.destinations.setdefault(key, DEFAULT_SCENE4_PLACE['targets'].get(key, SCENE4_PLACE_FRAME))
        self.frame_slots = normalized_scene4_frame_slots(self.frame_slots, self.item_keys)
        for destination in SCENE4_SHELF_LEVELS:
            self.shelf_slots[destination] = normalized_scene4_shelf_slots(
                self.shelf_slots.get(destination),
                self.item_keys,
                destination,
            )
        if self.active_key not in self.item_keys:
            self.active_key = None
        if self.hover_key not in self.item_keys:
            self.hover_key = None
        self.update()

    def set_frame_slots(self, slots):
        self.frame_slots = normalized_scene4_frame_slots(slots, self.item_keys)
        self.update()

    def set_shelf_slots(self, slots_by_destination):
        slots_by_destination = slots_by_destination if isinstance(slots_by_destination, dict) else {}
        for destination in SCENE4_SHELF_LEVELS:
            self.shelf_slots[destination] = normalized_scene4_shelf_slots(
                slots_by_destination.get(destination),
                self.item_keys,
                destination,
            )
        self.update()

    def set_absolute_positions(self, absolute_positions):
        self.absolute_positions = normalized_scene4_absolute_positions(absolute_positions)
        self.update()

    def _layer_rects(self):
        margin = 12.0
        gap = 8.0
        total_w = max(1.0, float(self.width()) - margin * 2.0)
        total_h = max(1.0, float(self.height()) - margin * 2.0)
        shelf_h = max(50.0, min(68.0, total_h * 0.25))
        available_frame_h = total_h - shelf_h * 2.0 - gap * 2.0
        frame_h = max(54.0, available_frame_h)
        x = margin
        y = margin
        return {
            SCENE4_SHELF_LEVEL1: QRectF(x, y, total_w, shelf_h),
            SCENE4_SHELF_LEVEL2: QRectF(x, y + shelf_h + gap, total_w, shelf_h),
            SCENE4_PLACE_FRAME: QRectF(x, y + shelf_h * 2.0 + gap * 2.0, total_w, frame_h),
        }

    def _shelf_inner_rect(self, destination):
        rect = self._layer_rects()[destination]
        return rect.adjusted(12.0, 18.0, -12.0, -8.0)

    def _shelf_slot_rects(self, destination):
        inner = self._shelf_inner_rect(destination)
        slot_count = max(SCENE4_SHELF_SLOT_COUNT, len(self.shelf_slots.get(destination, [])))
        gap = 8.0 if slot_count > SCENE4_SHELF_SLOT_COUNT else 10.0
        slot_w = (inner.width() - gap * (slot_count - 1)) / max(1, slot_count)
        return [
            QRectF(inner.left() + index * (slot_w + gap), inner.top(), slot_w, inner.height())
            for index in range(slot_count)
        ]

    def _shelf_slot_index(self, pos, destination):
        point = QPointF(pos)
        slots = self._shelf_slot_rects(destination)
        for index, rect in enumerate(slots):
            if rect.contains(point):
                return index
        inner = self._shelf_inner_rect(destination)
        if not inner.contains(point):
            return None
        slot_count = max(1, len(slots))
        rel_x = (float(point.x()) - inner.left()) / max(inner.width(), 1e-6)
        return max(0, min(slot_count - 1, int(rel_x * slot_count)))

    def _frame_inner_rect(self):
        return self._layer_rects()[SCENE4_PLACE_FRAME].adjusted(12.0, 18.0, -12.0, -10.0)

    def _frame_cell_rects(self):
        inner = self._frame_inner_rect()
        slot_count = SCENE4_FRAME_SLOT_COUNT
        cols = 2
        rows = 2
        gap = 10.0
        cell_w = (inner.width() - gap * (cols - 1)) / max(1, cols)
        cell_h = (inner.height() - gap * (rows - 1)) / max(1, rows)
        rects = []
        for index in range(slot_count):
            row = index // cols
            col = index % cols
            rects.append(QRectF(
                inner.left() + col * (cell_w + gap),
                inner.top() + row * (cell_h + gap),
                cell_w,
                cell_h,
            ))
        return rects

    def _frame_cell_index(self, pos):
        point = QPointF(pos)
        cells = self._frame_cell_rects()
        for index, rect in enumerate(cells):
            if rect.contains(point):
                return index
        inner = self._frame_inner_rect()
        if not inner.contains(point):
            return None
        slot_count = max(1, len(cells))
        cols = 2
        rows = 2
        rel_x = (float(point.x()) - inner.left()) / max(inner.width(), 1e-6)
        rel_y = (float(point.y()) - inner.top()) / max(inner.height(), 1e-6)
        col = max(0, min(cols - 1, int(rel_x * cols)))
        row = max(0, min(rows - 1, int(rel_y * rows)))
        return max(0, min(slot_count - 1, row * cols + col))

    def _destination_at(self, pos):
        point = QPointF(pos)
        rects = self._layer_rects()
        for destination in (SCENE4_SHELF_LEVEL1, SCENE4_SHELF_LEVEL2, SCENE4_PLACE_FRAME):
            if rects[destination].contains(point):
                return destination
        return None

    def _destination_for_key(self, key):
        destination = self.destinations.get(key, DEFAULT_SCENE4_PLACE['targets'].get(key, SCENE4_PLACE_FRAME))
        if destination not in (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS):
            destination = SCENE4_PLACE_FRAME
        return destination

    def _card_size(self, area):
        return max(28.0, min(48.0, area.width() * 0.085, area.height() * 0.68))

    def _clamp_world(self, x, y):
        x = max(0.0, min(float(self.board_length_m), float(x)))
        half_w = float(self.board_width_m) * 0.5
        y = max(-half_w, min(half_w, float(y)))
        return x, y

    def _world_to_point(self, x, y, board):
        x, y = self._clamp_world(x, y)
        px = board.left() + (x / max(self.board_length_m, 1e-6)) * board.width()
        py = board.top() + (0.5 - y / max(self.board_width_m, 1e-6)) * board.height()
        return QPointF(px, py)

    def _point_to_world(self, point, board):
        nx = (float(point.x()) - board.left()) / max(board.width(), 1e-6)
        ny = (float(point.y()) - board.top()) / max(board.height(), 1e-6)
        x = nx * float(self.board_length_m)
        y = (0.5 - ny) * float(self.board_width_m)
        return self._clamp_world(x, y)

    def _slot_for_key(self, key):
        for index, slot_key in enumerate(self.frame_slots):
            if slot_key == key:
                return index
        return None

    def _shelf_slot_for_key(self, key, destination):
        if destination not in SCENE4_SHELF_LEVELS or self._destination_for_key(key) != destination:
            return None
        for index, slot_key in enumerate(self.shelf_slots.get(destination, [])):
            if slot_key == key:
                return index
        return None

    def _slot_for_key_in_destination(self, key, destination):
        if destination == SCENE4_PLACE_FRAME:
            return self._slot_for_key(key)
        if destination in SCENE4_SHELF_LEVELS:
            return self._shelf_slot_for_key(key, destination)
        return None

    def _fixed_position_for_key(self, key, destination):
        if destination == SCENE4_PLACE_FRAME:
            return scene4_fixed_position(
                key,
                destination,
                self._slot_for_key(key),
                absolute_positions=self.absolute_positions,
            )
        if destination in SCENE4_SHELF_LEVELS:
            return scene4_fixed_position(
                key,
                destination,
                shelf_slot_index=self._shelf_slot_for_key(key, destination),
                absolute_positions=self.absolute_positions,
            )
        return scene4_default_position(key)

    def _frame_rect(self, key):
        destination = self._destination_for_key(key)
        if destination == SCENE4_PLACE_FRAME:
            slot = self._slot_for_key(key)
            if slot is None:
                slot = min(len(self.item_keys) - 1, self.item_keys.index(key)) if key in self.item_keys else 0
            cells = self._frame_cell_rects()
            cell = cells[max(0, min(len(cells) - 1, slot))]
            size = max(16.0, min(40.0, cell.width() * 0.46, cell.height() * 0.82))
            center = cell.center()
        else:
            slot = self._shelf_slot_for_key(key, destination)
            if slot is None:
                pos = self.targets.get(key, scene4_default_position(key))
                slot = 1 if pos[1] < 0 else 0
            cells = self._shelf_slot_rects(destination)
            slot = max(0, min(len(cells) - 1, slot))
            cell = cells[slot]
            center = cell.center()
            size = max(16.0, min(40.0, cell.width() * 0.44, cell.height() * 0.86))
        return QRectF(center.x() - size * 0.5, center.y() - size * 0.5, size, size)

    def _hit_test(self, pos):
        p = QPointF(pos)
        for key in reversed(self.item_keys):
            if self._frame_rect(key).contains(p):
                return key
        return None

    def set_scene_targets(self, place_targets, length_m=SCENE4_SHELF_LENGTH_M, width_m=SCENE4_SHELF_WIDTH_M, destinations=None, shelf_slots=None, absolute_positions=None):
        self.board_length_m = float(length_m or SCENE4_SHELF_LENGTH_M)
        self.board_width_m = float(width_m or SCENE4_SHELF_WIDTH_M)
        self.set_absolute_positions(absolute_positions)
        for key in self.item_keys:
            default = scene4_default_position(key)
            pos = place_targets.get(key, default) if isinstance(place_targets, dict) else default
            self.targets[key] = normalized_position(pos, default)
        if isinstance(destinations, dict):
            for key in self.item_keys:
                destination = destinations.get(key, DEFAULT_SCENE4_PLACE['targets'].get(key, SCENE4_PLACE_FRAME))
                if destination in (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS):
                    self.destinations[key] = destination
        self.set_shelf_slots(shelf_slots)
        self.update()

    def set_active_key(self, key):
        self.active_key = key if key in self.item_keys else None
        self.update()

    def mouseMoveEvent(self, event):
        pos = QPointF(event.pos())
        if self.drag_key is not None:
            if (pos - self.press_pos).manhattanLength() > 3.0:
                self.drag_moved = True
            self.drag_pos = pos - self.drag_offset
            self.drag_destination = self._destination_at(pos) or self.drag_origin_destination
            if self.drag_destination == SCENE4_PLACE_FRAME:
                self.drop_slot = self._frame_cell_index(pos)
            elif self.drag_destination in SCENE4_SHELF_LEVELS:
                self.drop_slot = self._shelf_slot_index(pos, self.drag_destination)
            else:
                self.drop_slot = None
            self.update()
            return
        key = self._hit_test(event.pos())
        if key != self.hover_key:
            self.hover_key = key
            self.update()
        self.setCursor(Qt.PointingHandCursor if key else Qt.ArrowCursor)

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        key = self._hit_test(event.pos())
        if key is None:
            return
        self.active_key = key
        self.drag_key = key
        self.drag_origin_destination = self._destination_for_key(key)
        self.drag_origin_slot = self._slot_for_key_in_destination(key, self.drag_origin_destination)
        self.drag_destination = self.drag_origin_destination
        self.press_pos = QPointF(event.pos())
        rect = self._frame_rect(key)
        self.drag_offset = QPointF(event.pos()) - rect.center()
        self.drag_pos = rect.center()
        self.drop_slot = self.drag_origin_slot
        self.drag_moved = False
        self.update()

    def _place_key_in_frame_slot(self, key, drop_slot, origin_destination):
        if drop_slot is None:
            drop_slot = self.drag_origin_slot
        if drop_slot is None:
            drop_slot = self._slot_for_key(key)
        if drop_slot is None:
            drop_slot = 0
        slots = normalized_scene4_frame_slots(self.frame_slots, self.item_keys)
        drop_slot = max(0, min(len(slots) - 1, int(drop_slot)))
        old_slot = self._slot_for_key(key)
        old_slots = list(slots)
        occupant = slots[drop_slot] if drop_slot < len(slots) else ''
        if occupant == key or self._destination_for_key(occupant) != SCENE4_PLACE_FRAME:
            occupant = ''
        if old_slot is not None:
            slots[old_slot] = ''
        if occupant and old_slot is not None:
            slots[old_slot] = occupant
        elif occupant and origin_destination in SCENE4_SHELF_LEVELS:
            self.destinations[occupant] = origin_destination
            origin_slot = self.drag_origin_slot
            if origin_slot is not None:
                origin_slots = normalized_scene4_shelf_slots(
                    self.shelf_slots.get(origin_destination),
                    self.item_keys,
                    origin_destination,
                )
                origin_slot = max(0, min(len(origin_slots) - 1, int(origin_slot)))
                origin_slots[origin_slot] = occupant
                self.shelf_slots[origin_destination] = normalized_scene4_shelf_slots(
                    origin_slots,
                    self.item_keys,
                    origin_destination,
                )
        slots[drop_slot] = key
        self.frame_slots = normalized_scene4_frame_slots(slots, self.item_keys)
        changed_destinations = []
        if occupant and origin_destination in SCENE4_SHELF_LEVELS:
            changed_destinations.append((occupant, origin_destination))
        return old_slots != self.frame_slots, changed_destinations

    def _place_key_in_shelf_slot(self, key, drop_slot, destination, origin_destination):
        if drop_slot is None:
            drop_slot = self.drag_origin_slot if origin_destination == destination else self._shelf_slot_for_key(key, destination)
        if drop_slot is None:
            drop_slot = 0
        slots = normalized_scene4_shelf_slots(self.shelf_slots.get(destination), self.item_keys, destination)
        drop_slot = max(0, min(len(slots) - 1, int(drop_slot)))
        old_slot = self._shelf_slot_for_key(key, destination)
        old_slots = list(slots)
        occupant = slots[drop_slot] if drop_slot < len(slots) else ''
        if occupant == key or self._destination_for_key(occupant) != destination:
            occupant = ''
        if old_slot is not None:
            slots[old_slot] = ''

        changed_destinations = []
        if occupant:
            if origin_destination == SCENE4_PLACE_FRAME:
                origin_slot = self.drag_origin_slot
                if origin_slot is not None:
                    frame_slots = normalized_scene4_frame_slots(self.frame_slots, self.item_keys)
                    origin_slot = max(0, min(len(frame_slots) - 1, int(origin_slot)))
                    frame_slots[origin_slot] = occupant
                    self.frame_slots = normalized_scene4_frame_slots(frame_slots, self.item_keys)
            elif origin_destination in SCENE4_SHELF_LEVELS:
                origin_slot = self.drag_origin_slot
                if origin_slot is not None:
                    origin_slots = normalized_scene4_shelf_slots(
                        self.shelf_slots.get(origin_destination),
                        self.item_keys,
                        origin_destination,
                    )
                    origin_slot = max(0, min(len(origin_slots) - 1, int(origin_slot)))
                    origin_slots[origin_slot] = occupant
                    self.shelf_slots[origin_destination] = normalized_scene4_shelf_slots(
                        origin_slots,
                        self.item_keys,
                        origin_destination,
                    )
            self.destinations[occupant] = origin_destination
            changed_destinations.append((occupant, origin_destination))

        slots[drop_slot] = key
        self.shelf_slots[destination] = normalized_scene4_shelf_slots(slots, self.item_keys, destination)
        return old_slots != self.shelf_slots[destination], changed_destinations

    def mouseReleaseEvent(self, event):
        if self.drag_key is None:
            return
        key = self.drag_key
        release_pos = QPointF(event.pos())
        origin_destination = self.drag_origin_destination or self._destination_for_key(key)
        origin_slot = self.drag_origin_slot
        drop_destination = self._destination_at(release_pos) or origin_destination
        drop_slot = self.drop_slot
        if drop_destination not in (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS):
            drop_destination = origin_destination
        self.drag_key = None
        self.drag_origin_destination = None
        self.drag_destination = None
        self.drop_slot = None
        if not self.drag_moved:
            self.drag_origin_slot = None
            self.frameClicked.emit(key)
            self.update()
            return
        frame_changed = False
        shelf_changed = False
        changed_destinations = []
        self.drag_origin_slot = origin_slot
        if drop_destination == SCENE4_PLACE_FRAME:
            frame_changed, changed_destinations = self._place_key_in_frame_slot(key, drop_slot, origin_destination)
            self.destinations[key] = SCENE4_PLACE_FRAME
            self.targets[key] = self._fixed_position_for_key(key, SCENE4_PLACE_FRAME)
        else:
            shelf_changed, changed_destinations = self._place_key_in_shelf_slot(
                key,
                drop_slot,
                drop_destination,
                origin_destination,
            )
            self.destinations[key] = drop_destination
            self.targets[key] = self._fixed_position_for_key(key, drop_destination)
        for moved_key, moved_destination in changed_destinations:
            self.targets[moved_key] = self._fixed_position_for_key(moved_key, moved_destination)
        if drop_destination != origin_destination:
            self.destinationChanged.emit(key, drop_destination)
        for moved_key, moved_destination in changed_destinations:
            self.destinationChanged.emit(moved_key, moved_destination)
        frame_snapshot = list(self.frame_slots)
        shelf_snapshot = {
            destination: list(self.shelf_slots.get(destination, []))
            for destination in SCENE4_SHELF_LEVELS
        }
        if frame_changed or drop_destination == SCENE4_PLACE_FRAME or origin_destination == SCENE4_PLACE_FRAME:
            self.frameGridChanged.emit(self.scene_mode, frame_snapshot)
        if shelf_changed or drop_destination in SCENE4_SHELF_LEVELS or origin_destination in SCENE4_SHELF_LEVELS:
            for destination, slots in shelf_snapshot.items():
                self.shelfGridChanged.emit(self.scene_mode, destination, slots)
        self.drag_origin_slot = None
        self.update()

    def _draw_shelf_layer(self, painter, destination, rect):
        layer_titles = {
            SCENE4_SHELF_LEVEL1: '上层',
            SCENE4_SHELF_LEVEL2: '下层',
        }
        fills = {
            SCENE4_SHELF_LEVEL1: QColor(45, 47, 63),
            SCENE4_SHELF_LEVEL2: QColor(45, 47, 63),
        }
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 18))
        painter.drawRoundedRect(rect.translated(0, 3), 6, 6)
        painter.setBrush(QBrush(fills.get(destination, QColor(45, 47, 63))))
        painter.setPen(QPen(QColor(74, 77, 94), 1))
        painter.drawRoundedRect(rect, 6, 6)

        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(250, 143, 1))
        painter.drawText(rect.adjusted(8, 3, -8, -3), Qt.AlignTop | Qt.AlignLeft, layer_titles.get(destination, destination))
        for index, slot_rect in enumerate(self._shelf_slot_rects(destination)):
            active = (
                self.drag_key is not None
                and self.drag_destination == destination
                and self.drop_slot == index
            )
            painter.setBrush(QBrush(QColor(245, 245, 245, 34) if active else QColor(37, 39, 53, 110)))
            painter.setPen(QPen(QColor(245, 245, 245, 210 if active else 120), 2 if active else 1))
            painter.drawRoundedRect(slot_rect, 4, 4)
            font = painter.font()
            font.setPointSize(8)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QColor(245, 245, 245, 180))
            painter.drawText(slot_rect.adjusted(6, 2, -6, -2), Qt.AlignTop | Qt.AlignLeft, f'{index + 1}')

    def _draw_frame_layer(self, painter, rect):
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 18))
        painter.drawRoundedRect(rect.translated(0, 3), 6, 6)
        painter.setBrush(QBrush(QColor(45, 47, 63)))
        painter.setPen(QPen(QColor(74, 77, 94), 1))
        painter.drawRoundedRect(rect, 6, 6)
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(250, 143, 1))
        painter.drawText(rect.adjusted(8, 3, -8, -3), Qt.AlignTop | Qt.AlignLeft, '田字')
        cells = self._frame_cell_rects()
        for index, cell in enumerate(cells):
            active = (
                self.drag_key is not None
                and self.drag_destination == SCENE4_PLACE_FRAME
                and self.drop_slot == index
            )
            painter.setBrush(QBrush(QColor(245, 245, 245, 34) if active else QColor(37, 39, 53, 120)))
            painter.setPen(QPen(QColor(245, 245, 245, 210 if active else 120), 2 if active else 1))
            painter.drawRoundedRect(cell, 4, 4)
            font = painter.font()
            font.setPointSize(8)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QColor(176, 190, 197))
            painter.drawText(cell.adjusted(6, 4, -6, -4), Qt.AlignTop | Qt.AlignLeft, f'{index + 1}')

    def _draw_card(self, painter, rect, key, active=False, dragging=False):
        fill = self.item_colors.get(key, QColor(180, 180, 180))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 42 if active or dragging else 20))
        painter.drawRoundedRect(rect.translated(0, 1), 7, 7)
        painter.setBrush(QBrush(fill))
        painter.setPen(QPen(QColor(15, 23, 42), 2 if active or dragging else 1))
        painter.drawRoundedRect(rect, 7, 7)
        label = SCENE4_CARD_LABELS.get(key, self.item_labels.get(key, key))
        font = painter.font()
        font.setPointSize(max(8, min(14, int(rect.height() * 0.62))))
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(17, 24, 39))
        painter.drawText(rect.adjusted(2, 0, -2, 0), Qt.AlignCenter | Qt.TextWordWrap, label)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(30, 31, 49))
        rects = self._layer_rects()

        self._draw_shelf_layer(painter, SCENE4_SHELF_LEVEL1, rects[SCENE4_SHELF_LEVEL1])
        self._draw_shelf_layer(painter, SCENE4_SHELF_LEVEL2, rects[SCENE4_SHELF_LEVEL2])
        self._draw_frame_layer(painter, rects[SCENE4_PLACE_FRAME])

        for key in self.item_keys:
            if key == self.drag_key and self.drag_moved:
                continue
            rect = self._frame_rect(key)
            active = key == self.active_key or key == self.hover_key
            self._draw_card(painter, rect, key, active=active)
        if self.drag_key is not None and self.drag_moved:
            rect = self._frame_rect(self.drag_key)
            rect.moveCenter(self.drag_pos)
            self._draw_card(painter, rect, self.drag_key, active=True, dragging=True)


class MainWindow(QMainWindow):
    scene1_status_signal = pyqtSignal(str)
    scene1_processed_image_signal = pyqtSignal(object)
    scene1_rgb_image_signal = pyqtSignal(object)
    scene2_status_signal = pyqtSignal(str)
    scene2_result_image_signal = pyqtSignal(object)
    scene2_waste_image_signal = pyqtSignal(object)
    scene2_rgb_image_signal = pyqtSignal(object)
    scene3_status_signal = pyqtSignal(str)
    scene3_color_image_signal = pyqtSignal(object)
    scene3_waste_image_signal = pyqtSignal(object)
    scene3_rgb_image_signal = pyqtSignal(object)
    scene4_status_signal = pyqtSignal(str)
    scene4_result_image_signal = pyqtSignal(object)
    scene4_waste_image_signal = pyqtSignal(object)
    scene4_rgb_image_signal = pyqtSignal(object)
    scene5_status_signal = pyqtSignal(str)
    scene5_arm_a_image_signal = pyqtSignal(object)
    scene5_arm_b_image_signal = pyqtSignal(object)
    scene5_waste_image_signal = pyqtSignal(object)
    scene5_rgb_image_signal = pyqtSignal(object)
    calib_result_image_signal = pyqtSignal(object)
    calib_depth_image_signal = pyqtSignal(object)
    calib_rgb_image_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.node = None
        threading.Thread(target=self.ros_node, daemon=True).start()
        wait_start = time.time()
        while not init_finish and time.time() - wait_start <= 5.0:
            time.sleep(0.1)

        self.scene_cfg = self.load_scene_cfg()
        self.params = self.load_calibration_params()
        self.button_object = {}
        self.home = {}
        self.scene1_preview_pixmaps = {'processed': None, 'rgb': None}
        self.scene1_processed_hold_until = 0.0
        self.scene2_preview_pixmaps = {'result': None, 'waste': None, 'rgb': None}
        self.scene2_mode = 'color'
        self.scene2_active_result = 'result'
        self.scene3_preview_pixmaps = {'color': None, 'waste': None, 'rgb': None}
        self.scene3_active_result = 'color'
        self.scene4_preview_pixmaps = {'result': None, 'waste': None, 'rgb': None}
        self.scene4_mode = SCENE4_MODE_COLOR
        self.scene4_active_result = 'result'
        self.scene5_preview_pixmaps = {'arm_a': None, 'arm_b': None}
        self.calib_preview_pixmaps = {'result': None, 'depth': None, 'rgb': None}
        self._preview_last_update = {}
        self.scene5_active_result = 'arm_b'
        self._updating_scene2_coords = False
        self._updating_scene3_coords = False
        self._updating_scene4_coords = False
        self._updating_scene4_pick = False
        self._updating_scene4_place = False
        self._updating_scene4_absolute = False
        self._updating_scene5_grid = False

        self.setup_ui()
        self.scene1_status_signal.connect(self._set_scene1_action_status)
        self.scene2_status_signal.connect(self._set_scene2_action_status)
        self.scene3_status_signal.connect(self._set_scene3_action_status)
        self.scene4_status_signal.connect(self._set_scene4_action_status)
        self.scene5_status_signal.connect(self._set_scene5_action_status)
        self.scene1_processed_image_signal.connect(lambda msg: self._update_scene1_preview('processed', msg))
        self.scene1_rgb_image_signal.connect(lambda msg: self._update_scene1_preview('rgb', msg))
        self.scene2_result_image_signal.connect(lambda msg: self._update_scene2_preview('result', msg))
        self.scene2_waste_image_signal.connect(lambda msg: self._update_scene2_preview('waste', msg))
        self.scene2_rgb_image_signal.connect(lambda msg: self._update_scene2_preview('rgb', msg))
        self.scene3_color_image_signal.connect(lambda msg: self._update_scene3_preview('color', msg))
        self.scene3_waste_image_signal.connect(lambda msg: self._update_scene3_preview('waste', msg))
        self.scene3_rgb_image_signal.connect(lambda msg: self._update_scene3_preview('rgb', msg))
        self.scene4_result_image_signal.connect(lambda msg: self._update_scene4_preview('result', msg))
        self.scene4_waste_image_signal.connect(lambda msg: self._update_scene4_preview('waste', msg))
        self.scene4_rgb_image_signal.connect(lambda msg: self._update_scene4_preview('rgb', msg))
        self.scene5_arm_a_image_signal.connect(lambda msg: self._update_scene5_preview('arm_a', msg))
        self.scene5_arm_b_image_signal.connect(lambda msg: self._update_scene5_preview('arm_b', msg))
        self.scene5_waste_image_signal.connect(lambda msg: self._update_scene5_preview('arm_b', msg))
        self.scene5_rgb_image_signal.connect(lambda msg: self._update_scene5_preview('arm_a', msg))
        self.calib_result_image_signal.connect(lambda msg: self._update_calib_preview('result', msg))
        self.calib_depth_image_signal.connect(lambda msg: self._update_calib_preview('depth', msg))
        self.calib_rgb_image_signal.connect(lambda msg: self._update_calib_preview('rgb', msg))
        if self.node is not None:
            self.node.set_scene1_image_callbacks(
                processed_callback=self.scene1_processed_image_signal.emit,
                rgb_callback=self.scene1_rgb_image_signal.emit,
            )
            self.node.set_scene2_image_callbacks(
                result_callback=self.scene2_result_image_signal.emit,
                waste_callback=self.scene2_waste_image_signal.emit,
                rgb_callback=self.scene2_rgb_image_signal.emit,
            )
            self.node.set_scene3_image_callbacks(
                color_callback=self.scene3_color_image_signal.emit,
                waste_callback=self.scene3_waste_image_signal.emit,
                rgb_callback=self.scene3_rgb_image_signal.emit,
            )
            self.node.set_scene4_image_callbacks(
                result_callback=self.scene4_result_image_signal.emit,
                waste_callback=self.scene4_waste_image_signal.emit,
                rgb_callback=self.scene4_rgb_image_signal.emit,
            )
            self.node.set_scene5_image_callbacks(
                arm_a_callback=self.scene5_arm_a_image_signal.emit,
                arm_b_callback=self.scene5_arm_b_image_signal.emit,
            )
            self.node.set_calib_image_callbacks(
                result_callback=self.calib_result_image_signal.emit,
                depth_callback=self.calib_depth_image_signal.emit,
                rgb_callback=self.calib_rgb_image_signal.emit,
            )
        self.load_scene_combo()
        self.index_changed(0)
        _start_calibration_subprocess()

    def closeEvent(self, event):
        _stop_calibration_subprocess()
        event.accept()

    def ros_node(self):
        self.node = ArmControlNode('calibration_main_scene')
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self.node)
        executor.spin()
        self.node.destroy_node()

    def load_calibration_params(self):
        with open(POSITIONS_YAML_PATH, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def load_scene_cfg(self):
        if not os.path.exists(SCENE_YAML_PATH):
            cfg = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG))
            self.normalize_scene_cfg(cfg)
            self.apply_play_configs(cfg)
            self.normalize_scene_cfg(cfg)
            if ENV_CURRENT_SCENE in cfg.get('scenes', {}):
                os.makedirs(os.path.dirname(SCENE_YAML_PATH), exist_ok=True)
                with open(SCENE_YAML_PATH, 'w', encoding='utf-8') as f:
                    yaml.safe_dump(self.scene_cfg_for_save(cfg), f, sort_keys=False, allow_unicode=True)
                self.save_play_configs(cfg)
            return cfg
        with open(SCENE_YAML_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        if 'scenes' not in cfg or not cfg['scenes']:
            cfg = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG))
        self.normalize_scene_cfg(cfg)
        self.apply_play_configs(cfg)
        self.normalize_scene_cfg(cfg)
        env_scene_selected = ENV_CURRENT_SCENE in cfg['scenes']
        if env_scene_selected:
            cfg['current_scene'] = ENV_CURRENT_SCENE
        if cfg.get('current_scene') not in cfg['scenes']:
            cfg['current_scene'] = DEFAULT_CURRENT_SCENE if DEFAULT_CURRENT_SCENE in cfg['scenes'] else next(iter(cfg['scenes'].keys()))
        if env_scene_selected:
            os.makedirs(os.path.dirname(SCENE_YAML_PATH), exist_ok=True)
            with open(SCENE_YAML_PATH, 'w', encoding='utf-8') as f:
                yaml.safe_dump(self.scene_cfg_for_save(cfg), f, sort_keys=False, allow_unicode=True)
            self.save_play_configs(cfg)
        return cfg

    def save_scene_cfg(self):
        self.normalize_scene_cfg(self.scene_cfg)
        self.save_play_configs(self.scene_cfg)
        os.makedirs(os.path.dirname(SCENE_YAML_PATH), exist_ok=True)
        with open(SCENE_YAML_PATH, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.scene_cfg_for_save(self.scene_cfg), f, sort_keys=False, allow_unicode=True)
        self.sync_global_place_offset_config()
        self.sync_scene4_current_scene_config()

    def sync_global_place_offset_config(self):
        offset = normalize_global_place_offset(self.scene_cfg.get('global_place_offset'))
        self.scene_cfg['global_place_offset'] = offset
        primary_path = os.path.abspath(SCENE_YAML_PATH)
        for path in (APP_SCENE_YAML_PATH, STEPPER_SCENE_YAML_PATH):
            if os.path.abspath(path) == primary_path:
                continue
            cfg = read_yaml_dict(path)
            if not cfg:
                cfg = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG))
            self.normalize_scene_cfg(cfg)
            cfg['global_place_offset'] = dict(offset)
            write_yaml_dict(path, self.scene_cfg_for_save(cfg))

    def sync_scene4_current_scene_config(self):
        if self.scene_cfg.get('current_scene') != SCENE4_ID:
            return
        scenes = self.scene_cfg.get('scenes', {})
        scene4 = scenes.get(SCENE4_ID) if isinstance(scenes, dict) else None
        if not isinstance(scene4, dict):
            return
        primary_path = os.path.abspath(SCENE_YAML_PATH)
        for path in (APP_SCENE_YAML_PATH, STEPPER_SCENE_YAML_PATH):
            if os.path.abspath(path) == primary_path:
                continue
            cfg = read_yaml_dict(path)
            if not cfg:
                cfg = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG))
            self.normalize_scene_cfg(cfg)
            self.apply_play_configs(cfg)
            cfg.setdefault('scenes', {})[SCENE4_ID] = copy.deepcopy(scene4)
            cfg['current_scene'] = SCENE4_ID
            write_yaml_dict(path, self.scene_cfg_for_save(cfg))

    def apply_play_configs(self, cfg):
        scenes = cfg.get('scenes', {}) if isinstance(cfg, dict) else {}
        if not isinstance(scenes, dict):
            return
        for scene_id, scene in scenes.items():
            apply_scene_play_config(scene_id, scene)

    def save_play_configs(self, cfg):
        scenes = cfg.get('scenes', {}) if isinstance(cfg, dict) else {}
        if not isinstance(scenes, dict):
            return
        for scene_id, scene in scenes.items():
            path = scene_play_config_path(scene_id)
            if not path:
                continue
            write_yaml_dict(path, extract_scene_play_config(scene_id, scene))

    def scene_cfg_for_save(self, cfg):
        clean = copy.deepcopy(cfg) if isinstance(cfg, dict) else {}
        scenes = clean.get('scenes', {})
        if not isinstance(scenes, dict):
            return clean
        for scene_id, scene in list(scenes.items()):
            scenes[scene_id] = strip_scene_play_config(scene_id, scene)
        return clean

    def normalize_scene_cfg(self, cfg):
        if not isinstance(cfg, dict):
            cfg = {}
        raw_place_offset = cfg.get('global_place_offset')
        if raw_place_offset is None:
            raw_place_offset = cfg.get('place_offset')
        cfg['global_place_offset'] = normalize_global_place_offset(raw_place_offset)
        cfg.pop('place_offset', None)
        scenes = cfg.get('scenes', {})
        if not isinstance(scenes, dict):
            scenes = {}
        if SCENE5_ID in scenes and not isinstance(scenes.get(SCENE5_ID), dict):
            legacy_scene5 = {}
            for key in (
                'name',
                'mode',
                'length_m',
                'width_m',
                'use_calibration_scene',
                'calibration_tag',
                'home_pose',
                'calibration_pose',
                'place_policy',
                'place_targets',
                'scene5_grid',
            ):
                if key in scenes:
                    legacy_scene5[key] = scenes.pop(key)
            scenes[SCENE5_ID] = legacy_scene5
        if SCENE0_ID not in scenes:
            scenes[SCENE0_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE0_ID]))
        if DEFAULT_SCENE_ID not in scenes:
            scenes[DEFAULT_SCENE_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]))
        if SCENE2_ID not in scenes:
            scenes[SCENE2_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE2_ID]))
        if SCENE3_ID not in scenes:
            scenes[SCENE3_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE3_ID]))
        if SCENE4_ID not in scenes:
            scenes[SCENE4_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE4_ID]))
        if SCENE5_ID not in scenes:
            scenes[SCENE5_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE5_ID]))
        cfg['scenes'] = scenes
        for scene_id, scene in scenes.items():
            if not isinstance(scene, dict):
                scenes[scene_id] = {}
                scene = scenes[scene_id]
            if scene_id in BUILTIN_SCENE_NAMES:
                scene['name'] = BUILTIN_SCENE_NAMES[scene_id]
            else:
                scene.setdefault('name', scene_id.replace('_', ' ').title())
            if scene_id == SCENE0_ID:
                kin = scene.get('kinematics')
                if not isinstance(kin, dict):
                    kin = {}
                    scene['kinematics'] = kin
                params = kin.get('params')
                if not isinstance(params, list) or len(params) != 9:
                    kin['params'] = list(DEFAULT_BODY_KINEMATICS_PARAMS)
                else:
                    try:
                        kin['params'] = [float(v) for v in params[:9]]
                    except Exception:
                        kin['params'] = list(DEFAULT_BODY_KINEMATICS_PARAMS)
            if scene_id == SCENE4_ID:
                default_scene4 = DEFAULT_SCENE_CONFIG['scenes'][SCENE4_ID]
                scene['length_m'] = float(default_scene4['length_m'])
                scene['width_m'] = float(default_scene4['width_m'])
                scene['use_calibration_scene'] = SCENE4_ID
                scene['calibration_pose'] = normalized_calibration_pose(
                    scene.get('calibration_pose'),
                    DEFAULT_SCENE4_CALIBRATION_POSE,
                )
                scene['scene4_pick'] = normalized_scene4_pick(scene.get('scene4_pick'))
                scene['scene4_place'] = normalized_scene4_place(scene.get('scene4_place'))
                scene['scene4_shelf'] = normalized_scene4_shelf(scene.get('scene4_shelf'))
                if not isinstance(scene.get('calibration_tag'), dict):
                    scene['calibration_tag'] = yaml.safe_load(yaml.safe_dump(default_scene4['calibration_tag']))
            if scene_id == SCENE5_ID:
                default_scene5 = DEFAULT_SCENE_CONFIG['scenes'][SCENE5_ID]
                scene['mode'] = 'dual_arm_single_conveyor'
                scene['length_m'] = float(scene.get('length_m', default_scene5['length_m']))
                scene['width_m'] = float(scene.get('width_m', default_scene5['width_m']))
                scene['use_calibration_scene'] = SCENE4_ID
                scene.pop('rail', None)
                tag = scene.get('calibration_tag')
                if not isinstance(tag, dict):
                    tag = {}
                tag = _merge_missing_dict(tag, yaml.safe_load(yaml.safe_dump(default_scene5['calibration_tag'])))
                tag['center_in_map_m'] = yaml.safe_load(yaml.safe_dump(default_scene5['calibration_tag']['center_in_map_m']))
                tag['yaw_deg'] = float(tag.get('yaw_deg', default_scene5['calibration_tag'].get('yaw_deg', 0.0)))
                scene['calibration_tag'] = tag
                scene['calibration_pose'] = normalized_calibration_pose(
                    scene.get('calibration_pose'),
                    DEFAULT_SCENE5_CALIBRATION_POSE,
                )
                grid = scene.get('scene5_grid')
                if not isinstance(grid, dict):
                    grid = {}
                    scene['scene5_grid'] = grid
                color_slots = []
                for key in grid.get('color_slots', list(SCENE5_COLOR_KEYS)):
                    if key in SCENE5_COLOR_KEYS and key not in color_slots:
                        color_slots.append(key)
                for key in SCENE5_COLOR_KEYS:
                    if key not in color_slots:
                        color_slots.append(key)
                grid['color_slots'] = color_slots[:len(SCENE5_COLOR_KEYS)]
                waste_slots = []
                for key in grid.get('waste_slots', list(SCENE5_WASTE_KEYS)):
                    if key in WASTE_KEYS and key not in waste_slots:
                        waste_slots.append(key)
                for key in SCENE5_WASTE_KEYS:
                    if key not in waste_slots:
                        waste_slots.append(key)
                grid['waste_slots'] = waste_slots[:len(SCENE5_WASTE_KEYS)]
                dual_arm = scene.get('scene5_dual_arm')
                if not isinstance(dual_arm, dict):
                    dual_arm = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM))
                else:
                    dual_arm = _merge_missing_dict(
                        dual_arm,
                        yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM)),
                    )
                place_targets = dual_arm.get('arm_b_place_targets')
                if not isinstance(place_targets, dict):
                    place_targets = {}
                dual_arm['arm_b_place_targets'] = _merge_missing_dict(
                    place_targets,
                    yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'])),
                )
                scene['scene5_dual_arm'] = dual_arm
            if scene_id in (SCENE2_ID, SCENE3_ID):
                base_scene = scenes.get(DEFAULT_SCENE_ID, DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID])
                scene['use_calibration_scene'] = DEFAULT_SCENE_ID
                if scene_id == SCENE2_ID:
                    scene['length_m'] = float(base_scene.get('length_m', DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]['length_m']))
                    scene['width_m'] = float(base_scene.get('width_m', DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]['width_m']))
                elif scene_id == SCENE3_ID:
                    default_scene3 = DEFAULT_SCENE_CONFIG['scenes'][SCENE3_ID]
                    scene['length_m'] = float(default_scene3['length_m'])
                    scene['width_m'] = float(default_scene3['width_m'])
                    tag = scene.get('calibration_tag')
                    if not isinstance(tag, dict):
                        tag = {}
                    tag = _merge_missing_dict(tag, yaml.safe_load(yaml.safe_dump(default_scene3['calibration_tag'])))
                    tag['center_in_map_m'] = yaml.safe_load(
                        yaml.safe_dump(default_scene3['calibration_tag']['center_in_map_m'])
                    )
                    scene['calibration_tag'] = tag
                else:
                    scene['calibration_tag'] = yaml.safe_load(yaml.safe_dump(base_scene.get(
                        'calibration_tag',
                        DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]['calibration_tag'],
                    )))
            default_home = DEFAULT_SCENE_HOME_POSES.get(
                scene_id,
                DEFAULT_SCENE_HOME_POSES[DEFAULT_SCENE_ID],
            )
            scene['home_pose'] = normalized_home_pose(scene.get('home_pose'), default_home)
            policy = scene.get('place_policy')
            if not isinstance(policy, dict):
                policy = {}
                scene['place_policy'] = policy
            policy.setdefault('only_left_y_positive', bool(scene_id == DEFAULT_SCENE_ID))
            policy.setdefault('min_place_z', DEFAULT_PLACE_POLICY['min_place_z'])

            targets = scene.get('place_targets')
            if not isinstance(targets, dict):
                targets = {}
                scene['place_targets'] = targets
            for key, value in DEFAULT_SCENE_PLACE_TARGETS.items():
                if key not in targets:
                    targets[key] = list(value)
                elif not isinstance(targets[key], list) or len(targets[key]) != 3:
                    targets[key] = list(value)
            if scene_id == SCENE2_ID:
                grid = scene.get('color_grid')
                if not isinstance(grid, dict):
                    grid = {}
                    scene['color_grid'] = grid
                slots = grid.get('slots', list(SCENE2_COLOR_KEYS))
                clean = []
                for key in slots:
                    if key in SCENE2_COLOR_KEYS and key not in clean:
                        clean.append(key)
                for key in SCENE2_COLOR_KEYS:
                    if key not in clean:
                        clean.append(key)
                grid['slots'] = clean[:4]
                slot_targets = grid.get('slot_targets')
                if not isinstance(slot_targets, list) or len(slot_targets) != 4:
                    slot_targets = [list(DEFAULT_SCENE2_COLOR_TARGETS[key]) for key in SCENE2_SLOT_TARGET_KEYS]
                fixed_targets = []
                for idx, default_key in enumerate(SCENE2_SLOT_TARGET_KEYS):
                    raw = slot_targets[idx] if idx < len(slot_targets) else DEFAULT_SCENE2_COLOR_TARGETS[default_key]
                    if not isinstance(raw, list) or len(raw) != 3:
                        raw = DEFAULT_SCENE2_COLOR_TARGETS[default_key]
                    fixed_targets.append([float(raw[0]), float(raw[1]), float(raw[2])])
                grid['slot_targets'] = fixed_targets
                for idx, color_key in enumerate(grid['slots']):
                    targets[color_key] = list(fixed_targets[idx])
            if scene_id == SCENE3_ID:
                grid = normalized_scene3_grid(scene.get('scene3_grid'))
                scene['scene3_grid'] = grid
                for group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE):
                    slots = grid[SCENE3_GRID_SLOT_FIELDS[group]]
                    slot_targets = grid[SCENE3_GRID_TARGET_FIELDS[group]]
                    for idx, key in enumerate(slots):
                        targets[key] = list(slot_targets[idx])
                place_pitch = scene.get('place_pitch')
                if not isinstance(place_pitch, dict):
                    place_pitch = {}
                    scene['place_pitch'] = place_pitch
                for key, pitch in SCENE3_PLACE_PITCH.items():
                    place_pitch[key] = float(place_pitch.get(key, pitch))
            if scene_id == SCENE5_ID:
                policy['only_left_y_positive'] = False
                targets.setdefault('yellow', list(DEFAULT_SCENE2_COLOR_TARGETS['yellow']))
                for key in SCENE2_COLOR_KEYS:
                    targets.setdefault(key, list(DEFAULT_SCENE_PLACE_TARGETS.get(key, DEFAULT_SCENE2_COLOR_TARGETS.get(key, [0.0, 0.0, 0.015]))))
                for key in WASTE_KEYS:
                    targets.setdefault(key, list(DEFAULT_SCENE_PLACE_TARGETS[key]))
            if scene_id == SCENE4_ID:
                policy['only_left_y_positive'] = False
                for key in WASTE_KEYS:
                    targets.pop(key, None)
                scene['scene4_place'] = normalized_scene4_place(scene.get('scene4_place'))
                scene['scene4_shelf'] = normalized_scene4_shelf(scene.get('scene4_shelf'))
                scene['scene4_absolute_positions'] = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
                grid = scene.get('scene4_grid')
                if not isinstance(grid, dict):
                    grid = {}
                    scene['scene4_grid'] = grid
                for field in ('waste_slots', 'waste_upper_slots', 'waste_lower_slots'):
                    grid.pop(field, None)
                for mode in SCENE4_CONFIG_MODES:
                    slot_field = SCENE4_GRID_SLOT_FIELDS[mode]
                    keys = scene4_keys_for_mode(mode)
                    grid[slot_field] = normalized_scene4_frame_slots(grid.get(slot_field), keys)
                    for destination in SCENE4_SHELF_LEVELS:
                        shelf_slot_field = SCENE4_SHELF_SLOT_FIELDS[mode][destination]
                        grid[shelf_slot_field] = normalized_scene4_shelf_slots(
                            grid.get(shelf_slot_field),
                            keys,
                            destination,
                        )
                    for key in keys:
                        destination = scene['scene4_place']['targets'].get(
                            key,
                            scene['scene4_place'].get('default_destination', SCENE4_PLACE_FRAME),
                        )
                        if destination == SCENE4_PLACE_FRAME:
                            try:
                                slot_index = grid[slot_field].index(key)
                            except ValueError:
                                if mode == SCENE4_MODE_ALL and key in WASTE_KEYS:
                                    group_slots = grid.get(SCENE4_GRID_SLOT_FIELDS[SCENE4_MODE_WASTE], [])
                                    try:
                                        slot_index = group_slots.index(key)
                                    except ValueError:
                                        slot_index = list(WASTE_KEYS).index(key)
                                elif mode == SCENE4_MODE_ALL and key in SCENE4_COLOR_KEYS:
                                    group_slots = grid.get(SCENE4_GRID_SLOT_FIELDS[SCENE4_MODE_COLOR], [])
                                    try:
                                        slot_index = group_slots.index(key)
                                    except ValueError:
                                        slot_index = list(SCENE4_COLOR_KEYS).index(key)
                                else:
                                    slot_index = keys.index(key)
                            targets[key] = scene4_fixed_position(
                                key,
                                destination,
                                slot_index,
                                absolute_positions=scene['scene4_absolute_positions'],
                            )
                        elif destination in SCENE4_SHELF_LEVELS:
                            shelf_slot_field = SCENE4_SHELF_SLOT_FIELDS[mode][destination]
                            try:
                                shelf_slot_index = grid[shelf_slot_field].index(key)
                            except ValueError:
                                shelf_slot_index = None
                            targets[key] = scene4_fixed_position(
                                key,
                                destination,
                                shelf_slot_index=shelf_slot_index,
                                absolute_positions=scene['scene4_absolute_positions'],
                            )

                rail = scene.get('rail')
                if not isinstance(rail, dict):
                    rail = {}
                    scene['rail'] = rail
                rail['enabled'] = bool(rail.get('enabled', DEFAULT_SCENE4_RAIL['enabled']))
                for key in ('total_steps', 'subdivision', 'calibration_abs_position', 'place_abs_position'):
                    try:
                        rail[key] = int(rail.get(key, DEFAULT_SCENE4_RAIL[key]))
                    except Exception:
                        rail[key] = int(DEFAULT_SCENE4_RAIL[key])
                for key in ('reset_wait_sec', 'speed_steps_per_sec'):
                    try:
                        rail[key] = float(rail.get(key, DEFAULT_SCENE4_RAIL[key]))
                    except Exception:
                        rail[key] = float(DEFAULT_SCENE4_RAIL[key])

                kin = scene.get('kinematics')
                if not isinstance(kin, dict):
                    kin = {}
                    scene['kinematics'] = kin
                for key, value in DEFAULT_SCENE4_KINEMATICS.items():
                    if key == 'params':
                        params = kin.get(key, value)
                        if not isinstance(params, list) or len(params) != 9:
                            kin[key] = list(value)
                        else:
                            try:
                                kin[key] = [float(v) for v in params[:9]]
                            except Exception:
                                kin[key] = list(value)
                        continue
                    try:
                        kin[key] = float(kin.get(key, value))
                    except Exception:
                        kin[key] = float(value)

                grid['color_slot_targets'] = [list(targets[key]) for key in grid['color_slots'] if key in targets]


    def setup_ui(self):
        self.setWindowTitle('nexarm 场景工具')
        self.preview_min_size = (300, 200)
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            width = min(1280, max(800, int(avail.width() * 0.96)))
            height = min(900, int(avail.height() * 0.94))
            is_embedded = avail.width() <= 1400 or avail.height() <= 900
            if avail.width() < 1000 or avail.height() < 720:
                self.preview_min_size = (180, 130)
            elif is_embedded:
                self.preview_min_size = (220, 160)
            self.setFixedSize(width, height)
        else:
            self.setFixedSize(1280, 860)
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1E1F31;
                color: #FFFFFF;
                font-family: "Microsoft YaHei", "PingFang SC", "Segoe UI", sans-serif;
                font-size: 10pt;
            }
            QLabel {
                color: #FFFFFF;
                background-color: transparent;
            }
            QScrollArea {
                border: none;
                background: transparent;
            }
            QTabWidget::pane {
                border: none;
                background: #1E1F31;
            }
            QGroupBox {
                background-color: #1E1F31;
                border: none;
                border-top: 1px solid #343645;
                margin-top: 25px;
                padding-top: 12px;
                font-size: 11pt;
                font-weight: bold;
                color: #FFFFFF;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 0px;
                padding: 0 5px;
                color: #FA8F01;
            }
            QPushButton {
                background-color: #3C3F52;
                border: 1px solid #5A5D6E;
                border-radius: 4px;
                color: #FFFFFF;
                padding: 6px 14px;
                font-weight: bold;
                min-height: 24pt;
            }
            QPushButton:hover {
                background-color: #4A4D5E;
                border-color: #FA8F01;
            }
            QPushButton:pressed {
                background-color: #2D2F3F;
                border-color: #E68200;
            }
            QPushButton#playStateButton:checked {
                background-color: #FFC107;
                border-color: #FFD54F;
                color: #1E1F31;
            }
            QPushButton#playStateButton:checked:hover {
                background-color: #FFD54F;
                border-color: #FFE082;
                color: #1E1F31;
            }
            QPushButton#dangerButton {
                background-color: #FF8F00;
                border-color: #FA8F01;
                color: #1E1F31;
            }
            QPushButton#dangerButton:hover {
                background-color: #FA8F01;
                border-color: #FFC107;
            }
            QComboBox, QDoubleSpinBox {
                background-color: #343645;
                border: 1px solid #4A4D5E;
                border-radius: 4px;
                color: #FFFFFF;
                padding: 4px 8px;
                min-height: 20pt;
                selection-background-color: #FA8F01;
                selection-color: #FFFFFF;
            }
            QComboBox:focus, QDoubleSpinBox:focus {
                border: 2px solid #FA8F01;
                background-color: #3A3C4D;
            }
            QComboBox QAbstractItemView {
                background-color: #252735;
                color: #FFFFFF;
                border: 1px solid #4A4D5E;
                selection-background-color: #4A4D5E;
            }
        """)
        root = QWidget(self)
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        self.tabs.setTabBar(UprightWestTabBar())
        self.tabs.setTabPosition(QTabWidget.West)
        root_layout.addWidget(self.tabs)

        # ── 全场景标定：上方两图 + 下方控件网格 ──
        scene1_page = QWidget()
        scene1_outer = QVBoxLayout(scene1_page)
        scene1_outer.setContentsMargins(10, 10, 10, 10)
        scene1_outer.setSpacing(8)
        self.add_scroll_tab(scene1_page, '全场景标定')

        # ── 上：颜色标定 | 深度标定 两图横排 ──
        self.calib_result_preview = self._create_calib_preview_label(
            f'颜色标定\n{CALIB_DISPLAY_IMAGE_TOPIC}')
        self.calib_depth_preview = self._create_calib_preview_label(
            f'深度标定\n{CALIB_DEPTH_IMAGE_TOPIC}')
        self.calib_rgb_preview = None
        self._calib_result_img_size = (640, 480)
        self._calib_depth_img_size = (640, 480)
        self.calib_result_preview.clicked_at.connect(self._on_calib_result_click)
        self.calib_depth_preview.clicked_at.connect(self._on_calib_depth_click)

        color_box = QGroupBox('颜色标定')
        _cl = QVBoxLayout(color_box)
        _cl.setContentsMargins(4, 4, 4, 4)
        _cl.addWidget(self.calib_result_preview, 1)

        depth_box = QGroupBox('深度标定')
        _dl = QVBoxLayout(depth_box)
        _dl.setContentsMargins(4, 4, 4, 4)
        _dl.addWidget(self.calib_depth_preview, 1)

        calib_img_row = QHBoxLayout()
        calib_img_row.setSpacing(8)
        calib_img_row.addWidget(color_box, 1)
        calib_img_row.addWidget(depth_box, 1)
        scene1_outer.addLayout(calib_img_row, 3)

        # ── 下：两列控件（场景/功能 | 校准参数/按钮）──
        ctrl_row = QHBoxLayout()
        ctrl_row.setSpacing(12)

        # 左列：场景配置 + 功能
        left_col = QVBoxLayout()
        left_col.setSpacing(6)

        scene_box = QGroupBox('场景配置')
        scene_layout = QGridLayout(scene_box)
        self.cb_scene = QComboBox()
        scene_layout.addWidget(QLabel('场景'), 0, 0)
        scene_layout.addWidget(self.cb_scene, 0, 1)
        left_col.addWidget(scene_box)

        mode_box = QGroupBox('功能')
        mode_layout = QHBoxLayout(mode_box)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(['标定', '像素定位', '深度定位', '运动学校准'])
        mode_layout.addWidget(QLabel('功能选择'))
        mode_layout.addWidget(self.combo_mode)
        left_col.addWidget(mode_box)
        left_col.addStretch()
        ctrl_row.addLayout(left_col, 1)

        # 右列：校准参数 + 按钮
        right_col = QVBoxLayout()
        right_col.setSpacing(6)

        calib_box = QGroupBox('校准参数')
        calib_layout = QGridLayout(calib_box)
        self.offset_x = QDoubleSpinBox(); self.offset_x.setRange(-1.0, 1.0); self.offset_x.setDecimals(3); self.offset_x.setSingleStep(0.001)
        self.offset_y = QDoubleSpinBox(); self.offset_y.setRange(-1.0, 1.0); self.offset_y.setDecimals(3); self.offset_y.setSingleStep(0.001)
        self.offset_z = QDoubleSpinBox(); self.offset_z.setRange(-1.0, 1.0); self.offset_z.setDecimals(3); self.offset_z.setSingleStep(0.001)
        self.scale_x = QDoubleSpinBox(); self.scale_x.setRange(0.0, 2.0); self.scale_x.setDecimals(2); self.scale_x.setSingleStep(0.01); self.scale_x.setValue(1.0)
        self.scale_y = QDoubleSpinBox(); self.scale_y.setRange(0.0, 2.0); self.scale_y.setDecimals(2); self.scale_y.setSingleStep(0.01); self.scale_y.setValue(1.0)
        self.scale_z = QDoubleSpinBox(); self.scale_z.setRange(0.0, 2.0); self.scale_z.setDecimals(2); self.scale_z.setSingleStep(0.01); self.scale_z.setValue(1.0)
        params = [('定位X偏移', self.offset_x), ('定位Y偏移', self.offset_y), ('定位Z偏移', self.offset_z),
                  ('定位X缩放', self.scale_x), ('定位Y缩放', self.scale_y), ('定位Z缩放', self.scale_z)]
        for i, (name, sp) in enumerate(params):
            calib_layout.addWidget(QLabel(name), i // 3, (i % 3) * 2)
            calib_layout.addWidget(sp, i // 3, (i % 3) * 2 + 1)
        right_col.addWidget(calib_box)

        action_layout = QHBoxLayout()
        self.pushButton_init = QPushButton('标定/复位')
        self.pushButton_reset = QPushButton('重置')
        self.pushButton_save = QPushButton('保存')
        self.pushButton_save_scene = QPushButton('保存场景')
        self.pushButton_clear_grab_calib = QPushButton('清除校准')
        self.pushButton_clear_grab_calib.setVisible(False)
        action_layout.addWidget(self.pushButton_init)
        action_layout.addWidget(self.pushButton_reset)
        action_layout.addWidget(self.pushButton_save)
        action_layout.addWidget(self.pushButton_save_scene)
        action_layout.addWidget(self.pushButton_clear_grab_calib)
        right_col.addLayout(action_layout)
        right_col.addStretch()
        ctrl_row.addLayout(right_col, 2)

        scene1_outer.addLayout(ctrl_row, 2)
        # main_layout 兼容 refresh_target_editor 等使用 button_object
        main_layout = scene1_outer

        self.cb_scene.currentIndexChanged.connect(self.on_scene_changed)
        self.combo_mode.currentIndexChanged.connect(self.index_changed)
        self.pushButton_init.pressed.connect(lambda: self.button_clicked('init'))
        self.pushButton_reset.pressed.connect(lambda: self.button_clicked('reset'))
        self.pushButton_save.pressed.connect(lambda: self.button_clicked('save'))
        self.pushButton_save_scene.pressed.connect(self.save_current_scene)
        self.pushButton_clear_grab_calib.pressed.connect(lambda: self.button_clicked('clear_grab_calib'))

        self.offset_x.valueChanged.connect(lambda value: self.value_changed('offset', 0, value))
        self.offset_y.valueChanged.connect(lambda value: self.value_changed('offset', 1, value))
        self.offset_z.valueChanged.connect(lambda value: self.value_changed('offset', 2, value))
        self.scale_x.valueChanged.connect(lambda value: self.value_changed('scale', 0, value))
        self.scale_y.valueChanged.connect(lambda value: self.value_changed('scale', 1, value))
        self.scale_z.valueChanged.connect(lambda value: self.value_changed('scale', 2, value))

        self.setup_scene1_tab()
        self.setup_scene2_tab()
        self.setup_scene3_tab()
        self.setup_scene4_tab()
        self.setup_scene5_tab()

    def add_scroll_tab(self, page, title):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(page)
        self.tabs.addTab(scroll, title)

    def configure_status_label(self, label, color='#B0BEC5', max_height=36):
        label.setStyleSheet(f'color:{color};')
        label.setWordWrap(False)
        label.setMaximumHeight(max_height)
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        return label

    def _compact_status_text(self, text, limit=64):
        compact = ' '.join(str(text or '').split())
        if len(compact) <= limit:
            return compact
        return compact[:max(0, limit - 3)] + '...'

    def _set_status_label_text(self, label, text, limit=64):
        label.setToolTip(str(text or ''))
        label.setText(self._compact_status_text(text, limit))

    def configure_play_state_buttons(self, start_button, stop_button, running=False):
        for button in (start_button, stop_button):
            button.setObjectName('playStateButton')
            button.setCheckable(True)
            button.setAutoExclusive(False)
        self.set_play_state_buttons(start_button, stop_button, running)

    def set_play_state_buttons(self, start_button, stop_button, running):
        for button, checked in ((start_button, bool(running)), (stop_button, not bool(running))):
            button.blockSignals(True)
            button.setChecked(checked)
            button.blockSignals(False)

    def set_scene_play_state(self, scene_id, running):
        pairs = {
            DEFAULT_SCENE_ID: ('btn_scene1_start_play', 'btn_scene1_stop'),
            SCENE2_ID: ('btn_scene2_start_play', 'btn_scene2_stop'),
            SCENE3_ID: ('btn_scene3_start_play', 'btn_scene3_stop'),
            SCENE4_ID: ('btn_scene4_start_play', 'btn_scene4_stop'),
            SCENE5_ID: ('btn_scene5_start_pipeline', 'btn_scene5_stop'),
        }
        names = pairs.get(scene_id)
        if not names:
            return
        start_button = getattr(self, names[0], None)
        stop_button = getattr(self, names[1], None)
        if start_button is not None and stop_button is not None:
            self.set_play_state_buttons(start_button, stop_button, running)

    _DARK_DIALOG_STYLE = """
        QDialog, QWidget {
            background-color: #1E1F31;
            color: #FFFFFF;
            font-family: "Microsoft YaHei", "PingFang SC", "Segoe UI", sans-serif;
            font-size: 10pt;
        }
        QLabel { color: #FFFFFF; background: transparent; }
        QGroupBox {
            background: #1E1F31; border: none;
            border-top: 1px solid #343645; margin-top: 22px;
            padding-top: 10px; font-size: 11pt; font-weight: bold; color: #FFFFFF;
        }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left;
            left: 0px; padding: 0 5px; color: #FA8F01; }
        QPushButton {
            background-color: #3C3F52; border: 1px solid #5A5D6E; border-radius: 4px;
            color: #FFFFFF; padding: 6px 14px; font-weight: bold; min-height: 24pt;
        }
        QPushButton:hover { background-color: #4A4D5E; border-color: #FA8F01; }
        QDoubleSpinBox {
            background-color: #343645; border: 1px solid #4A4D5E; border-radius: 4px;
            color: #FFFFFF; padding: 4px 8px; min-height: 20pt;
        }
        QDoubleSpinBox:focus { border: 2px solid #FA8F01; }
    """

    def _make_coord_dialog(self, title, inner_widget):
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.setStyleSheet(self._DARK_DIALOG_STYLE)
        dlg.setMinimumWidth(480)
        dlg_layout = QVBoxLayout(dlg)
        dlg_layout.setContentsMargins(12, 12, 12, 8)
        dlg_layout.setSpacing(8)
        dlg_layout.addWidget(inner_widget)
        close_btn = QPushButton('关闭')
        close_btn.setMaximumWidth(120)
        close_btn.clicked.connect(dlg.close)
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(close_btn)
        dlg_layout.addLayout(btn_row)
        return dlg

    def global_place_offset_cfg(self):
        offset = normalize_global_place_offset(self.scene_cfg.get('global_place_offset'))
        self.scene_cfg['global_place_offset'] = offset
        return offset

    def _build_global_place_offset_dialog(self):
        dlg = QDialog(self)
        dlg.setWindowTitle('全场景放置偏差微调')
        dlg.setStyleSheet(self._DARK_DIALOG_STYLE)
        dlg.setMinimumWidth(360)
        layout = QVBoxLayout(dlg)
        layout.setContentsMargins(12, 12, 12, 8)
        layout.setSpacing(8)

        box = QGroupBox('XYZ 放置偏差 (mm)', dlg)
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        self.sp_global_place_offset_x = QDoubleSpinBox()
        self.sp_global_place_offset_y = QDoubleSpinBox()
        self.sp_global_place_offset_z = QDoubleSpinBox()
        for spin in (self.sp_global_place_offset_x, self.sp_global_place_offset_y, self.sp_global_place_offset_z):
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(1)
            spin.setSingleStep(0.5)
            spin.setSuffix(' mm')
            spin.setMinimumHeight(32)
        grid.addWidget(QLabel('X 偏差'), 0, 0)
        grid.addWidget(self.sp_global_place_offset_x, 0, 1)
        grid.addWidget(QLabel('Y 偏差'), 1, 0)
        grid.addWidget(self.sp_global_place_offset_y, 1, 1)
        grid.addWidget(QLabel('Z 偏差'), 2, 0)
        grid.addWidget(self.sp_global_place_offset_z, 2, 1)
        layout.addWidget(box)

        btn_row = QHBoxLayout()
        btn_save = QPushButton('保存')
        btn_close = QPushButton('关闭')
        btn_save.clicked.connect(self.save_global_place_offset)
        btn_close.clicked.connect(dlg.close)
        btn_row.addStretch()
        btn_row.addWidget(btn_save)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)
        return dlg

    def refresh_global_place_offset_editor(self):
        if not hasattr(self, 'sp_global_place_offset_x'):
            return
        offset = self.global_place_offset_cfg()
        for spin in (self.sp_global_place_offset_x, self.sp_global_place_offset_y, self.sp_global_place_offset_z):
            spin.blockSignals(True)
        self.sp_global_place_offset_x.setValue(float(offset['x']) * 1000.0)
        self.sp_global_place_offset_y.setValue(float(offset['y']) * 1000.0)
        self.sp_global_place_offset_z.setValue(float(offset.get('z', 0.0)) * 1000.0)
        for spin in (self.sp_global_place_offset_x, self.sp_global_place_offset_y, self.sp_global_place_offset_z):
            spin.blockSignals(False)

    def show_global_place_offset_dialog(self):
        if not hasattr(self, 'global_place_offset_dialog'):
            self.global_place_offset_dialog = self._build_global_place_offset_dialog()
        self.refresh_global_place_offset_editor()
        self.global_place_offset_dialog.show()
        self.global_place_offset_dialog.raise_()
        self.global_place_offset_dialog.activateWindow()

    def save_global_place_offset(self):
        if not hasattr(self, 'sp_global_place_offset_x'):
            return
        for spin in (self.sp_global_place_offset_x, self.sp_global_place_offset_y, self.sp_global_place_offset_z):
            spin.interpretText()
        offset = normalize_global_place_offset({
            'x': self.sp_global_place_offset_x.value() / 1000.0,
            'y': self.sp_global_place_offset_y.value() / 1000.0,
            'z': self.sp_global_place_offset_z.value() / 1000.0,
        })
        self.scene_cfg['global_place_offset'] = offset
        self.save_scene_cfg()
        text = (f'放置偏差已保存: X={offset["x"]*1000:.1f}mm '
                f'Y={offset["y"]*1000:.1f}mm Z={offset["z"]*1000:.1f}mm')
        for label_name in ('scene2_status', 'scene3_status', 'scene4_status', 'scene5_status'):
            label = getattr(self, label_name, None)
            if label is not None:
                self._set_status_label_text(label, text)
        QMessageBox.information(self, '保存成功', text)

    def _build_scene2_coord_editor(self, parent_widget):
        self.scene2_coord_box = QGroupBox('颜色放置坐标 (m)', parent_widget)
        coord_layout = QGridLayout(self.scene2_coord_box)
        coord_layout.setHorizontalSpacing(8)
        coord_layout.setVerticalSpacing(8)
        self.scene2_coord_kind_label = QLabel('目标')
        coord_layout.addWidget(self.scene2_coord_kind_label, 0, 0)
        coord_layout.addWidget(QLabel('X'), 0, 1)
        coord_layout.addWidget(QLabel('Y'), 0, 2)
        coord_layout.addWidget(QLabel('Z'), 0, 3)
        self.scene2_coord_rows = []
        for row_idx in range(1, 5):
            label = QLabel('')
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumWidth(52)
            coord_layout.addWidget(label, row_idx, 0)
            spins = []
            for col_idx, axis in enumerate(('x', 'y', 'z'), start=1):
                spin = QDoubleSpinBox()
                spin.setRange(-0.500 if axis != 'z' else 0.000, 0.500 if axis != 'z' else 0.300)
                spin.setDecimals(3)
                spin.setSingleStep(0.001)
                spin.valueChanged.connect(lambda value, r=row_idx - 1, a=col_idx - 1: self.on_scene2_coord_row_changed(r, a, value))
                coord_layout.addWidget(spin, row_idx, col_idx)
                spins.append(spin)
            self.scene2_coord_rows.append({'key': None, 'label': label, 'spins': tuple(spins)})
        return self.scene2_coord_box

    def setup_scene1_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        left = QWidget()
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        control_box = QGroupBox('场景1玩法控制')
        control_layout = QGridLayout(control_box)
        self.btn_scene1_start_play = QPushButton('开启分拣')
        self.btn_scene1_stop = QPushButton('关闭分拣')
        for btn in (self.btn_scene1_start_play, self.btn_scene1_stop):
            btn.setMinimumHeight(38)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.configure_play_state_buttons(self.btn_scene1_start_play, self.btn_scene1_stop, running=False)
        control_layout.addWidget(self.btn_scene1_start_play, 0, 0)
        control_layout.addWidget(self.btn_scene1_stop, 0, 1)
        layout.addWidget(control_box)

        self.scene1_status = QLabel('基础分拣沙盘：色块和垃圾一起夹取。')
        self.configure_status_label(self.scene1_status)
        layout.addWidget(self.scene1_status)
        layout.addStretch()

        _scene1_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sandbox_basic.png')
        _scene1_img_label = QLabel()
        _scene1_img_label.setAlignment(Qt.AlignCenter)
        _scene1_img_label.setStyleSheet('QLabel{background:transparent;border:none;}')
        _scene1_img_label.setMaximumSize(360, 280)
        _scene1_img_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        _pix = QPixmap(_scene1_img_path)
        if not _pix.isNull():
            _scene1_img_label.setPixmap(
                _pix.scaled(348, 268, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        _img_row = QHBoxLayout()
        _img_row.addStretch()
        _img_row.addWidget(_scene1_img_label)
        layout.addLayout(_img_row)

        outer.addWidget(left, 2)

        right = QWidget()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        self.scene1_rgb_preview = self._create_scene2_preview_label('等待原始图像\n' + SCENE2_RGB_IMAGE_TOPIC)
        self.scene1_processed_preview = self._create_scene2_preview_label(
            '等待处理图像\n' + SCENE2_RESULT_IMAGE_TOPIC + '\n' + SCENE3_WASTE_IMAGE_TOPIC
        )
        rgb_box = QGroupBox('原始图像')
        rgb_layout = QVBoxLayout(rgb_box)
        rgb_layout.setContentsMargins(4, 4, 4, 4)
        rgb_layout.addWidget(self.scene1_rgb_preview, 1)
        processed_box = QGroupBox('处理图像')
        processed_layout = QVBoxLayout(processed_box)
        processed_layout.setContentsMargins(4, 4, 4, 4)
        processed_layout.addWidget(self.scene1_processed_preview, 1)
        right_layout.addWidget(rgb_box, 1)
        right_layout.addWidget(processed_box, 1)

        outer.addWidget(right, 3)

        self.add_scroll_tab(page, BUILTIN_SCENE_NAMES[DEFAULT_SCENE_ID])
        self.btn_scene1_start_play.pressed.connect(self.start_scene1_all_sorting)
        self.btn_scene1_stop.pressed.connect(self.stop_scene1_tasks)

    def setup_scene2_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # ── 左侧：控制区 ──
        left = QWidget()
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        control_box = QGroupBox('场景2玩法控制')
        control_layout = QGridLayout(control_box)
        self.btn_scene2_start_play = QPushButton('开启分拣')
        self.btn_scene2_stop = QPushButton('关闭分拣')
        for btn in (self.btn_scene2_start_play, self.btn_scene2_stop):
            btn.setMinimumHeight(38)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.configure_play_state_buttons(self.btn_scene2_start_play, self.btn_scene2_stop, running=False)
        control_layout.addWidget(self.btn_scene2_start_play, 0, 0)
        control_layout.addWidget(self.btn_scene2_stop, 0, 1)
        self.btn_scene2_place_offset = QPushButton('放置偏差微调')
        self.btn_scene2_place_offset.setMinimumHeight(36)
        control_layout.addWidget(self.btn_scene2_place_offset, 1, 0, 1, 2)
        layout.addWidget(control_box)

        # 坐标编辑对话框
        self._build_scene2_coord_editor(None)
        self.scene2_coord_dialog = self._make_coord_dialog('颜色放置坐标编辑', self.scene2_coord_box)

        self.scene2_status = QLabel('')
        self.configure_status_label(self.scene2_status)
        layout.addWidget(self.scene2_status)
        self.scene2_action_status = QLabel('场景2使用固定放置位置；放置偏差在本场景中微调。')
        self.configure_status_label(self.scene2_action_status)
        layout.addWidget(self.scene2_action_status)
        layout.addStretch()

        _s2_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sandbox_scene2.png')
        _s2_img_label = QLabel()
        _s2_img_label.setAlignment(Qt.AlignCenter)
        _s2_img_label.setStyleSheet('QLabel{background:transparent;border:none;}')
        _s2_img_label.setMaximumSize(360, 280)
        _s2_img_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        _s2_pix = QPixmap(_s2_img_path)
        if not _s2_pix.isNull():
            _s2_img_label.setPixmap(_s2_pix.scaled(348, 268, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        _s2_img_row = QHBoxLayout()
        _s2_img_row.addStretch()
        _s2_img_row.addWidget(_s2_img_label)
        layout.addLayout(_s2_img_row)

        outer.addWidget(left, 2)

        # ── 右侧：回传画面（全高度）──
        right = QWidget()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        self.scene2_result_preview = self._create_scene2_preview_label('等待颜色夹取图像\n' + SCENE2_RESULT_IMAGE_TOPIC)
        self.scene2_rgb_preview = self._create_scene2_preview_label('等待原始图像\n' + SCENE2_RGB_IMAGE_TOPIC)
        self.scene2_result_box = QGroupBox('颜色夹取图像')
        result_layout = QVBoxLayout(self.scene2_result_box)
        result_layout.setContentsMargins(4, 4, 4, 4)
        result_layout.addWidget(self.scene2_result_preview, 1)
        rgb_box = QGroupBox('原始图像')
        rgb_layout = QVBoxLayout(rgb_box)
        rgb_layout.setContentsMargins(4, 4, 4, 4)
        rgb_layout.addWidget(self.scene2_rgb_preview, 1)
        right_layout.addWidget(self.scene2_result_box, 1)
        right_layout.addWidget(rgb_box, 1)

        outer.addWidget(right, 3)

        self.add_scroll_tab(page, BUILTIN_SCENE_NAMES[SCENE2_ID])
        self.btn_scene2_start_play.pressed.connect(self.start_scene2_all_sorting)
        self.btn_scene2_stop.pressed.connect(self.stop_scene2_color_sorting)
        self.btn_scene2_place_offset.pressed.connect(self.show_global_place_offset_dialog)

    def _create_calib_preview_label(self, text):
        label = ClickablePreviewLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumSize(*self.preview_min_size)
        label.setStyleSheet(
            'QLabel{background:#2D2F3F;border:1px solid #4A4D5E;'
            'border-radius:8px;color:#B0BEC5;font-weight:bold;}'
        )
        label.setWordWrap(True)
        return label

    def _on_calib_result_click(self, fx, fy):
        w, h = self._calib_result_img_size
        self._send_calib_click('color', fx * w, fy * h)

    def _on_calib_depth_click(self, fx, fy):
        w, h = self._calib_depth_img_size
        self._send_calib_click('depth', fx * w, fy * h)

    def _send_calib_click(self, mode, x, y):
        if self.node is not None:
            self.node.publish_calib_click(mode, x, y)

    def _create_scene2_preview_label(self, text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumSize(*self.preview_min_size)
        label.setStyleSheet(
            'QLabel{background:#2D2F3F;border:1px solid #4A4D5E;'
            'border-radius:8px;color:#B0BEC5;font-weight:bold;}'
        )
        label.setWordWrap(True)
        return label

    def _build_scene3_coord_editor(self, parent_widget):
        coord_box = QGroupBox('对应位置坐标 (m)', parent_widget)
        coord_layout = QGridLayout(coord_box)
        coord_layout.setHorizontalSpacing(8)
        coord_layout.setVerticalSpacing(8)
        coord_layout.addWidget(QLabel('目标'), 0, 0)
        coord_layout.addWidget(QLabel('X'), 0, 1)
        coord_layout.addWidget(QLabel('Y'), 0, 2)
        coord_layout.addWidget(QLabel('Z'), 0, 3)
        self.scene3_coord_spins = {}
        rows = list(SCENE2_COLOR_KEYS) + list(WASTE_KEYS)
        for row_idx, key in enumerate(rows, start=1):
            label_text = SCENE2_COLOR_LABELS.get(key, WASTE_LABELS.get(key, key))
            bg = SCENE2_COLOR_QCOLORS.get(key, WASTE_QCOLORS.get(key, QColor(180, 180, 180))).name()
            label = QLabel(label_text)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumWidth(70)
            label.setStyleSheet(
                f'background:{bg};border:1px solid #5A5D6E;border-radius:4px;'
                'font-weight:bold;padding:5px;color:#111827;'
            )
            coord_layout.addWidget(label, row_idx, 0)
            spins = []
            for col_idx, axis in enumerate(('x', 'y', 'z'), start=1):
                spin = QDoubleSpinBox()
                spin.setRange(-0.500 if axis != 'z' else 0.000, 0.500 if axis != 'z' else 0.300)
                spin.setDecimals(3)
                spin.setSingleStep(0.001)
                spin.setKeyboardTracking(False)
                spin.valueChanged.connect(lambda value, k=key, a=col_idx - 1: self.on_scene3_coord_changed(k, a, value))
                coord_layout.addWidget(spin, row_idx, col_idx)
                spins.append(spin)
            self.scene3_coord_spins[key] = tuple(spins)
        return coord_box

    def setup_scene3_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # ── 左侧：控制区 ──
        left = QWidget()
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        control_box = QGroupBox('场景3抓取控制')
        control_layout = QGridLayout(control_box)
        self.btn_scene3_start_play = QPushButton('开启分拣')
        self.btn_scene3_stop = QPushButton('关闭分拣')
        for btn in (self.btn_scene3_start_play, self.btn_scene3_stop):
            btn.setMinimumHeight(38)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.configure_play_state_buttons(self.btn_scene3_start_play, self.btn_scene3_stop, running=False)
        control_layout.addWidget(self.btn_scene3_start_play, 0, 0)
        control_layout.addWidget(self.btn_scene3_stop, 0, 1)
        self.btn_scene3_place_offset = QPushButton('放置偏差微调')
        self.btn_scene3_place_offset.setMinimumHeight(36)
        control_layout.addWidget(self.btn_scene3_place_offset, 1, 0, 1, 2)
        layout.addWidget(control_box)

        # 坐标编辑对话框
        coord_box = self._build_scene3_coord_editor(None)
        self.scene3_coord_dialog = self._make_coord_dialog('场景3放置坐标编辑', coord_box)

        self.scene3_status = QLabel('场景3使用固定放置位置；放置偏差在本场景中微调。')
        self.configure_status_label(self.scene3_status)
        layout.addWidget(self.scene3_status)
        layout.addStretch()

        _s3_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sandbox_scene3.png')
        _s3_img_label = QLabel()
        _s3_img_label.setAlignment(Qt.AlignCenter)
        _s3_img_label.setStyleSheet('QLabel{background:transparent;border:none;}')
        _s3_img_label.setMaximumSize(360, 280)
        _s3_img_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        _s3_pix = QPixmap(_s3_img_path)
        if not _s3_pix.isNull():
            _s3_img_label.setPixmap(_s3_pix.scaled(348, 268, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        _s3_img_row = QHBoxLayout()
        _s3_img_row.addStretch()
        _s3_img_row.addWidget(_s3_img_label)
        layout.addLayout(_s3_img_row)

        outer.addWidget(left, 2)

        # ── 右侧：回传画面（全高度）──
        right = QWidget()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        self.scene3_rgb_preview = self._create_scene2_preview_label('等待原始图像\n' + SCENE2_RGB_IMAGE_TOPIC)
        self.scene3_result_preview = self._create_scene2_preview_label('点击左侧色块或右侧垃圾分类后显示对应结果图')
        rgb_box = QGroupBox('原始图像')
        rgb_layout = QVBoxLayout(rgb_box)
        rgb_layout.setContentsMargins(4, 4, 4, 4)
        rgb_layout.addWidget(self.scene3_rgb_preview, 1)
        self.scene3_result_box = QGroupBox('当前结果图像')
        result_layout = QVBoxLayout(self.scene3_result_box)
        result_layout.setContentsMargins(4, 4, 4, 4)
        result_layout.addWidget(self.scene3_result_preview, 1)
        right_layout.addWidget(rgb_box, 1)
        right_layout.addWidget(self.scene3_result_box, 1)

        outer.addWidget(right, 3)

        self.add_scroll_tab(page, BUILTIN_SCENE_NAMES[SCENE3_ID])
        self.btn_scene3_start_play.pressed.connect(self.start_scene3_all_sorting)
        self.btn_scene3_stop.pressed.connect(self.stop_scene3_tasks)
        self.btn_scene3_place_offset.pressed.connect(self.show_global_place_offset_dialog)
        self.refresh_scene3_coord_editor()

    def scene3_cfg(self):
        scenes = self.scene_cfg.setdefault('scenes', {})
        if SCENE3_ID not in scenes:
            scenes[SCENE3_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE3_ID]))
        self.normalize_scene_cfg(self.scene_cfg)
        return scenes[SCENE3_ID]

    def scene3_grid(self):
        scene = self.scene3_cfg()
        grid = normalized_scene3_grid(scene.get('scene3_grid'))
        scene['scene3_grid'] = grid
        return grid

    def scene3_slots(self, group):
        grid = self.scene3_grid()
        return list(grid[SCENE3_GRID_SLOT_FIELDS[group]])

    def scene3_slot_targets(self, group):
        grid = self.scene3_grid()
        return [list(pos) for pos in grid[SCENE3_GRID_TARGET_FIELDS[group]]]

    def apply_scene3_slots_to_targets(self, group=None):
        scene = self.scene3_cfg()
        grid = self.scene3_grid()
        targets = scene.setdefault('place_targets', {})
        groups = (group,) if group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE) else (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE)
        for current_group in groups:
            slots = grid[SCENE3_GRID_SLOT_FIELDS[current_group]]
            slot_targets = grid[SCENE3_GRID_TARGET_FIELDS[current_group]]
            for idx, key in enumerate(slots):
                targets[key] = list(slot_targets[idx])

    def refresh_scene3_board(self):
        if hasattr(self, 'scene3_board'):
            self.scene3_board.set_slots(
                self.scene3_slots(SCENE3_GROUP_COLOR),
                self.scene3_slots(SCENE3_GROUP_WASTE),
            )

    def refresh_scene3_coord_editor(self):
        self.refresh_scene3_board()
        if not hasattr(self, 'scene3_coord_spins'):
            return
        scene = self.scene3_cfg()
        targets = scene.setdefault('place_targets', {})
        self._updating_scene3_coords = True
        try:
            for key, spins in self.scene3_coord_spins.items():
                defaults = SCENE3_COLOR_TARGETS if key in SCENE2_COLOR_KEYS else SCENE3_WASTE_TARGETS
                pos = targets.get(key, defaults.get(key, [0.0, 0.0, 0.015]))
                if not isinstance(pos, list) or len(pos) != 3:
                    pos = defaults.get(key, [0.0, 0.0, 0.015])
                for spin, value in zip(spins, pos):
                    spin.blockSignals(True)
                    spin.setValue(float(value))
                    spin.blockSignals(False)
        finally:
            self._updating_scene3_coords = False

    def _set_scene3_target_position(self, scene, key, pos):
        pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        targets = scene.setdefault('place_targets', {})
        targets[key] = list(pos)
        grid = normalized_scene3_grid(scene.get('scene3_grid'))
        scene['scene3_grid'] = grid
        for group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE):
            slots = grid[SCENE3_GRID_SLOT_FIELDS[group]]
            if key in slots:
                grid[SCENE3_GRID_TARGET_FIELDS[group]][slots.index(key)] = list(pos)

    def commit_scene3_coord_editor(self):
        if not hasattr(self, 'scene3_coord_spins'):
            return
        old_updating = getattr(self, '_updating_scene3_coords', False)
        self._updating_scene3_coords = True
        try:
            for spins in self.scene3_coord_spins.values():
                for spin in spins:
                    spin.interpretText()
        finally:
            self._updating_scene3_coords = old_updating
        scene = self.scene3_cfg()
        for key, spins in self.scene3_coord_spins.items():
            self._set_scene3_target_position(scene, key, [spin.value() for spin in spins])

    def on_scene3_coord_changed(self, key, axis, value):
        if self._updating_scene3_coords:
            return
        scene = self.scene3_cfg()
        targets = scene.setdefault('place_targets', {})
        defaults = SCENE3_COLOR_TARGETS if key in SCENE2_COLOR_KEYS else SCENE3_WASTE_TARGETS
        pos = list(targets.get(key, defaults.get(key, [0.0, 0.0, 0.015])))
        if len(pos) != 3:
            pos = list(defaults.get(key, [0.0, 0.0, 0.015]))
        pos[axis] = float(value)
        self._set_scene3_target_position(scene, key, pos)
        label = SCENE2_COLOR_LABELS.get(key, WASTE_LABELS.get(key, key))
        self._set_status_label_text(self.scene3_status, f'{label} 坐标: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}')

    def save_scene3(self):
        self.commit_scene3_coord_editor()
        self.apply_scene3_slots_to_targets()
        self.scene_cfg['current_scene'] = SCENE3_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        QMessageBox.information(self, '保存成功', '场景3坐标已保存')

    def reset_scene3(self):
        scene = self.scene3_cfg()
        defaults = DEFAULT_SCENE_CONFIG['scenes'][SCENE3_ID]
        scene['length_m'] = defaults['length_m']
        scene['width_m'] = defaults['width_m']
        scene['home_pose'] = dict(DEFAULT_SCENE_HOME_POSES[SCENE3_ID])
        scene['scene3_grid'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE3_GRID))
        scene['place_pitch'] = dict(SCENE3_PLACE_PITCH)
        targets = scene.setdefault('place_targets', {})
        for key, pos in SCENE3_COLOR_TARGETS.items():
            targets[key] = list(pos)
        for key, pos in SCENE3_WASTE_TARGETS.items():
            targets[key] = list(pos)
        self.scene_cfg['current_scene'] = SCENE3_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene3_coord_editor()

    def on_scene3_grid_changed(self, group, slots):
        scene = self.scene3_cfg()
        grid = scene.setdefault('scene3_grid', {})
        grid[SCENE3_GRID_SLOT_FIELDS[group]] = normalized_unique_slots(slots, scene3_keys_for_group(group))
        self.apply_scene3_slots_to_targets(group)
        self.scene_cfg['current_scene'] = SCENE3_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene3_coord_editor()
        labels = scene3_labels_for_group(group)
        text = '  '.join(labels.get(key, key) for key in self.scene3_slots(group))
        self._set_status_label_text(self.scene3_status, f'当前顺序: {text}')

    def _set_scene3_action_status(self, text):
        if hasattr(self, 'scene3_status'):
            self._set_status_label_text(self.scene3_status, text)

    def _build_scene4_absolute_editor(self, parent_widget):
        self.scene4_absolute_box = QGroupBox('场景4绝对放置位置 (m)', parent_widget)
        absolute_layout = QGridLayout(self.scene4_absolute_box)
        absolute_layout.setHorizontalSpacing(8)
        absolute_layout.setVerticalSpacing(8)
        self.scene4_absolute_spins = {}
        absolute_layout.addWidget(QLabel('位置'), 0, 0)
        absolute_layout.addWidget(QLabel('X'), 0, 1)
        absolute_layout.addWidget(QLabel('Y'), 0, 2)
        absolute_layout.addWidget(QLabel('Z'), 0, 3)

        def add_absolute_row(row, label, field, index):
            absolute_layout.addWidget(QLabel(label), row, 0)
            for axis_index, axis_label in enumerate(('X', 'Y', 'Z')):
                spin = QDoubleSpinBox()
                if axis_label == 'Z':
                    spin.setRange(0.0, 1.0)
                else:
                    spin.setRange(-1.0, 1.0)
                spin.setDecimals(3)
                spin.setSingleStep(0.001)
                spin.setMinimumHeight(30)
                spin.valueChanged.connect(
                    lambda value, p=(field, index, axis_index): self.on_scene4_absolute_position_changed(p, value)
                )
                absolute_layout.addWidget(spin, row, axis_index + 1)
                self.scene4_absolute_spins[(field, index, axis_index)] = spin

        absolute_rows = [
            (f'田字{index + 1}', 'frame_slots', index)
            for index in range(SCENE4_FRAME_SLOT_COUNT)
        ]
        absolute_rows.extend(
            (f'上层{index + 1}', 'upper_shelf_slots', index)
            for index in range(SCENE4_SHELF_SLOT_COUNT)
        )
        absolute_rows.extend(
            (f'下层{index + 1}', 'lower_shelf_slots', index)
            for index in range(SCENE4_SHELF_SLOT_COUNT)
        )
        for row_index, (label, field, index) in enumerate(absolute_rows, 1):
            add_absolute_row(row_index, label, field, index)
        return self.scene4_absolute_box

    def setup_scene4_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # ── 左侧：控制区 ──
        left = QWidget()
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        control_box = QGroupBox('场景4抓取控制')
        control_layout = QGridLayout(control_box)
        self.btn_scene4_start_play = QPushButton('开启分拣')
        self.btn_scene4_stop = QPushButton('关闭分拣')
        for btn in (self.btn_scene4_start_play, self.btn_scene4_stop):
            btn.setMinimumHeight(40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.configure_play_state_buttons(self.btn_scene4_start_play, self.btn_scene4_stop, running=False)
        control_layout.addWidget(self.btn_scene4_start_play, 0, 0)
        control_layout.addWidget(self.btn_scene4_stop, 0, 1)
        self.btn_scene4_place_offset = QPushButton('放置偏差微调')
        self.btn_scene4_place_offset.setMinimumHeight(36)
        control_layout.addWidget(self.btn_scene4_place_offset, 1, 0, 1, 2)
        layout.addWidget(control_box)

        self.scene4_board_box = QGroupBox('场景4放置地图')
        board_layout = QVBoxLayout(self.scene4_board_box)
        self.scene4_board = Scene4BoardWidget()
        self.scene4_board.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        board_layout.addWidget(self.scene4_board, 1)
        self.btn_scene4_confirm_place = QPushButton('确认位置')
        self.btn_scene4_confirm_place.setMinimumHeight(36)
        self.btn_scene4_confirm_place.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        board_layout.addWidget(self.btn_scene4_confirm_place)
        layout.addWidget(self.scene4_board_box)

        # 绝对位置编辑对话框
        abs_box = self._build_scene4_absolute_editor(None)
        self.scene4_absolute_dialog = self._make_coord_dialog('场景4绝对放置位置编辑', abs_box)

        self.scene4_status = QLabel('场景4：田字4个位置，按颜色分拣；拖动后点确认位置，再开启分拣。')
        self.configure_status_label(self.scene4_status)
        layout.addWidget(self.scene4_status)
        layout.addStretch()

        _s4_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sandbox_scene4.png')
        _s4_img_label = QLabel()
        _s4_img_label.setAlignment(Qt.AlignCenter)
        _s4_img_label.setStyleSheet('QLabel{background:transparent;border:none;}')
        _s4_img_label.setMaximumSize(360, 280)
        _s4_img_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        _s4_pix = QPixmap(_s4_img_path)
        if not _s4_pix.isNull():
            _s4_img_label.setPixmap(_s4_pix.scaled(348, 268, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        _s4_img_row = QHBoxLayout()
        _s4_img_row.addStretch()
        _s4_img_row.addWidget(_s4_img_label)
        layout.addLayout(_s4_img_row)

        outer.addWidget(left, 2)

        # ── 右侧：回传画面（全高度）──
        right = QWidget()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        self.scene4_rgb_preview = self._create_scene2_preview_label('等待原始图像\n' + SCENE2_RGB_IMAGE_TOPIC)
        self.scene4_result_preview = self._create_scene2_preview_label('点击场景4任务后显示对应结果图')
        rgb_box = QGroupBox('原始图像')
        rgb_layout = QVBoxLayout(rgb_box)
        rgb_layout.setContentsMargins(4, 4, 4, 4)
        rgb_layout.addWidget(self.scene4_rgb_preview, 1)
        self.scene4_result_box = QGroupBox('当前结果图像')
        result_layout = QVBoxLayout(self.scene4_result_box)
        result_layout.setContentsMargins(4, 4, 4, 4)
        result_layout.addWidget(self.scene4_result_preview, 1)
        right_layout.addWidget(rgb_box, 1)
        right_layout.addWidget(self.scene4_result_box, 1)

        outer.addWidget(right, 3)

        self.add_scroll_tab(page, BUILTIN_SCENE_NAMES[SCENE4_ID])
        self.btn_scene4_start_play.pressed.connect(self.start_scene4_all_sorting)
        self.btn_scene4_stop.pressed.connect(self.stop_scene4_tasks)
        self.btn_scene4_place_offset.pressed.connect(self.show_global_place_offset_dialog)
        self.btn_scene4_confirm_place.pressed.connect(self.confirm_scene4_placement)
        self.scene4_board.frameMoved.connect(self.on_scene4_frame_moved)
        self.scene4_board.frameClicked.connect(self.on_scene4_frame_clicked)
        self.scene4_board.destinationChanged.connect(self.on_scene4_destination_changed_from_board)
        self.scene4_board.frameGridChanged.connect(self.on_scene4_frame_grid_changed)
        self.scene4_board.shelfGridChanged.connect(self.on_scene4_shelf_grid_changed)
        self.set_scene4_mode(SCENE4_MODE_COLOR)
        self.refresh_scene4_pick_editor()
        self.refresh_scene4_rail_editor()
        self.refresh_scene4_absolute_editor()

    def scene4_cfg(self):
        scenes = self.scene_cfg.setdefault('scenes', {})
        if SCENE4_ID not in scenes:
            scenes[SCENE4_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE4_ID]))
        self.normalize_scene_cfg(self.scene_cfg)
        return scenes[SCENE4_ID]

    def scene4_pick_cfg(self):
        scene = self.scene4_cfg()
        scene['scene4_pick'] = normalized_scene4_pick(scene.get('scene4_pick'))
        return scene['scene4_pick']

    def scene4_place_cfg(self):
        scene = self.scene4_cfg()
        scene['scene4_place'] = normalized_scene4_place(scene.get('scene4_place'))
        return scene['scene4_place']

    def scene4_rail_cfg(self):
        scene = self.scene4_cfg()
        rail = scene.setdefault('rail', {})
        if not isinstance(rail, dict):
            rail = {}
            scene['rail'] = rail
        for key, value in DEFAULT_SCENE4_RAIL.items():
            rail.setdefault(key, value)
        return rail

    def _scene4_rail_defaults_for_subdivision(self, subdivision):
        code = int(subdivision)
        if code == 3:
            return {
                'subdivision': 3,
                'total_steps': 8400,
                'calibration_abs_position': 8000,
                'place_abs_position': 1400,
            }
        return {
            'subdivision': 2,
            'total_steps': 4200,
            'calibration_abs_position': 4000,
            'place_abs_position': 700,
        }

    def refresh_scene4_rail_editor(self):
        if not hasattr(self, 'cb_scene4_subdivision'):
            return
        rail = self.scene4_rail_cfg()
        subdivision = int(rail.get('subdivision', DEFAULT_SCENE4_RAIL['subdivision']))
        if subdivision not in (2, 3):
            subdivision = DEFAULT_SCENE4_RAIL['subdivision']
        idx = self.cb_scene4_subdivision.findData(subdivision)
        if idx >= 0:
            self.cb_scene4_subdivision.blockSignals(True)
            self.cb_scene4_subdivision.setCurrentIndex(idx)
            self.cb_scene4_subdivision.blockSignals(False)
        self.sp_scene4_rail_speed.blockSignals(True)
        self.sp_scene4_rail_speed.setValue(float(rail.get('speed_steps_per_sec', DEFAULT_SCENE4_RAIL['speed_steps_per_sec'])))
        self.sp_scene4_rail_speed.blockSignals(False)

    def on_scene4_rail_param_changed(self, *_args):
        if not hasattr(self, 'cb_scene4_subdivision'):
            return
        rail = self.scene4_rail_cfg()
        defaults = self._scene4_rail_defaults_for_subdivision(self.cb_scene4_subdivision.currentData() or DEFAULT_SCENE4_RAIL['subdivision'])
        rail.update(defaults)
        rail['enabled'] = True
        rail['reset_wait_sec'] = float(rail.get('reset_wait_sec', DEFAULT_SCENE4_RAIL['reset_wait_sec']))
        rail['speed_steps_per_sec'] = float(self.sp_scene4_rail_speed.value())
        if hasattr(self, 'scene4_status'):
            factor = 8 if rail['subdivision'] == 3 else 4
            self._set_status_label_text(self.scene4_status, f'滑轨设置: {factor}细分，速度 {rail["speed_steps_per_sec"]:.0f} step/s')

    def current_scene4_pick_zone(self):
        return SCENE4_PICK_ZONE_LOWER

    def set_scene4_pick_zone(self, zone, save=False):
        clean_zone = SCENE4_PICK_ZONE_LOWER
        pick = self.scene4_pick_cfg()
        pick['active_zone'] = clean_zone
        self.scene_cfg['current_scene'] = SCENE4_ID
        if hasattr(self, 'cb_scene4_pick_zone'):
            idx = self.cb_scene4_pick_zone.findData(clean_zone)
            if idx >= 0 and self.cb_scene4_pick_zone.currentIndex() != idx:
                self.cb_scene4_pick_zone.blockSignals(True)
                self.cb_scene4_pick_zone.setCurrentIndex(idx)
                self.cb_scene4_pick_zone.blockSignals(False)
        if save:
            self.save_scene_cfg()
        return clean_zone

    def on_scene4_pick_zone_changed(self):
        if self._updating_scene4_pick:
            return
        zone = self.current_scene4_pick_zone()
        self.set_scene4_pick_zone(zone, save=True)
        self._update_scene4_pick_visibility(zone)
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, '色块来源固定为: 下层色块')

    def _update_scene4_pick_visibility(self, zone=None):
        if hasattr(self, 'scene4_upper_pick_box'):
            self.scene4_upper_pick_box.setVisible(True)

    def refresh_scene4_pick_editor(self):
        if not hasattr(self, 'scene4_pick_spins'):
            return
        pick = self.scene4_pick_cfg()
        self._updating_scene4_pick = True
        try:
            if hasattr(self, 'cb_scene4_pick_zone'):
                idx = self.cb_scene4_pick_zone.findData(pick.get('active_zone', SCENE4_PICK_ZONE_LOWER))
                if idx >= 0:
                    self.cb_scene4_pick_zone.blockSignals(True)
                    self.cb_scene4_pick_zone.setCurrentIndex(idx)
                    self.cb_scene4_pick_zone.blockSignals(False)
            self._update_scene4_pick_visibility(pick.get('active_zone', SCENE4_PICK_ZONE_LOWER))
            for path, spin in self.scene4_pick_spins.items():
                value = pick
                for key in path:
                    value = value.get(key, {}) if isinstance(value, dict) else {}
                spin.blockSignals(True)
                spin.setValue(float(value if not isinstance(value, dict) else 0.0))
                spin.blockSignals(False)
        finally:
            self._updating_scene4_pick = False

    def on_scene4_pick_param_changed(self, path, value):
        if self._updating_scene4_pick:
            return
        pick = self.scene4_pick_cfg()
        node = pick
        for key in path[:-1]:
            node = node.setdefault(key, {})
        node[path[-1]] = float(value)
        scene = self.scene4_cfg()
        scene['scene4_pick'] = normalized_scene4_pick(pick)
        label = ' > '.join(path)
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, f'下层取物过滤已更新: {label}={value:g}')

    def scene4_absolute_cfg(self):
        scene = self.scene4_cfg()
        scene['scene4_absolute_positions'] = normalized_scene4_absolute_positions(
            scene.get('scene4_absolute_positions')
        )
        return scene['scene4_absolute_positions']

    def refresh_scene4_absolute_editor(self):
        if not hasattr(self, 'scene4_absolute_spins'):
            return
        absolute_positions = self.scene4_absolute_cfg()
        self._updating_scene4_absolute = True
        try:
            for path, spin in self.scene4_absolute_spins.items():
                field, index, axis_index = path
                value = absolute_positions.get(field, [])[index][axis_index]
                spin.blockSignals(True)
                spin.setValue(float(value))
                spin.blockSignals(False)
        finally:
            self._updating_scene4_absolute = False

    def on_scene4_absolute_position_changed(self, path, value):
        if self._updating_scene4_absolute:
            return
        field, index, axis_index = path
        scene = self.scene4_cfg()
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        if field not in absolute_positions:
            return
        index = max(0, min(len(absolute_positions[field]) - 1, int(index)))
        axis_index = max(0, min(2, int(axis_index)))
        absolute_positions[field][index][axis_index] = round(float(value), 3)
        scene['scene4_absolute_positions'] = normalized_scene4_absolute_positions(absolute_positions)
        self._sync_scene4_mapped_targets(scene)
        self.refresh_scene4_coord_editor()
        self.refresh_scene4_absolute_editor()
        labels = {
            'frame_slots': tuple(f'田字{index + 1}' for index in range(SCENE4_FRAME_SLOT_COUNT)),
            'upper_shelf_slots': tuple(f'上层{index + 1}' for index in range(SCENE4_SHELF_SLOT_COUNT)),
            'lower_shelf_slots': tuple(f'下层{index + 1}' for index in range(SCENE4_SHELF_SLOT_COUNT)),
        }
        axes = ('X', 'Y', 'Z')
        slot_label = labels.get(field, ('位置',))[index]
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, f'{slot_label} {axes[axis_index]} 坐标已更新: {float(value):.3f}m')

    def scene4_mode_keys(self):
        return scene4_keys_for_mode(getattr(self, 'scene4_mode', SCENE4_MODE_COLOR))

    def scene4_mode_labels(self):
        return scene4_labels_for_mode(getattr(self, 'scene4_mode', SCENE4_MODE_COLOR))

    def scene4_mode_colors(self):
        return scene4_colors_for_mode(getattr(self, 'scene4_mode', SCENE4_MODE_COLOR))

    def scene4_target_label(self, key):
        if key in TAG_LABELS:
            return TAG_LABELS[key]
        if key in WASTE_LABELS:
            return WASTE_LABELS[key]
        return SCENE2_COLOR_LABELS.get(key, key)

    def scene4_frame_slots(self, scene=None, mode=None):
        scene = scene or self.scene4_cfg()
        mode = mode if mode in SCENE4_MODES else getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
        grid = scene.setdefault('scene4_grid', {})
        slot_field = SCENE4_GRID_SLOT_FIELDS[mode]
        slots = normalized_scene4_frame_slots(grid.get(slot_field), scene4_keys_for_mode(mode))
        grid[slot_field] = slots
        return slots

    def scene4_frame_slot_index(self, scene, mode, key):
        slots = self.scene4_frame_slots(scene, mode)
        try:
            return slots.index(key)
        except ValueError:
            return None

    def scene4_shelf_slots(self, scene, mode, destination):
        scene = scene or self.scene4_cfg()
        mode = mode if mode in SCENE4_MODES else getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
        destination = destination if destination in SCENE4_SHELF_LEVELS else SCENE4_SHELF_LEVEL1
        grid = scene.setdefault('scene4_grid', {})
        slot_field = SCENE4_SHELF_SLOT_FIELDS[mode][destination]
        slots = normalized_scene4_shelf_slots(grid.get(slot_field), scene4_keys_for_mode(mode), destination)
        grid[slot_field] = slots
        return slots

    def scene4_shelf_slot_index(self, scene, mode, key, destination):
        slots = self.scene4_shelf_slots(scene, mode, destination)
        try:
            return slots.index(key)
        except ValueError:
            return None

    def apply_scene4_frame_slots_to_targets(self, scene, mode):
        targets = scene.setdefault('place_targets', {})
        place = normalized_scene4_place(scene.get('scene4_place'))
        scene['scene4_place'] = place
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        slots = self.scene4_frame_slots(scene, mode)
        for index, key in enumerate(slots):
            if key and place['targets'].get(key, SCENE4_PLACE_FRAME) == SCENE4_PLACE_FRAME:
                targets[key] = scene4_fixed_position(
                    key,
                    SCENE4_PLACE_FRAME,
                    index,
                    absolute_positions=absolute_positions,
                )

    def apply_scene4_shelf_slots_to_targets(self, scene, mode, destination):
        targets = scene.setdefault('place_targets', {})
        place = normalized_scene4_place(scene.get('scene4_place'))
        scene['scene4_place'] = place
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        slots = self.scene4_shelf_slots(scene, mode, destination)
        for index, key in enumerate(slots):
            if key and place['targets'].get(key, place['default_destination']) == destination:
                targets[key] = scene4_fixed_position(
                    key,
                    destination,
                    shelf_slot_index=index,
                    absolute_positions=absolute_positions,
                )

    def set_scene4_mode(self, mode):
        mode = mode if mode in SCENE4_MODES else SCENE4_MODE_COLOR
        self.scene4_mode = mode
        titles = {
            SCENE4_MODE_COLOR: '场景4色块放置地图',
            SCENE4_MODE_WASTE: '场景4垃圾分类地图',
            SCENE4_MODE_ALL: '场景4色块垃圾放置地图',
        }
        if hasattr(self, 'scene4_board_box'):
            self.scene4_board_box.setTitle(titles.get(mode, titles[SCENE4_MODE_COLOR]))
        self.refresh_scene4_coord_editor()

    def _sync_scene4_mapped_targets(self, scene, mode=None):
        targets = scene.setdefault('place_targets', {})
        place = normalized_scene4_place(scene.get('scene4_place'))
        scene['scene4_place'] = place
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        grid = scene.setdefault('scene4_grid', {})
        modes = (mode,) if mode in SCENE4_MODES else SCENE4_CONFIG_MODES
        for current_mode in modes:
            keys = scene4_keys_for_mode(current_mode)
            slot_field = SCENE4_GRID_SLOT_FIELDS[current_mode]
            slots = normalized_scene4_frame_slots(grid.get(slot_field), keys)
            grid[slot_field] = slots
            shelf_slots = {}
            for destination in SCENE4_SHELF_LEVELS:
                shelf_slot_field = SCENE4_SHELF_SLOT_FIELDS[current_mode][destination]
                shelf_slots[destination] = normalized_scene4_shelf_slots(
                    grid.get(shelf_slot_field),
                    keys,
                    destination,
                )
                grid[shelf_slot_field] = shelf_slots[destination]
            for key in keys:
                destination = place['targets'].get(key, place['default_destination'])
                if destination == SCENE4_PLACE_FRAME:
                    try:
                        slot_index = slots.index(key)
                    except ValueError:
                        slot_index = keys.index(key)
                    targets[key] = scene4_fixed_position(
                        key,
                        destination,
                        slot_index,
                        absolute_positions=absolute_positions,
                    )
                elif destination in SCENE4_SHELF_LEVELS:
                    try:
                        shelf_slot_index = shelf_slots[destination].index(key)
                    except ValueError:
                        shelf_slot_index = None
                    targets[key] = scene4_fixed_position(
                        key,
                        destination,
                        shelf_slot_index=shelf_slot_index,
                        absolute_positions=absolute_positions,
                    )
        grid['color_slot_targets'] = [
            list(normalized_position(targets.get(key), scene4_default_position(key)))
            for key in grid['color_slots']
            if key
        ]

    def refresh_scene4_coord_editor(self):
        scene = self.scene4_cfg()
        targets = scene.setdefault('place_targets', {})
        self._sync_scene4_mapped_targets(scene)
        place = scene.get('scene4_place', normalized_scene4_place({}))
        shelf = normalized_scene4_shelf(scene.get('scene4_shelf'))
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        mode = getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
        keys = self.scene4_mode_keys()
        labels = self.scene4_mode_labels()
        colors = self.scene4_mode_colors()
        slots = self.scene4_frame_slots(scene, mode)
        shelf_slots = {
            destination: self.scene4_shelf_slots(scene, mode, destination)
            for destination in SCENE4_SHELF_LEVELS
        }
        if hasattr(self, 'scene4_board'):
            self.scene4_board.set_scene_mode(mode, keys, labels, colors)
            self.scene4_board.set_frame_slots(slots)
            self.scene4_board.set_scene_targets(
                targets,
                length_m=float(shelf.get('length_m', SCENE4_SHELF_LENGTH_M)),
                width_m=float(shelf.get('width_m', SCENE4_SHELF_WIDTH_M)),
                destinations=place.get('targets', {}),
                shelf_slots=shelf_slots,
                absolute_positions=absolute_positions,
            )

    def on_scene4_destination_changed_from_board(self, key, destination):
        scene = self.scene4_cfg()
        place = self.scene4_place_cfg()
        if destination not in (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS):
            destination = SCENE4_PLACE_FRAME
        place['targets'][key] = destination
        targets = scene.setdefault('place_targets', {})
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        if destination in SCENE4_SHELF_LEVELS:
            shelf_slot_index = self.scene4_shelf_slot_index(
                scene,
                getattr(self, 'scene4_mode', SCENE4_MODE_COLOR),
                key,
                destination,
            )
            targets[key] = scene4_fixed_position(
                key,
                destination,
                shelf_slot_index=shelf_slot_index,
                absolute_positions=absolute_positions,
            )
        elif destination == SCENE4_PLACE_FRAME:
            slot_index = self.scene4_frame_slot_index(scene, getattr(self, 'scene4_mode', SCENE4_MODE_COLOR), key)
            targets[key] = scene4_fixed_position(
                key,
                destination,
                slot_index,
                absolute_positions=absolute_positions,
            )
        self._sync_scene4_mapped_targets(scene)
        self.refresh_scene4_coord_editor()
        label = self.scene4_target_label(key)
        destination_label = SCENE4_PLACE_LABELS.get(destination, destination)
        self._set_status_label_text(self.scene4_status, f'{label} 去向: {destination_label}')

    def on_scene4_frame_grid_changed(self, mode, slots):
        mode = mode if mode in SCENE4_MODES else getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
        scene = self.scene4_cfg()
        grid = scene.setdefault('scene4_grid', {})
        slot_field = SCENE4_GRID_SLOT_FIELDS[mode]
        grid[slot_field] = normalized_scene4_frame_slots(slots, scene4_keys_for_mode(mode))
        self.apply_scene4_frame_slots_to_targets(scene, mode)
        self._sync_scene4_mapped_targets(scene, mode)
        self.refresh_scene4_coord_editor()
        labels = scene4_labels_for_mode(mode)
        text = '  '.join(labels.get(key, key) for key in grid[slot_field] if key)
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, f'下方框顺序: {text}')

    def on_scene4_shelf_grid_changed(self, mode, destination, slots):
        mode = mode if mode in SCENE4_MODES else getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
        if destination not in SCENE4_SHELF_LEVELS:
            return
        scene = self.scene4_cfg()
        grid = scene.setdefault('scene4_grid', {})
        slot_field = SCENE4_SHELF_SLOT_FIELDS[mode][destination]
        grid[slot_field] = normalized_scene4_shelf_slots(slots, scene4_keys_for_mode(mode), destination)
        self.apply_scene4_shelf_slots_to_targets(scene, mode, destination)
        self._sync_scene4_mapped_targets(scene, mode)
        self.refresh_scene4_coord_editor()
        labels = scene4_labels_for_mode(mode)
        text = '  '.join(labels.get(key, key) if key else '空' for key in grid[slot_field])
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, f'{SCENE4_PLACE_LABELS.get(destination, destination)}顺序: {text}')

    def on_scene4_frame_moved(self, key, x, y):
        scene = self.scene4_cfg()
        targets = scene.setdefault('place_targets', {})
        place = self.scene4_place_cfg()
        destination = place['targets'].get(key, place['default_destination'])
        absolute_positions = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        scene['scene4_absolute_positions'] = absolute_positions
        if destination == SCENE4_PLACE_FRAME:
            slot_index = self.scene4_frame_slot_index(scene, getattr(self, 'scene4_mode', SCENE4_MODE_COLOR), key)
            targets[key] = scene4_fixed_position(
                key,
                destination,
                slot_index,
                absolute_positions=absolute_positions,
            )
        elif destination in SCENE4_SHELF_LEVELS:
            shelf_slot_index = self.scene4_shelf_slot_index(scene, getattr(self, 'scene4_mode', SCENE4_MODE_COLOR), key, destination)
            targets[key] = scene4_fixed_position(
                key,
                destination,
                shelf_slot_index=shelf_slot_index,
                absolute_positions=absolute_positions,
            )
        self._sync_scene4_mapped_targets(scene)
        self.refresh_scene4_coord_editor()
        if hasattr(self, 'scene4_board'):
            self.scene4_board.set_active_key(key)
        pos = normalized_position(targets.get(key), scene4_default_position(key))
        label = self.scene4_target_label(key)
        self._set_status_label_text(self.scene4_status, f'{label} 固定放置坐标: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}')

    def on_scene4_frame_clicked(self, key):
        if hasattr(self, 'scene4_board'):
            self.scene4_board.set_active_key(key)
        label = self.scene4_target_label(key)
        self._set_status_label_text(self.scene4_status, f'已选择{label}，拖动到目标位置后点击确认位置')

    def confirm_scene4_placement(self, show_status=True):
        scene = self.scene4_cfg()
        mode = getattr(self, 'scene4_mode', SCENE4_MODE_ALL)
        mode = mode if mode in SCENE4_MODES else SCENE4_MODE_ALL
        keys = scene4_keys_for_mode(mode)
        grid = scene.setdefault('scene4_grid', {})
        place = self.scene4_place_cfg()
        if hasattr(self, 'scene4_board'):
            grid[SCENE4_GRID_SLOT_FIELDS[mode]] = normalized_scene4_frame_slots(
                self.scene4_board.frame_slots,
                keys,
            )
            for destination in SCENE4_SHELF_LEVELS:
                grid[SCENE4_SHELF_SLOT_FIELDS[mode][destination]] = normalized_scene4_shelf_slots(
                    self.scene4_board.shelf_slots.get(destination),
                    keys,
                    destination,
                )
            for key in keys:
                destination = self.scene4_board.destinations.get(key, place['default_destination'])
                if destination not in (SCENE4_PLACE_FRAME, *SCENE4_SHELF_LEVELS):
                    destination = SCENE4_PLACE_FRAME
                place['targets'][key] = destination
        self._sync_scene4_mapped_targets(scene, mode)
        self.scene_cfg['current_scene'] = SCENE4_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene4_coord_editor()
        if show_status and hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, '场景4放置位置已确认，点击开启分拣后生效运行')

    def save_scene4(self):
        scene = self.scene4_cfg()
        self._sync_scene4_mapped_targets(scene)
        scene['scene4_pick'] = normalized_scene4_pick(scene.get('scene4_pick'))
        scene['scene4_place'] = normalized_scene4_place(scene.get('scene4_place'))
        scene['scene4_shelf'] = normalized_scene4_shelf(scene.get('scene4_shelf'))
        scene['scene4_absolute_positions'] = normalized_scene4_absolute_positions(scene.get('scene4_absolute_positions'))
        self.scene_cfg['current_scene'] = SCENE4_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        QMessageBox.information(self, '保存成功', '场景4坐标已保存')

    def reset_scene4(self):
        scene = self.scene4_cfg()
        defaults = DEFAULT_SCENE_CONFIG['scenes'][SCENE4_ID]
        scene['length_m'] = defaults['length_m']
        scene['width_m'] = defaults['width_m']
        scene['home_pose'] = dict(DEFAULT_SCENE_HOME_POSES[SCENE4_ID])
        scene['rail'] = dict(DEFAULT_SCENE4_RAIL)
        scene['calibration_pose'] = dict(DEFAULT_SCENE4_CALIBRATION_POSE)
        scene['scene4_pick'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_PICK))
        scene['scene4_place'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_PLACE))
        scene['scene4_shelf'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_SHELF))
        scene['scene4_absolute_positions'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_ABSOLUTE_POSITIONS))
        scene['kinematics'] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE4_KINEMATICS))
        targets = scene.setdefault('place_targets', {})
        for key in WASTE_KEYS:
            targets.pop(key, None)
        for key, pos in SCENE4_DEFAULT_ALL_TARGETS.items():
            targets[key] = list(pos)
        scene['scene4_grid'] = {
            'color_slots': list(SCENE4_COLOR_KEYS),
            'all_slots': list(SCENE4_COLOR_KEYS),
            'color_upper_slots': ['red', '', '', 'green'],
            'color_lower_slots': ['yellow', '', '', 'blue'],
            'all_upper_slots': ['red', '', '', 'green'],
            'all_lower_slots': ['yellow', '', '', 'blue'],
            'color_slot_targets': [list(SCENE4_COLOR_TARGETS[key]) for key in SCENE4_COLOR_KEYS],
        }
        self._sync_scene4_mapped_targets(scene)
        self.scene_cfg['current_scene'] = SCENE4_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene4_coord_editor()
        self.refresh_scene4_pick_editor()
        self.refresh_scene4_rail_editor()
        self.refresh_scene4_absolute_editor()

    def _set_scene4_action_status(self, text):
        if hasattr(self, 'scene4_status'):
            self._set_status_label_text(self.scene4_status, text)

    def setup_scene5_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # ── 左侧：控制区 ──
        left = QWidget()
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        control_box = QGroupBox('场景5双臂流水线控制')
        control_layout = QGridLayout(control_box)
        self.btn_scene5_start_pipeline = QPushButton('开启分拣')
        self.btn_scene5_stop = QPushButton('关闭分拣')
        scene5_control_buttons = (
            self.btn_scene5_start_pipeline,
            self.btn_scene5_stop,
        )
        for index, btn in enumerate(scene5_control_buttons):
            btn.setMinimumHeight(38)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            control_layout.addWidget(btn, 0, index)
        self.configure_play_state_buttons(self.btn_scene5_start_pipeline, self.btn_scene5_stop, running=False)
        layout.addWidget(control_box)

        settings_box = QGroupBox('场景5参数')
        settings_layout = QGridLayout(settings_box)
        settings_layout.setContentsMargins(10, 8, 10, 8)
        settings_layout.setHorizontalSpacing(8)
        settings_layout.setVerticalSpacing(8)
        settings_layout.addWidget(QLabel('传送带速度'), 0, 0)
        self.scene5_speed_buttons = {}
        for col, (label, speed) in enumerate(SCENE5_CONVEYOR_SPEED_PRESETS, start=1):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setMinimumHeight(34)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.scene5_speed_buttons[int(speed)] = btn
            settings_layout.addWidget(btn, 0, col)
        self.btn_scene5_place_offset = QPushButton('放置偏差微调')
        self.btn_scene5_place_offset.setMinimumHeight(34)
        settings_layout.addWidget(self.btn_scene5_place_offset, 1, 0, 1, 4)
        layout.addWidget(settings_box)

        self.scene5_status = QLabel('场景5：双机械臂单传送带；速度为反向负值三档，放置偏差在本场景中微调。')
        self.configure_status_label(self.scene5_status)
        self.scene5_status.setWordWrap(False)
        self.scene5_status.setMaximumHeight(36)
        self.scene5_status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        layout.addWidget(self.scene5_status)
        layout.addStretch()

        _s5_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sandbox_scene5.png')
        _s5_img_label = QLabel()
        _s5_img_label.setAlignment(Qt.AlignCenter)
        _s5_img_label.setStyleSheet('QLabel{background:transparent;border:none;}')
        _s5_img_label.setMaximumSize(360, 280)
        _s5_img_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        _s5_pix = QPixmap(_s5_img_path)
        if not _s5_pix.isNull():
            _s5_img_label.setPixmap(_s5_pix.scaled(348, 268, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        _s5_img_row = QHBoxLayout()
        _s5_img_row.addStretch()
        _s5_img_row.addWidget(_s5_img_label)
        layout.addLayout(_s5_img_row)

        outer.addWidget(left, 2)

        # ── 右侧：回传画面（全高度）──
        right = QWidget()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        self.scene5_arm_a_preview = self._create_scene2_preview_label('等待A机械臂识别画面\n' + SCENE5_ARM_A_IMAGE_TOPIC)
        self.scene5_arm_b_preview = self._create_scene2_preview_label('等待B机械臂识别画面\n' + SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC)
        self.scene5_arm_a_box = QGroupBox('A机械臂识别画面')
        arm_a_layout = QVBoxLayout(self.scene5_arm_a_box)
        arm_a_layout.setContentsMargins(4, 4, 4, 4)
        arm_a_layout.addWidget(self.scene5_arm_a_preview, 1)
        self.scene5_result_box = QGroupBox('B机械臂识别画面')
        result_layout = QVBoxLayout(self.scene5_result_box)
        result_layout.setContentsMargins(4, 4, 4, 4)
        result_layout.addWidget(self.scene5_arm_b_preview, 1)
        right_layout.addWidget(self.scene5_arm_a_box, 1)
        right_layout.addWidget(self.scene5_result_box, 1)

        outer.addWidget(right, 3)

        self.add_scroll_tab(page, BUILTIN_SCENE_NAMES[SCENE5_ID])
        self.btn_scene5_start_pipeline.pressed.connect(self.start_scene5_pipeline)
        self.btn_scene5_stop.pressed.connect(self.stop_scene5_tasks)
        for speed, btn in self.scene5_speed_buttons.items():
            btn.pressed.connect(lambda s=speed: self.apply_scene5_conveyor_speed(s))
        self.btn_scene5_place_offset.pressed.connect(self.show_global_place_offset_dialog)
        self.refresh_scene5_grids()
        self.refresh_scene5_conveyor_editor()
        self.refresh_scene5_b_fixed_pick_editor()

    def scene5_cfg(self):
        scenes = self.scene_cfg.setdefault('scenes', {})
        if SCENE5_ID not in scenes:
            scenes[SCENE5_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE5_ID]))
        self.normalize_scene_cfg(self.scene_cfg)
        return scenes[SCENE5_ID]

    def scene5_grid_cfg(self):
        scene = self.scene5_cfg()
        grid = scene.setdefault('scene5_grid', yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_GRID)))
        if not isinstance(grid, dict):
            grid = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_GRID))
            scene['scene5_grid'] = grid
        return grid

    def scene5_dual_arm_cfg(self):
        scene = self.scene5_cfg()
        dual_arm = scene.setdefault('scene5_dual_arm', yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM)))
        if not isinstance(dual_arm, dict):
            dual_arm = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM))
            scene['scene5_dual_arm'] = dual_arm
        return dual_arm

    def scene5_place_targets(self):
        dual_arm = self.scene5_dual_arm_cfg()
        targets = dual_arm.setdefault(
            'arm_b_place_targets',
            yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'])),
        )
        if not isinstance(targets, dict):
            targets = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets']))
            dual_arm['arm_b_place_targets'] = targets
        dual_arm['arm_b_place_targets'] = _merge_missing_dict(
            targets,
            yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'])),
        )
        return dual_arm['arm_b_place_targets']

    def scene5_conveyor_cfg(self):
        dual_arm = self.scene5_dual_arm_cfg()
        conveyor = dual_arm.setdefault('conveyor', {})
        if not isinstance(conveyor, dict):
            conveyor = {}
            dual_arm['conveyor'] = conveyor
        conveyor = _merge_missing_dict(conveyor, yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['conveyor'])))
        dual_arm['conveyor'] = conveyor
        return conveyor

    def scene5_color_slots(self):
        grid = self.scene5_grid_cfg()
        slots = []
        for key in grid.get('color_slots', list(SCENE5_COLOR_KEYS)):
            if key in SCENE5_COLOR_KEYS and key not in slots:
                slots.append(key)
        for key in SCENE5_COLOR_KEYS:
            if key not in slots:
                slots.append(key)
        grid['color_slots'] = slots[:len(SCENE5_COLOR_KEYS)]
        return grid['color_slots']

    def scene5_waste_slots(self):
        grid = self.scene5_grid_cfg()
        slots = []
        for key in grid.get('waste_slots', list(SCENE5_WASTE_KEYS)):
            if key in WASTE_KEYS and key not in slots:
                slots.append(key)
        for key in SCENE5_WASTE_KEYS:
            if key not in slots:
                slots.append(key)
        grid['waste_slots'] = slots[:len(SCENE5_WASTE_KEYS)]
        return grid['waste_slots']

    def refresh_scene5_grids(self):
        if not hasattr(self, 'scene5_place_map'):
            return
        self._updating_scene5_grid = True
        try:
            self.scene5_color_slots()
            self.scene5_waste_slots()
            self.scene5_place_map.set_scene_targets(self.scene5_place_targets())
        finally:
            self._updating_scene5_grid = False

    def refresh_scene5_conveyor_editor(self):
        conveyor = self.scene5_conveyor_cfg()
        speed = int(conveyor.get('speed', DEFAULT_SCENE5_DUAL_ARM['conveyor']['speed']))
        preset_speeds = [int(value) for _label, value in SCENE5_CONVEYOR_SPEED_PRESETS]
        if speed not in preset_speeds:
            speed = int(DEFAULT_SCENE5_DUAL_ARM['conveyor']['speed'])
            conveyor['speed'] = speed
        if hasattr(self, 'scene5_speed_buttons'):
            for preset_speed, button in self.scene5_speed_buttons.items():
                button.blockSignals(True)
                button.setChecked(int(preset_speed) == speed)
                button.blockSignals(False)

    def current_scene5_conveyor_speed(self):
        return int(self.scene5_conveyor_cfg().get('speed', DEFAULT_SCENE5_DUAL_ARM['conveyor']['speed']))

    def apply_scene5_conveyor_speed(self, speed=None):
        if speed is None:
            speed = self.current_scene5_conveyor_speed()
        speed = int(speed)
        preset_speeds = [int(value) for _label, value in SCENE5_CONVEYOR_SPEED_PRESETS]
        if speed not in preset_speeds:
            speed = int(DEFAULT_SCENE5_DUAL_ARM['conveyor']['speed'])
        self.scene5_conveyor_cfg()['speed'] = int(speed)
        self.save_scene_cfg()
        self.refresh_scene5_conveyor_editor()
        if self.node is None:
            if hasattr(self, 'scene5_status'):
                self._set_status_label_text(self.scene5_status, f'传送带速度已设为 {speed}，ROS 节点未初始化')
            return
        self._run_scene5_action_async(
            '设置传送带速度',
            lambda: self.node.set_scene5_conveyor_speed(speed),
            stop_existing=False,
        )

    def on_scene5_place_targets_changed(self, targets):
        if self._updating_scene5_grid:
            return
        dual_arm = self.scene5_dual_arm_cfg()
        dual_arm['arm_b_place_targets'] = _merge_missing_dict(
            yaml.safe_load(yaml.safe_dump(targets)),
            yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE5_DUAL_ARM['arm_b_place_targets'])),
        )
        self.save_scene_cfg()
        if hasattr(self, 'scene5_status'):
            self._set_status_label_text(self.scene5_status, 'B机械臂固定放置位置已更新')

    def refresh_scene5_b_fixed_pick_editor(self):
        if not hasattr(self, 'sp_b_trigger_cx'):
            return
        fp = self.scene5_dual_arm_cfg().get('arm_b_fixed_pick', {})
        defaults = DEFAULT_SCENE5_DUAL_ARM['arm_b_fixed_pick']
        self.sp_b_trigger_cx.setValue(float(fp.get('trigger_center_x', defaults['trigger_center_x'])))
        self.sp_b_trigger_cy.setValue(float(fp.get('trigger_center_y', defaults['trigger_center_y'])))
        self.sp_b_trigger_tx.setValue(float(fp.get('trigger_tolerance_x', defaults['trigger_tolerance_x'])))
        self.sp_b_trigger_ty.setValue(float(fp.get('trigger_tolerance_y', defaults['trigger_tolerance_y'])))
        self.sp_b_pick_x.setValue(float(fp.get('x', defaults['x'])))
        self.sp_b_pick_y.setValue(float(fp.get('y', defaults['y'])))
        self.sp_b_pick_z.setValue(float(fp.get('z', defaults['z'])))
        self.sp_b_pick_pitch.setValue(float(fp.get('pitch', defaults['pitch'])))
        self.sp_b_pick_roll.setValue(float(fp.get('roll', defaults['roll'])))
        self.sp_b_pre_grab_roll.setValue(float(fp.get('pre_grab_roll', defaults['pre_grab_roll'])))

    def apply_scene5_b_fixed_pick(self):
        params = {
            'trigger_center_x': self.sp_b_trigger_cx.value(),
            'trigger_center_y': self.sp_b_trigger_cy.value(),
            'trigger_tolerance_x': self.sp_b_trigger_tx.value(),
            'trigger_tolerance_y': self.sp_b_trigger_ty.value(),
            'x': self.sp_b_pick_x.value(),
            'y': self.sp_b_pick_y.value(),
            'z': self.sp_b_pick_z.value(),
            'pitch': self.sp_b_pick_pitch.value(),
            'roll': self.sp_b_pick_roll.value(),
            'pre_grab_roll': self.sp_b_pre_grab_roll.value(),
        }
        dual_arm = self.scene5_dual_arm_cfg()
        fp_cfg = dual_arm.setdefault('arm_b_fixed_pick', dict(DEFAULT_SCENE5_DUAL_ARM['arm_b_fixed_pick']))
        fp_cfg.update(params)
        self.save_scene_cfg()
        if self.node is None:
            if hasattr(self, 'scene5_status'):
                self._set_status_label_text(self.scene5_status, 'B夹取参数已保存（ROS节点未初始化）')
            return
        self._run_scene5_action_async(
            '发送B夹取参数',
            lambda: self.node.set_scene5_b_fixed_pick(params),
            stop_existing=False,
        )

    def on_scene5_waste_grid_changed(self, slots):
        if self._updating_scene5_grid:
            return
        grid = self.scene5_grid_cfg()
        grid['waste_slots'] = list(slots)
        if hasattr(self, 'scene5_status'):
            labels = '  '.join(WASTE_LABELS.get(key, key) for key in grid['waste_slots'])
            self._set_status_label_text(self.scene5_status, f'B机械臂放置槽位顺序: {labels}')

    def save_scene5(self):
        scene = self.scene5_cfg()
        scene['mode'] = 'dual_arm_single_conveyor'
        scene['calibration_pose'] = normalized_calibration_pose(
            scene.get('calibration_pose'),
            DEFAULT_SCENE5_CALIBRATION_POSE,
        )
        self.scene5_grid_cfg()
        self.scene5_conveyor_cfg()['speed'] = self.current_scene5_conveyor_speed()
        self.scene5_place_targets()
        self.scene_cfg['current_scene'] = SCENE5_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        QMessageBox.information(self, '保存成功', '场景5双臂流水线配置已保存')

    def reset_scene5(self):
        self.scene_cfg.setdefault('scenes', {})[SCENE5_ID] = yaml.safe_load(
            yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE5_ID])
        )
        self.scene_cfg['current_scene'] = SCENE5_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene5_grids()
        self.refresh_scene5_conveyor_editor()
        self.refresh_scene5_b_fixed_pick_editor()

    def _set_scene5_action_status(self, text):
        if hasattr(self, 'scene5_status'):
            self._set_status_label_text(self.scene5_status, text)

    def _compact_scene5_status(self, text, limit=64):
        return self._compact_status_text(text, limit)

    def _activate_scene_for_backend(self, scene_id):
        if scene_id == DEFAULT_SCENE_ID:
            scenes = self.scene_cfg.setdefault('scenes', {})
            if DEFAULT_SCENE_ID not in scenes:
                scenes[DEFAULT_SCENE_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]))
        elif scene_id == SCENE2_ID:
            self.apply_scene2_slots_to_targets()
        elif scene_id == SCENE3_ID:
            self.scene3_cfg()
        elif scene_id == SCENE4_ID:
            scene = self.scene4_cfg()
            self._sync_scene4_mapped_targets(scene)
            scene['scene4_pick'] = normalized_scene4_pick(scene.get('scene4_pick'))
            scene['scene4_place'] = normalized_scene4_place(scene.get('scene4_place'))
            scene['scene4_shelf'] = normalized_scene4_shelf(scene.get('scene4_shelf'))
        elif scene_id == SCENE5_ID:
            scene = self.scene5_cfg()
            scene['mode'] = 'dual_arm_single_conveyor'
            self.scene5_color_slots()
            self.scene5_waste_slots()
            self.scene5_place_targets()
            self.scene5_conveyor_cfg()['speed'] = self.current_scene5_conveyor_speed()
        self.scene_cfg['current_scene'] = scene_id
        self.save_scene_cfg()
        if hasattr(self, 'cb_scene'):
            idx = self.cb_scene.findData(scene_id)
            if idx >= 0 and self.cb_scene.currentIndex() != idx:
                self.cb_scene.blockSignals(True)
                self.cb_scene.setCurrentIndex(idx)
                self.cb_scene.blockSignals(False)
        self.on_scene_changed()

    def _set_scene1_action_status(self, text):
        if hasattr(self, 'scene1_status'):
            self._set_status_label_text(self.scene1_status, text)

    def _set_scene3_result_mode(self, mode):
        self.scene3_active_result = 'waste' if mode == 'waste' else 'color'
        if hasattr(self, 'scene3_result_box'):
            if self.scene3_active_result == 'waste':
                self.scene3_result_box.setTitle('垃圾分类图像')
            else:
                self.scene3_result_box.setTitle('色块夹取图像')
        self._refresh_scene3_result_preview()

    def _set_scene4_result_mode(self, mode):
        self.scene4_active_result = 'waste' if mode == 'waste' else 'result'
        if hasattr(self, 'scene4_result_box'):
            if self.scene4_active_result == 'waste':
                self.scene4_result_box.setTitle('垃圾分类图像')
            else:
                self.scene4_result_box.setTitle('颜色夹取图像')
        self._refresh_scene4_result_preview()

    def _set_scene5_result_mode(self, mode):
        self.scene5_active_result = 'arm_b'
        if hasattr(self, 'scene5_result_box'):
            self.scene5_result_box.setTitle('B机械臂识别画面')
        self._refresh_scene5_arm_b_preview()

    def _stop_calibration_for_play(self):
        if self.node is None:
            return
        if self.scene_cfg.get('current_scene') == SCENE5_ID:
            try:
                self.node.stop_scene5_tasks()
            except Exception:
                pass
        try:
            self.node.enable_calibration(False)
        except Exception:
            pass
        try:
            self.node.exit_calibration()
        except Exception:
            pass

    def start_scene4_waste(self, waste_key=None):
        self.set_scene4_mode(SCENE4_MODE_WASTE)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE4_ID)
        self._set_scene4_result_mode('waste')
        label = '全部垃圾' if waste_key is None else WASTE_LABELS.get(waste_key, waste_key)
        self.scene4_status_signal.emit(f'正在开启场景4垃圾夹取: {label}')
        threading.Thread(target=self._scene4_waste_worker, args=(waste_key,), daemon=True).start()

    def _scene4_waste_worker(self, waste_key=None):
        ok, msg = self.node.start_waste_classification(waste_key)
        self.scene4_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def start_scene4_color(self, color_key=None):
        self.set_scene4_mode(SCENE4_MODE_COLOR)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        zone = self.current_scene4_pick_zone()
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE4_ID)
        self.set_scene4_pick_zone(zone, save=True)
        self._set_scene4_result_mode('result')
        color_label = '全部色块' if color_key is None else SCENE2_COLOR_LABELS.get(color_key, color_key)
        self.scene4_status_signal.emit(f'正在开启场景4下层夹取并按去向放置: {color_label}')
        threading.Thread(target=self._scene4_color_worker, args=(color_key,), daemon=True).start()

    def _scene4_color_worker(self, color_key=None):
        ok, msg = self.node.start_color_sorting(color_key, stop_all=True)
        self.scene4_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def start_scene4_all_sorting(self):
        self.set_scene_play_state(SCENE4_ID, True)
        self.set_scene4_mode(SCENE4_MODE_COLOR)
        self.confirm_scene4_placement(show_status=False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE4_ID)
        self.set_scene4_pick_zone(SCENE4_PICK_ZONE_LOWER, save=True)
        self._set_scene4_result_mode('result')
        self.scene4_status_signal.emit('正在开启场景4色块夹取...')
        threading.Thread(target=self._scene4_all_sort_worker, daemon=True).start()

    def _scene4_all_sort_worker(self):
        ok, msg = self.node.start_color_sorting(stop_all=True)
        self.scene4_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def stop_scene4_tasks(self):
        self.set_scene_play_state(SCENE4_ID, False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._activate_scene_for_backend(SCENE4_ID)
        self.scene4_status_signal.emit('正在停止场景4玩法...')
        threading.Thread(target=self._scene4_stop_worker, daemon=True).start()

    def _scene4_stop_worker(self):
        ok, msg = self.node.stop_scene3_tasks()
        self.scene4_status_signal.emit(('已停止: ' if ok else '停止失败: ') + msg)

    def _run_scene5_action_async(self, label, fn, stop_existing=True):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        if stop_existing:
            self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE5_ID)
        self._set_scene5_result_mode('waste')
        self.scene5_status_signal.emit(f'正在执行: {label}')
        threading.Thread(target=self._scene5_action_worker, args=(label, fn), daemon=True).start()

    def _scene5_action_worker(self, label, fn):
        ok, msg = fn()
        if ok:
            short = {
                '启动场景5': '场景5已启动',
                '传送带启动': '传送带已启动',
                '传送带停止': '传送带已停止',
                '设置传送带速度': '传送带速度已设置',
                '发送B夹取参数': 'B夹取参数已发送',
            }.get(label, msg)
            self.scene5_status_signal.emit(short)
        else:
            self.scene5_status_signal.emit(f'{label}失败: {msg}')

    def start_scene5_pipeline(self):
        self.set_scene_play_state(SCENE5_ID, True)
        slots = self.scene5_waste_slots()
        place_targets = self.scene5_place_targets()
        speed = self.current_scene5_conveyor_speed()

        def start_with_speed():
            ok, msg = self.node.set_scene5_conveyor_speed(speed)
            if not ok:
                return ok, msg
            ok, msg = self.node.start_scene5_pipeline(slots, place_targets)
            return ok, f'{msg} | conveyor speed={speed}'

        self._run_scene5_action_async('启动场景5', start_with_speed)

    def run_scene5_one_cycle(self):
        slots = self.scene5_waste_slots()
        place_targets = self.scene5_place_targets()
        self._run_scene5_action_async('单循环', lambda: self.node.run_scene5_one_cycle(slots, place_targets))

    def scene5_arm_a_home(self):
        self._run_scene5_action_async('A回零', self.node.scene5_arm_a_home, stop_existing=False)

    def scene5_arm_a_start(self):
        self._run_scene5_action_async('A开始识别夹取', self.node.scene5_arm_a_start, stop_existing=False)

    def scene5_arm_a_stop(self):
        self._run_scene5_action_async('A停止识别夹取', self.node.scene5_arm_a_stop, stop_existing=False)

    def scene5_arm_a_load_once(self):
        self._run_scene5_action_async('A固定放置一次', self.node.scene5_arm_a_load_once, stop_existing=False)

    def scene5_arm_b_enter(self):
        slots = self.scene5_waste_slots()
        place_targets = self.scene5_place_targets()
        self._run_scene5_action_async(
            'B进入准备',
            lambda: self.node.scene5_arm_b_enter(slots, place_targets),
            stop_existing=False,
        )

    def scene5_arm_b_start(self):
        slots = self.scene5_waste_slots()
        place_targets = self.scene5_place_targets()
        self._run_scene5_action_async(
            'B开始识别夹取',
            lambda: self.node.scene5_arm_b_start(slots, place_targets),
            stop_existing=False,
        )

    def scene5_arm_b_stop(self):
        self._run_scene5_action_async('B停止识别夹取', self.node.scene5_arm_b_stop, stop_existing=False)

    def start_scene5_conveyor(self):
        speed = self.current_scene5_conveyor_speed()

        def start_conveyor_with_speed():
            ok, msg = self.node.set_scene5_conveyor_speed(speed)
            if not ok:
                return ok, msg
            ok, msg = self.node.start_scene5_conveyor()
            return ok, f'{msg} | speed={speed}'

        self._run_scene5_action_async('传送带启动', start_conveyor_with_speed, stop_existing=False)

    def stop_scene5_conveyor(self):
        self._run_scene5_action_async('传送带停止', self.node.stop_scene5_conveyor, stop_existing=False)

    def stop_scene5_tasks(self):
        self.set_scene_play_state(SCENE5_ID, False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._activate_scene_for_backend(SCENE5_ID)
        self.scene5_status_signal.emit('正在停止场景5流水线...')
        threading.Thread(target=self._scene5_stop_worker, daemon=True).start()

    def _scene5_stop_worker(self):
        ok, msg = self.node.stop_scene5_tasks()
        self.scene5_status_signal.emit(('已停止: ' if ok else '停止失败: ') + msg)

    def start_scene3_color(self, color_key=None):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE3_ID)
        self._set_scene3_result_mode('color')
        label = '全部色块' if color_key is None else SCENE2_COLOR_LABELS.get(color_key, color_key)
        self.scene3_status_signal.emit(f'正在开启色块夹取: {label}')
        threading.Thread(target=self._scene3_color_worker, args=(color_key,), daemon=True).start()

    def _scene3_color_worker(self, color_key):
        ok, msg = self.node.start_color_sorting(color_key)
        self.scene3_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def start_scene3_all_sorting(self):
        self.set_scene_play_state(SCENE3_ID, True)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE3_ID)
        self._set_scene3_result_mode('color')
        self.scene3_status_signal.emit('正在开启色块和垃圾一起夹取...')
        threading.Thread(target=self._scene3_all_sort_worker, daemon=True).start()

    def _scene3_all_sort_worker(self):
        ok, msg = self.node.start_color_and_waste_sorting()
        self.scene3_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def start_scene3_waste(self, waste_key=None):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(SCENE3_ID)
        self._set_scene3_result_mode('waste')
        label = '全部垃圾' if waste_key is None else WASTE_LABELS.get(waste_key, waste_key)
        self.scene3_status_signal.emit(f'正在开启垃圾分类: {label}')
        threading.Thread(target=self._scene3_waste_worker, args=(waste_key,), daemon=True).start()

    def _scene3_waste_worker(self, waste_key):
        ok, msg = self.node.start_waste_classification(waste_key)
        self.scene3_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def stop_scene3_tasks(self):
        self.set_scene_play_state(SCENE3_ID, False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self.scene3_status_signal.emit('正在停止场景3任务...')
        threading.Thread(target=self._scene3_stop_worker, daemon=True).start()

    def _scene3_stop_worker(self):
        ok, msg = self.node.stop_scene3_tasks()
        self.scene3_status_signal.emit(('已停止: ' if ok else '停止失败: ') + msg)

    def scene2_cfg(self):
        scenes = self.scene_cfg.setdefault('scenes', {})
        if SCENE2_ID not in scenes:
            scenes[SCENE2_ID] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][SCENE2_ID]))
        self.normalize_scene_cfg(self.scene_cfg)
        return scenes[SCENE2_ID]

    def scene2_slots(self):
        scene = self.scene2_cfg()
        grid = scene.setdefault('color_grid', {})
        slots = grid.get('slots', list(SCENE2_COLOR_KEYS))
        clean = []
        for key in slots:
            if key in SCENE2_COLOR_KEYS and key not in clean:
                clean.append(key)
        for key in SCENE2_COLOR_KEYS:
            if key not in clean:
                clean.append(key)
        grid['slots'] = clean[:4]
        return grid['slots']

    def scene2_slot_targets(self):
        scene = self.scene2_cfg()
        grid = scene.setdefault('color_grid', {})
        targets = grid.get('slot_targets')
        if not isinstance(targets, list) or len(targets) != 4:
            targets = [list(DEFAULT_SCENE2_COLOR_TARGETS[key]) for key in SCENE2_SLOT_TARGET_KEYS]
        fixed = []
        for idx, default_key in enumerate(SCENE2_SLOT_TARGET_KEYS):
            raw = targets[idx] if idx < len(targets) else DEFAULT_SCENE2_COLOR_TARGETS[default_key]
            if not isinstance(raw, list) or len(raw) != 3:
                raw = DEFAULT_SCENE2_COLOR_TARGETS[default_key]
            fixed.append([float(raw[0]), float(raw[1]), float(raw[2])])
        grid['slot_targets'] = fixed
        return fixed

    def apply_scene2_slots_to_targets(self):
        scene = self.scene2_cfg()
        slots = self.scene2_slots()
        slot_targets = self.scene2_slot_targets()
        targets = scene.setdefault('place_targets', {})
        for idx, color_key in enumerate(slots):
            targets[color_key] = list(slot_targets[idx])

    def scene2_mode_keys(self):
        if self.scene2_mode == 'tag':
            return list(TAG_KEYS)
        if self.scene2_mode == 'waste':
            return list(WASTE_KEYS)
        return list(SCENE2_COLOR_KEYS)

    def scene2_mode_labels(self):
        if self.scene2_mode == 'tag':
            return TAG_LABELS
        if self.scene2_mode == 'waste':
            return WASTE_LABELS
        return SCENE2_COLOR_LABELS

    def scene2_mode_colors(self):
        if self.scene2_mode == 'tag':
            return TAG_QCOLORS
        if self.scene2_mode == 'waste':
            return WASTE_QCOLORS
        return SCENE2_COLOR_QCOLORS

    def scene2_default_position(self, key):
        if key in DEFAULT_SCENE2_COLOR_TARGETS:
            return list(DEFAULT_SCENE2_COLOR_TARGETS[key])
        return list(DEFAULT_SCENE_PLACE_TARGETS.get(key, [0.0, 0.0, 0.015]))

    def set_scene2_mode(self, mode):
        mode = mode if mode in ('color', 'tag', 'waste') else 'color'
        self.scene2_mode = mode
        keys = self.scene2_mode_keys()
        labels = self.scene2_mode_labels()
        colors = self.scene2_mode_colors()
        if hasattr(self, 'scene2_grid'):
            self.scene2_grid.set_items(keys, labels, colors, draggable=False)
            if mode == 'color':
                self.scene2_grid.set_slots(self.scene2_slots())
        if hasattr(self, 'scene2_coord_box'):
            titles = {'color': '颜色放置坐标(m)', 'tag': '标签放置坐标(m)', 'waste': '垃圾分类坐标(m)'}
            self.scene2_coord_box.setTitle(titles[mode])
        if hasattr(self, 'scene2_coord_kind_label'):
            labels0 = {'color': '颜色', 'tag': '标签', 'waste': '类别'}
            self.scene2_coord_kind_label.setText(labels0[mode])
        hints = {
            'color': '场景2使用固定放置位置；放置偏差在本场景中微调。',
            'tag': '场景2使用固定放置位置；放置偏差在本场景中微调。',
            'waste': '场景2使用固定放置位置；放置偏差在本场景中微调。',
        }
        if hasattr(self, 'scene2_action_status'):
            self._set_status_label_text(self.scene2_action_status, hints[mode])
        self.refresh_scene2_grid()

    def refresh_scene2_grid(self):
        if hasattr(self, 'scene2_grid'):
            if getattr(self, 'scene2_mode', 'color') == 'color':
                self.scene2_grid.set_items(SCENE2_COLOR_KEYS, SCENE2_COLOR_LABELS, SCENE2_COLOR_QCOLORS, draggable=False)
                self.scene2_grid.set_slots(self.scene2_slots())
            else:
                self.scene2_grid.set_items(
                    self.scene2_mode_keys(),
                    self.scene2_mode_labels(),
                    self.scene2_mode_colors(),
                    draggable=False,
                )
        if hasattr(self, 'scene2_status'):
            labels = self.scene2_mode_labels()
            if getattr(self, 'scene2_mode', 'color') == 'color':
                text = '  '.join(labels.get(k, k) for k in self.scene2_slots())
                self._set_status_label_text(self.scene2_status, f'当前颜色顺序: {text}')
            else:
                text = '  '.join(labels.get(k, k) for k in self.scene2_mode_keys())
                title = '标签框' if self.scene2_mode == 'tag' else '垃圾分类框'
                self._set_status_label_text(self.scene2_status, f'当前{title}: {text}')
        self.refresh_scene2_coord_editor()

    def refresh_scene2_coord_editor(self):
        if not hasattr(self, 'scene2_coord_rows'):
            return
        scene = self.scene2_cfg()
        targets = scene.setdefault('place_targets', {})
        keys = self.scene2_mode_keys()
        labels = self.scene2_mode_labels()
        colors = self.scene2_mode_colors()
        self._updating_scene2_coords = True
        try:
            for idx, row in enumerate(self.scene2_coord_rows):
                active = idx < len(keys)
                key = keys[idx] if active else None
                row['key'] = key
                row['label'].setVisible(active)
                for spin in row['spins']:
                    spin.setVisible(active)
                if not active:
                    continue
                row['label'].setText(labels.get(key, key))
                row['label'].setStyleSheet(
                    f'background:{colors.get(key, QColor(180, 180, 180)).name()};'
                    'border:1px solid #5A5D6E;border-radius:4px;'
                    'font-weight:bold;padding:6px;color:#111827;'
                )
                pos = targets.get(key, self.scene2_default_position(key))
                if not isinstance(pos, list) or len(pos) != 3:
                    pos = self.scene2_default_position(key)
                for spin, value in zip(row['spins'], pos):
                    spin.blockSignals(True)
                    spin.setValue(float(value))
                    spin.blockSignals(False)
        finally:
            self._updating_scene2_coords = False

    def on_scene2_coord_row_changed(self, row_index, axis, value):
        if not hasattr(self, 'scene2_coord_rows') or row_index >= len(self.scene2_coord_rows):
            return
        key = self.scene2_coord_rows[row_index].get('key')
        if key is None:
            return
        self.on_scene2_coord_changed(key, axis, value)

    def on_scene2_coord_changed(self, target_key, axis, value):
        if self._updating_scene2_coords:
            return
        scene = self.scene2_cfg()
        targets = scene.setdefault('place_targets', {})
        pos = list(targets.get(target_key, self.scene2_default_position(target_key)))
        if len(pos) != 3:
            pos = self.scene2_default_position(target_key)
        pos[axis] = float(value)
        targets[target_key] = pos
        slots = self.scene2_slots()
        if target_key in slots:
            grid = scene.setdefault('color_grid', {})
            slot_targets = self.scene2_slot_targets()
            slot_targets[slots.index(target_key)] = list(pos)
            grid['slot_targets'] = slot_targets
        labels = self.scene2_mode_labels()
        label = labels.get(target_key, target_key)
        if hasattr(self, 'scene2_action_status'):
            self._set_status_label_text(self.scene2_action_status, f'{label} 坐标: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}')

    def _set_scene2_action_status(self, text):
        if hasattr(self, 'scene2_action_status'):
            self._set_status_label_text(self.scene2_action_status, text)

    def _on_scene2_result_image_msg(self, msg):
        self.scene2_result_image_signal.emit(msg)

    def _on_scene2_rgb_image_msg(self, msg):
        self.scene2_rgb_image_signal.emit(msg)

    def _set_scene2_result_mode(self, mode):
        self.scene2_active_result = 'waste' if mode == 'waste' else 'result'
        if hasattr(self, 'scene2_result_box'):
            if self.scene2_active_result == 'waste':
                self.scene2_result_box.setTitle('垃圾分类图像')
            else:
                self.scene2_result_box.setTitle('颜色夹取图像')
        self._refresh_scene2_preview_label(self.scene2_active_result)

    def _preview_throttle(self, key, fps=15):
        now = time.monotonic()
        if now - self._preview_last_update.get(key, 0) < 1.0 / fps:
            return False
        self._preview_last_update[key] = now
        return True

    def _update_scene1_preview(self, image_type, msg):
        source = None
        if image_type == 'processed' and isinstance(msg, tuple) and len(msg) == 2:
            source, msg = msg
        if image_type == 'processed':
            now = time.monotonic()
            if source == 'waste':
                self.scene1_processed_hold_until = now + 1.5
            elif source == 'color' and now < getattr(self, 'scene1_processed_hold_until', 0.0):
                return
        throttle_key = f's1_{image_type}_{source or "default"}'
        if not self._preview_throttle(throttle_key):
            return
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        self.scene1_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        if image_type == 'rgb':
            self._refresh_scene1_rgb_preview()
        else:
            self._refresh_scene1_processed_preview()

    def _refresh_scene1_rgb_preview(self):
        if not hasattr(self, 'scene1_rgb_preview'):
            return
        pixmap = self.scene1_preview_pixmaps.get('rgb')
        if pixmap is None or pixmap.isNull():
            self.scene1_rgb_preview.clear()
            self.scene1_rgb_preview.setText('等待原始图像\n' + SCENE2_RGB_IMAGE_TOPIC)
            return
        scaled = pixmap.scaled(self.scene1_rgb_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene1_rgb_preview.setPixmap(scaled)

    def _refresh_scene1_processed_preview(self):
        if not hasattr(self, 'scene1_processed_preview'):
            return
        pixmap = self.scene1_preview_pixmaps.get('processed')
        if pixmap is None or pixmap.isNull():
            self.scene1_processed_preview.clear()
            self.scene1_processed_preview.setText('等待处理图像\n' + SCENE2_RESULT_IMAGE_TOPIC + '\n' + SCENE3_WASTE_IMAGE_TOPIC)
            return
        scaled = pixmap.scaled(self.scene1_processed_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene1_processed_preview.setPixmap(scaled)

    def _update_scene2_preview(self, image_type, msg):
        if not self._preview_throttle(f's2_{image_type}'):
            return
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        self.scene2_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        if image_type == 'rgb':
            self._refresh_scene2_preview_label('rgb')
        elif image_type == self.scene2_active_result:
            self._refresh_scene2_preview_label(image_type)

    def _update_scene3_preview(self, image_type, msg):
        if not self._preview_throttle(f's3_{image_type}'):
            return
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        self.scene3_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        if image_type == 'rgb':
            self._refresh_scene3_rgb_preview()
        elif image_type == self.scene3_active_result:
            self._refresh_scene3_result_preview()

    def _update_scene4_preview(self, image_type, msg):
        if not self._preview_throttle(f's4_{image_type}'):
            return
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        self.scene4_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        if image_type == 'rgb':
            self._refresh_scene4_rgb_preview()
        elif image_type == self.scene4_active_result:
            self._refresh_scene4_result_preview()

    def _update_scene5_preview(self, image_type, msg):
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        self.scene5_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        if image_type in ('arm_a', 'rgb'):
            self._refresh_scene5_arm_a_preview()
        elif image_type in ('arm_b', 'waste') or image_type == self.scene5_active_result:
            self._refresh_scene5_arm_b_preview()

    def _update_calib_preview(self, image_type, msg):
        if not self._preview_throttle(f'calib_{image_type}'):
            return
        qimage = self._image_msg_to_qimage(msg)
        if qimage is None:
            return
        if image_type == 'result' and hasattr(msg, 'width') and msg.width > 0:
            self._calib_result_img_size = (int(msg.width), int(msg.height))
        elif image_type == 'depth' and hasattr(msg, 'width') and msg.width > 0:
            self._calib_depth_img_size = (int(msg.width), int(msg.height))
        self.calib_preview_pixmaps[image_type] = QPixmap.fromImage(qimage)
        self._refresh_calib_preview(image_type)

    def _refresh_calib_preview(self, image_type):
        pixmap = self.calib_preview_pixmaps.get(image_type)
        label_map = {
            'result': getattr(self, 'calib_result_preview', None),
            'depth':  getattr(self, 'calib_depth_preview', None),
            'rgb':    getattr(self, 'calib_rgb_preview', None),
        }
        label = label_map.get(image_type)
        if label is None or pixmap is None:
            return
        label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _image_msg_to_qimage(self, msg):
        if isinstance(msg, CompressedImage) or (hasattr(msg, 'format') and not hasattr(msg, 'width')):
            return self._compressed_image_msg_to_qimage(msg)
        try:
            width = int(msg.width)
            height = int(msg.height)
            step = int(msg.step)
            data = bytes(msg.data)
            encoding = str(msg.encoding).lower()
            if width <= 0 or height <= 0:
                return None
            if encoding in ('rgb8', 'bgr8'):
                image = QImage(data, width, height, step, QImage.Format_RGB888)
                if encoding == 'bgr8':
                    image = image.rgbSwapped()
                return image.copy()
            if encoding in ('rgba8', 'bgra8'):
                image = QImage(data, width, height, step, QImage.Format_RGBA8888)
                if encoding == 'bgra8':
                    image = image.rgbSwapped()
                return image.copy()
            if encoding in ('mono8', '8uc1'):
                return QImage(data, width, height, step, QImage.Format_Grayscale8).copy()
        except Exception:
            return None
        return None

    def _compressed_image_msg_to_qimage(self, msg):
        try:
            image = QImage()
            if not image.loadFromData(bytes(msg.data)):
                return None
            return image.convertToFormat(QImage.Format_RGB888).copy()
        except Exception:
            return None

    def _refresh_scene2_preview_label(self, image_type):
        label = self.scene2_rgb_preview if image_type == 'rgb' else self.scene2_result_preview
        pixmap = self.scene2_preview_pixmaps.get(image_type)
        if pixmap is None or pixmap.isNull():
            if image_type == 'waste':
                label.clear()
                label.setText('等待垃圾分类图像\n' + SCENE3_WASTE_IMAGE_TOPIC)
            elif image_type == 'result':
                label.clear()
                label.setText('等待颜色夹取图像\n' + SCENE2_RESULT_IMAGE_TOPIC)
            return
        scaled = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(scaled)

    def _refresh_scene3_rgb_preview(self):
        if not hasattr(self, 'scene3_rgb_preview'):
            return
        pixmap = self.scene3_preview_pixmaps.get('rgb')
        if pixmap is None or pixmap.isNull():
            return
        scaled = pixmap.scaled(self.scene3_rgb_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene3_rgb_preview.setPixmap(scaled)

    def _refresh_scene3_result_preview(self):
        if not hasattr(self, 'scene3_result_preview'):
            return
        pixmap = self.scene3_preview_pixmaps.get(self.scene3_active_result)
        if pixmap is None or pixmap.isNull():
            topic = SCENE3_WASTE_IMAGE_TOPIC if self.scene3_active_result == 'waste' else SCENE3_COLOR_IMAGE_TOPIC
            title = '垃圾分类图像' if self.scene3_active_result == 'waste' else '色块夹取图像'
            self.scene3_result_preview.clear()
            self.scene3_result_preview.setText(f'等待{title}\n{topic}')
            return
        scaled = pixmap.scaled(self.scene3_result_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene3_result_preview.setPixmap(scaled)

    def _refresh_scene4_rgb_preview(self):
        if not hasattr(self, 'scene4_rgb_preview'):
            return
        pixmap = self.scene4_preview_pixmaps.get('rgb')
        if pixmap is None or pixmap.isNull():
            return
        scaled = pixmap.scaled(self.scene4_rgb_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene4_rgb_preview.setPixmap(scaled)

    def _refresh_scene4_result_preview(self):
        if not hasattr(self, 'scene4_result_preview'):
            return
        pixmap = self.scene4_preview_pixmaps.get(self.scene4_active_result)
        if pixmap is None or pixmap.isNull():
            topic = SCENE3_WASTE_IMAGE_TOPIC if self.scene4_active_result == 'waste' else SCENE2_RESULT_IMAGE_TOPIC
            title = '垃圾分类图像' if self.scene4_active_result == 'waste' else '颜色夹取图像'
            self.scene4_result_preview.clear()
            self.scene4_result_preview.setText(f'等待{title}\n{topic}')
            return
        scaled = pixmap.scaled(self.scene4_result_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene4_result_preview.setPixmap(scaled)

    def _refresh_scene5_arm_a_preview(self):
        if not hasattr(self, 'scene5_arm_a_preview'):
            return
        pixmap = self.scene5_preview_pixmaps.get('arm_a') or self.scene5_preview_pixmaps.get('rgb')
        if pixmap is None or pixmap.isNull():
            self.scene5_arm_a_preview.clear()
            self.scene5_arm_a_preview.setText(f'等待A机械臂识别画面\n{SCENE5_ARM_A_IMAGE_TOPIC}')
            return
        scaled = pixmap.scaled(self.scene5_arm_a_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene5_arm_a_preview.setPixmap(scaled)

    def _refresh_scene5_arm_b_preview(self):
        if not hasattr(self, 'scene5_arm_b_preview'):
            return
        pixmap = self.scene5_preview_pixmaps.get('arm_b') or self.scene5_preview_pixmaps.get('waste')
        if pixmap is None or pixmap.isNull():
            self.scene5_arm_b_preview.clear()
            self.scene5_arm_b_preview.setText(f'等待B机械臂识别画面\n{SCENE5_ARM_B_COMPRESSED_IMAGE_TOPIC}')
            return
        scaled = pixmap.scaled(self.scene5_arm_b_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene5_arm_b_preview.setPixmap(scaled)

    def _refresh_scene5_rgb_preview(self):
        self._refresh_scene5_arm_a_preview()

    def _refresh_scene5_result_preview(self):
        self._refresh_scene5_arm_b_preview()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'scene1_rgb_preview'):
            self._refresh_scene1_rgb_preview()
        if hasattr(self, 'scene1_processed_preview'):
            self._refresh_scene1_processed_preview()
        if hasattr(self, 'scene2_result_preview'):
            self._refresh_scene2_preview_label(getattr(self, 'scene2_active_result', 'result'))
        if hasattr(self, 'scene2_rgb_preview'):
            self._refresh_scene2_preview_label('rgb')
        if hasattr(self, 'scene3_rgb_preview'):
            self._refresh_scene3_rgb_preview()
        if hasattr(self, 'scene3_result_preview'):
            self._refresh_scene3_result_preview()
        if hasattr(self, 'scene4_rgb_preview'):
            self._refresh_scene4_rgb_preview()
        if hasattr(self, 'scene4_result_preview'):
            self._refresh_scene4_result_preview()
        if hasattr(self, 'scene5_arm_a_preview'):
            self._refresh_scene5_arm_a_preview()
        if hasattr(self, 'scene5_arm_b_preview'):
            self._refresh_scene5_arm_b_preview()

    def start_scene1_all_sorting(self):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self.set_scene_play_state(DEFAULT_SCENE_ID, True)
        self._stop_calibration_for_play()
        self._activate_scene_for_backend(DEFAULT_SCENE_ID)
        self.scene1_status_signal.emit('正在开启基础分拣：色块和垃圾一起夹取...')
        threading.Thread(target=self._scene1_all_sort_worker, daemon=True).start()

    def _scene1_all_sort_worker(self):
        ok, msg = self.node.start_color_and_waste_sorting()
        self.scene1_status_signal.emit(msg if ok else f'启动失败: {msg}')

    def stop_scene1_tasks(self):
        self.set_scene_play_state(DEFAULT_SCENE_ID, False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._activate_scene_for_backend(DEFAULT_SCENE_ID)
        self.scene1_status_signal.emit('正在停止基础分拣...')
        threading.Thread(target=self._scene1_stop_worker, daemon=True).start()

    def _scene1_stop_worker(self):
        ok, msg = self.node.stop_scene3_tasks()
        self.scene1_status_signal.emit(('已停止: ' if ok else '停止失败: ') + msg)

    def start_scene2_grid_target(self, target_key):
        mode = getattr(self, 'scene2_mode', 'color')
        if mode == 'tag':
            self.start_scene2_tag_sorting(target_key)
        elif mode == 'waste':
            self.start_scene2_waste(target_key)
        else:
            self.start_scene2_color_sorting(target_key)

    def start_scene2_color_sorting(self, color_key=None):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self.set_scene2_mode('color')
        self._activate_scene_for_backend(SCENE2_ID)
        self._set_scene2_result_mode('result')
        label = '全部颜色' if color_key is None else SCENE2_COLOR_LABELS.get(color_key, color_key)
        self.scene2_status_signal.emit(f'正在开启: {label}')
        threading.Thread(target=self._scene2_color_sort_worker, args=(color_key,), daemon=True).start()

    def _scene2_color_sort_worker(self, color_key):
        ok, msg = self.node.start_color_sorting(color_key)
        if ok:
            self.scene2_status_signal.emit(msg)
        else:
            self.scene2_status_signal.emit(f'启动失败: {msg}')

    def start_scene2_all_sorting(self):
        self.set_scene_play_state(SCENE2_ID, True)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self.set_scene2_mode('color')
        self._activate_scene_for_backend(SCENE2_ID)
        self._set_scene2_result_mode('result')
        self.scene2_status_signal.emit('正在开启色块和垃圾一起夹取...')
        threading.Thread(target=self._scene2_all_sort_worker, daemon=True).start()

    def _scene2_all_sort_worker(self):
        ok, msg = self.node.start_color_and_waste_sorting()
        if ok:
            self.scene2_status_signal.emit(msg)
        else:
            self.scene2_status_signal.emit(f'启动失败: {msg}')

    def start_scene2_tag_sorting(self, tag_key=None):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self.set_scene2_mode('tag')
        self._activate_scene_for_backend(SCENE2_ID)
        self._set_scene2_result_mode('result')
        label = '全部标签' if tag_key is None else TAG_LABELS.get(tag_key, tag_key)
        self.scene2_status_signal.emit(f'正在开启标签夹取: {label}')
        threading.Thread(target=self._scene2_tag_sort_worker, args=(tag_key,), daemon=True).start()

    def _scene2_tag_sort_worker(self, tag_key):
        ok, msg = self.node.start_tag_sorting(tag_key)
        if ok:
            self.scene2_status_signal.emit(msg)
        else:
            self.scene2_status_signal.emit(f'启动失败: {msg}')

    def start_scene2_waste(self, waste_key=None):
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self._stop_calibration_for_play()
        self.set_scene2_mode('waste')
        self._activate_scene_for_backend(SCENE2_ID)
        self._set_scene2_result_mode('waste')
        label = '全部垃圾' if waste_key is None else WASTE_LABELS.get(waste_key, waste_key)
        self.scene2_status_signal.emit(f'正在开启垃圾分类: {label}')
        threading.Thread(target=self._scene2_waste_worker, args=(waste_key,), daemon=True).start()

    def _scene2_waste_worker(self, waste_key):
        ok, msg = self.node.start_waste_classification(waste_key)
        if ok:
            self.scene2_status_signal.emit(msg)
        else:
            self.scene2_status_signal.emit(f'启动失败: {msg}')

    def stop_scene2_color_sorting(self):
        self.set_scene_play_state(SCENE2_ID, False)
        if self.node is None:
            QMessageBox.warning(self, '提示', 'ROS 节点还未初始化')
            return
        self.scene2_status_signal.emit('正在停止夹取...')
        threading.Thread(target=self._scene2_stop_worker, daemon=True).start()

    def _scene2_stop_worker(self):
        ok, msg = self.node.stop_scene3_tasks()
        self.scene2_status_signal.emit(('已停止: ' if ok else '停止失败: ') + msg)

    def on_scene2_grid_changed(self, slots):
        scene = self.scene2_cfg()
        scene.setdefault('color_grid', {})['slots'] = list(slots)
        self.apply_scene2_slots_to_targets()
        self.scene_cfg['current_scene'] = SCENE2_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene2_grid()

    def save_scene2_grid(self):
        self.apply_scene2_slots_to_targets()
        self.scene_cfg['current_scene'] = SCENE2_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        QMessageBox.information(self, '保存成功', '场景2放置位置已保存')

    def reset_scene2_grid(self):
        scene = self.scene2_cfg()
        targets = scene.setdefault('place_targets', {})
        if getattr(self, 'scene2_mode', 'color') == 'color':
            scene['color_grid'] = {
                'slots': list(SCENE2_COLOR_KEYS),
                'slot_targets': [list(DEFAULT_SCENE2_COLOR_TARGETS[key]) for key in SCENE2_SLOT_TARGET_KEYS],
            }
            self.apply_scene2_slots_to_targets()
        else:
            for key in self.scene2_mode_keys():
                targets[key] = self.scene2_default_position(key)
        self.scene_cfg['current_scene'] = SCENE2_ID
        self.save_scene_cfg()
        self.load_scene_combo()
        self.refresh_scene2_grid()

    def load_scene_combo(self):
        self.cb_scene.blockSignals(True)
        self.cb_scene.clear()
        for scene_id in self.scene_cfg['scenes'].keys():
            scene_name = self.scene_cfg['scenes'][scene_id].get('name', scene_id)
            self.cb_scene.addItem(f'{scene_id} ({scene_name})', userData=scene_id)
        current = self.scene_cfg.get('current_scene', DEFAULT_CURRENT_SCENE)
        idx = self.cb_scene.findData(current)
        self.cb_scene.setCurrentIndex(max(0, idx))
        self.cb_scene.blockSignals(False)
        self.on_scene_changed()

    def current_scene_id(self):
        scene_id = self.cb_scene.currentData()
        if not scene_id:
            scene_id = self.scene_cfg.get('current_scene', DEFAULT_CURRENT_SCENE)
        return scene_id

    def current_scene_cfg(self):
        scene_id = self.current_scene_id()
        scene = self.scene_cfg['scenes'].setdefault(scene_id, {})
        self.normalize_scene_cfg(self.scene_cfg)
        return scene

    def resolve_target_key(self, label):
        canonical = TARGET_LABEL_ALIASES.get(str(label).strip(), str(label).strip())
        return TARGET_KEY_MAP.get(canonical)

    def get_scene_target_position(self, target_key, apply_policy=True):
        scene = self.current_scene_cfg()
        targets = scene.get('place_targets', {})
        raw = targets.get(target_key, DEFAULT_SCENE_PLACE_TARGETS.get(target_key))
        if raw is None:
            return None
        try:
            pos = [float(raw[0]), float(raw[1]), float(raw[2])]
        except Exception:
            fallback = DEFAULT_SCENE_PLACE_TARGETS.get(target_key)
            if fallback is None:
                return None
            pos = [float(fallback[0]), float(fallback[1]), float(fallback[2])]
        if apply_policy:
            policy = scene.get('place_policy', {})
            if bool(policy.get('only_left_y_positive', False)) and pos[1] < 0.0:
                pos[1] = abs(pos[1])
            try:
                min_place_z = float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z']))
                if pos[2] < min_place_z:
                    pos[2] = min_place_z
            except Exception:
                pass
        return pos

    def refresh_target_editor(self):
        if not hasattr(self, 'cb_target'):
            return
        key = self.cb_target.currentData()
        if not key:
            return
        scene = self.current_scene_cfg()
        pos = self.get_scene_target_position(key, apply_policy=False)
        if pos is None:
            return
        self.sp_target_x.blockSignals(True)
        self.sp_target_y.blockSignals(True)
        self.sp_target_z.blockSignals(True)
        self.chk_left_only.blockSignals(True)
        self.sp_min_place_z.blockSignals(True)
        self.sp_target_x.setValue(float(pos[0]))
        self.sp_target_y.setValue(float(pos[1]))
        self.sp_target_z.setValue(float(pos[2]))
        policy = scene.get('place_policy', {})
        self.chk_left_only.setChecked(bool(policy.get('only_left_y_positive', False)))
        self.sp_min_place_z.setValue(float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z'])))
        self.sp_target_x.blockSignals(False)
        self.sp_target_y.blockSignals(False)
        self.sp_target_z.blockSignals(False)
        self.chk_left_only.blockSignals(False)
        self.sp_min_place_z.blockSignals(False)

    def refresh_point_button_state(self):
        if self.combo_mode.currentIndex() == 0:
            for button in self.button_object.values():
                button.setEnabled(False)
            return
        scene = self.current_scene_cfg()
        only_left = bool(scene.get('place_policy', {}).get('only_left_y_positive', False))
        for button in self.button_object.values():
            key = self.resolve_target_key(button.text())
            if key is None:
                button.setEnabled(False)
                continue
            if only_left and key in ('right_top', 'right_bottom'):
                button.setEnabled(False)
                continue
            button.setEnabled(self.get_scene_target_position(key) is not None)

    def on_target_changed(self):
        self.refresh_target_editor()

    def on_target_value_changed(self, index, value):
        if not hasattr(self, 'cb_target'):
            return
        key = self.cb_target.currentData()
        if not key:
            return
        scene = self.current_scene_cfg()
        pos = self.get_scene_target_position(key, apply_policy=False)
        if pos is None:
            return
        pos[index] = float(value)
        scene['place_targets'][key] = pos
        if self.current_scene_id() == SCENE2_ID and key in SCENE2_COLOR_KEYS:
            slots = self.scene2_slots()
            if key in slots:
                grid = scene.setdefault('color_grid', {})
                slot_targets = self.scene2_slot_targets()
                slot_targets[slots.index(key)] = list(pos)
                grid['slot_targets'] = slot_targets
        if self.current_scene_id() == SCENE3_ID and key in tuple(SCENE2_COLOR_KEYS) + tuple(WASTE_KEYS):
            for group in (SCENE3_GROUP_COLOR, SCENE3_GROUP_WASTE):
                slots = self.scene3_slots(group)
                if key in slots:
                    grid = scene.setdefault('scene3_grid', {})
                    slot_targets = self.scene3_slot_targets(group)
                    slot_targets[slots.index(key)] = list(pos)
                    grid[SCENE3_GRID_TARGET_FIELDS[group]] = slot_targets
            scene['place_targets'][key] = pos
            self.refresh_scene3_coord_editor()
        if self.current_scene_id() == SCENE4_ID and key in tuple(SCENE4_COLOR_KEYS) + tuple(WASTE_KEYS):
            self._sync_scene4_mapped_targets(scene)
            if hasattr(self, 'scene4_board'):
                shelf = normalized_scene4_shelf(scene.get('scene4_shelf'))
                mode = getattr(self, 'scene4_mode', SCENE4_MODE_COLOR)
                shelf_slots = {
                    destination: self.scene4_shelf_slots(scene, mode, destination)
                    for destination in SCENE4_SHELF_LEVELS
                }
                self.scene4_board.set_scene_targets(
                    scene.get('place_targets', {}),
                    shelf.get('length_m', SCENE4_SHELF_LENGTH_M),
                    shelf.get('width_m', SCENE4_SHELF_WIDTH_M),
                    scene.get('scene4_place', {}).get('targets', {}),
                    shelf_slots=shelf_slots,
                    absolute_positions=scene.get('scene4_absolute_positions'),
                )

    def on_place_policy_changed(self, checked):
        scene = self.current_scene_cfg()
        scene['place_policy']['only_left_y_positive'] = bool(checked)
        self.refresh_point_button_state()

    def on_min_place_z_changed(self, value):
        scene = self.current_scene_cfg()
        scene['place_policy']['min_place_z'] = float(value)

    def on_scene_changed(self):
        scene_id = self.current_scene_id()
        self.scene_cfg['current_scene'] = scene_id
        scene = self.current_scene_cfg()
        home = scene.get('home_pose', {})
        self.home = {
            'x': float(home.get('x', 110.0)),
            'y': float(home.get('y', 0.0)),
            'z': float(home.get('z', 220.0)),
            'pitch': float(home.get('pitch', -90.0)),
            'roll': float(home.get('roll', 0.0)),
            'claw': float(home.get('claw', 0.0)),
        }
        if isinstance(home, dict) and 'time_ms' in home:
            self.home['time_ms'] = int(float(home.get('time_ms', 1500)))
        self.refresh_target_editor()
        self.refresh_point_button_state()
        self.refresh_scene2_grid()
        self.refresh_scene3_coord_editor()
        self.refresh_scene4_coord_editor()
        self.refresh_scene4_pick_editor()
        self.refresh_scene4_rail_editor()
        self.refresh_scene4_absolute_editor()
        self.refresh_scene5_grids()
        self.refresh_scene5_conveyor_editor()
        self.save_scene_cfg()

    def save_current_scene(self):
        scene_id = self.current_scene_id()
        if scene_id not in self.scene_cfg['scenes']:
            return
        if scene_id == SCENE4_ID:
            self._sync_scene4_mapped_targets(self.scene_cfg['scenes'][scene_id])
            self.scene_cfg['scenes'][scene_id]['scene4_pick'] = normalized_scene4_pick(
                self.scene_cfg['scenes'][scene_id].get('scene4_pick')
            )
            self.scene_cfg['scenes'][scene_id]['scene4_place'] = normalized_scene4_place(
                self.scene_cfg['scenes'][scene_id].get('scene4_place')
            )
            self.scene_cfg['scenes'][scene_id]['scene4_shelf'] = normalized_scene4_shelf(
                self.scene_cfg['scenes'][scene_id].get('scene4_shelf')
            )
            self.scene_cfg['scenes'][scene_id]['scene4_absolute_positions'] = normalized_scene4_absolute_positions(
                self.scene_cfg['scenes'][scene_id].get('scene4_absolute_positions')
            )
        if scene_id == SCENE5_ID:
            self.scene_cfg['scenes'][scene_id]['mode'] = 'dual_arm_single_conveyor'
            self.scene_cfg['scenes'][scene_id]['calibration_pose'] = normalized_calibration_pose(
                self.scene_cfg['scenes'][scene_id].get('calibration_pose'),
                DEFAULT_SCENE5_CALIBRATION_POSE,
            )
            self.scene5_grid_cfg()
        self.scene_cfg['current_scene'] = scene_id
        self.home = dict(self.scene_cfg['scenes'][scene_id].get('home_pose', self.home))
        self.save_scene_cfg()
        QMessageBox.information(self, '保存成功', f'场景 {scene_id} 已保存')

    def add_scene(self):
        base = 'scene_'
        i = 1
        while f'{base}{i}' in self.scene_cfg['scenes']:
            i += 1
        scene_id = f'{base}{i}'
        self.scene_cfg['scenes'][scene_id] = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG['scenes'][DEFAULT_SCENE_ID]))
        self.scene_cfg['scenes'][scene_id]['name'] = f'Scene {i}'
        self.scene_cfg['current_scene'] = scene_id
        self.save_scene_cfg()
        self.load_scene_combo()

    def delete_scene(self):
        scene_id = self.current_scene_id()
        if scene_id == DEFAULT_SCENE_ID:
            QMessageBox.warning(self, '提示', 'scene_1 为默认场景，不能删除')
            return
        self.scene_cfg['scenes'].pop(scene_id, None)
        if not self.scene_cfg['scenes']:
            self.scene_cfg = yaml.safe_load(yaml.safe_dump(DEFAULT_SCENE_CONFIG))
        self.scene_cfg['current_scene'] = next(iter(self.scene_cfg['scenes'].keys()))
        self.save_scene_cfg()
        self.load_scene_combo()

    def ensure_scene5_calibration_allowed(self):
        scene_id = self.current_scene_id() if hasattr(self, 'current_scene_id') else active_scene_id()
        if scene5_calibration_allowed(scene_id):
            return True
        QMessageBox.warning(
            self,
            '提示',
            'Scene5校准只能在A机械臂上操作。\n请确认接屏幕的是A机械臂，并在tool中选择A机械臂后重启。',
        )
        return False

    def index_changed(self, i):
        if i == 0:
            if not self.ensure_scene5_calibration_allowed():
                return
            self.node.enter_calibration()
            self.scale_x.setEnabled(False)
            self.scale_y.setEnabled(False)
            self.scale_z.setEnabled(False)
            self.offset_x.setEnabled(False)
            self.offset_y.setEnabled(False)
            self.offset_z.setEnabled(False)
            self.pushButton_reset.setEnabled(False)
            self.pushButton_save.setEnabled(False)
            self.pushButton_init.setText('标定')
            self.refresh_point_button_state()
        else:
            self.node.exit_calibration()
            self.node.enable_calibration(False)
            self.pushButton_clear_grab_calib.setVisible(i == 3)
            self.scale_x.setEnabled(i != 3)
            self.scale_y.setEnabled(i != 3)
            self.scale_z.setEnabled(i != 3)
            self.offset_x.setEnabled(True)
            self.offset_y.setEnabled(True)
            self.offset_z.setEnabled(True)
            self.pushButton_reset.setEnabled(True)
            self.pushButton_save.setEnabled(True)
            self.pushButton_init.setText('复位')
            if i == 1:
                params = self.params.get('pixel', {})
            elif i == 2:
                params = self.params.get('depth', {})
            elif i == 3:
                params = self.params.get('kinematics', {})
            else:
                params = {}
            offset = params.get('offset', [0, 0, 0])
            scale = params.get('scale', [1, 1, 1])
            self.offset_x.setValue(offset[0])
            self.offset_y.setValue(offset[1])
            self.offset_z.setValue(offset[2])
            self.scale_x.setValue(scale[0])
            self.scale_y.setValue(scale[1])
            self.scale_z.setValue(scale[2])
            self.refresh_point_button_state()

    def init_pose(self):
        self.node.init_pose(self.home)

    def calibration_position(self, position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if yaw > 45:
            yaw = math.degrees(math.atan2(-position[0], position[1]))
            position = [position[0] * self.scale_y.value(), position[1] * self.scale_x.value(), position[2] * self.scale_z.value()]
            position = [position[0] - self.offset_y.value(), position[1] + self.offset_x.value(), position[2] + self.offset_z.value()]
        elif yaw < -45:
            yaw = math.degrees(math.atan2(position[0], -position[1]))
            position = [position[0] * self.scale_y.value(), position[1] * self.scale_x.value(), position[2] * self.scale_z.value()]
            position = [position[0] + self.offset_y.value(), position[1] - self.offset_x.value(), position[2] + self.offset_z.value()]
        else:
            position = [position[0] * self.scale_x.value(), position[1] * self.scale_y.value(), position[2] * self.scale_z.value()]
            position = [position[0] + self.offset_x.value(), position[1] + self.offset_y.value(), position[2] + self.offset_z.value()]
        return position, float(yaw)

    def move_to_position(self, position):
        if self.current_scene_id() == SCENE4_ID:
            result = self.node.prepare_scene_runtime()
            if result is None or not getattr(result, 'success', False):
                msg = getattr(result, 'message', '无响应') if result is not None else '无响应'
                QMessageBox.warning(self, '提示', f'场景4底层参数准备失败: {msg}')
                return
        self.init_pose()
        position, roll_deg = self.calibration_position(position)
        # Scene 4 RGBD pipeline: depth calibration is already applied by calibration_position;
        # additionally chain the kinematics calibration offset to match object_sorting.py behavior.
        if (self.current_scene_id() == SCENE4_ID and
                hasattr(self, 'combo_mode') and
                self.combo_mode.currentIndex() == 2):
            kin = self.params.get('kinematics', {})
            kin_offset = kin.get('offset', [0.0, 0.0, 0.0])
            kin_scale = kin.get('scale', [1.0, 1.0, 1.0])
            position = [
                float(position[0]) * float(kin_scale[0]) + float(kin_offset[0]),
                float(position[1]) * float(kin_scale[1]) + float(kin_offset[1]),
                float(position[2]) * float(kin_scale[2]) + float(kin_offset[2]),
            ]
        self.node.set_position(position, roll_deg, self.home)

    def resolve_target_position(self, button_label):
        key = self.resolve_target_key(button_label)
        if not key:
            return None
        return self.get_scene_target_position(key, apply_policy=True)

    def button_clicked(self, name):
        if name == 'init':
            if self.combo_mode.currentIndex() == 0:
                if not self.ensure_scene5_calibration_allowed():
                    return
                self.save_current_scene()
                self.node.enter_calibration()
                self.node.start_calibration()
                self.node.enable_calibration(True)
            else:
                self.init_pose()
        elif name == 'reset':
            self.offset_x.setValue(0)
            self.offset_y.setValue(0)
            self.offset_z.setValue(0)
            self.scale_x.setValue(1.0)
            self.scale_y.setValue(1.0)
            self.scale_z.setValue(1.0)
        elif name == 'save':
            save_positions_yaml(self.params)
            self.pushButton_save.setEnabled(False)
        elif name == 'clear_grab_calib':
            self.node.clear_grab_calibration()
        else:
            position = self.resolve_target_position(name)
            if position is not None:
                self.move_to_position(position)

    def value_changed(self, key, idx, value):
        if self.combo_mode.currentIndex() == 1:
            self.params['pixel'][key][idx] = value
        elif self.combo_mode.currentIndex() == 2:
            self.params['depth'][key][idx] = value
        elif self.combo_mode.currentIndex() == 3:
            self.params.setdefault('kinematics', {}).setdefault(key, [0.0, 0.0, 0.0])[idx] = value
        if not self.pushButton_save.isEnabled():
            self.pushButton_save.setEnabled(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
