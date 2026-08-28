#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import yaml
import math
import rclpy
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from rclpy.node import Node
from ui import Ui_MainWindow
from std_srvs.srv import SetBool, Trigger
from ros_robot_controller_msgs.msg import ArmCoords


TARGET_POSITION_MAP = {
    'Center': [0.235, 0.0, 0.015],
    'Left Top': [0.285, 0.16, 0.015],
    'Right Top': [0.285, -0.16, 0.015],
    'Left Bottom': [0.115, 0.16, 0.015],
    'Right Bottom': [0.115, -0.16, 0.015],
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
if chassis_type == 'Slide_Rails':
    POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration.yaml"
    SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
else:
    POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration.yaml"
    SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"

init_finish = False

def load_scene_home_pose():
    home = {
        'x': 110.0,
        'y': 0.0,
        'z': 220.0,
        'pitch': -90.0,
        'roll': 0.0,
        'claw': 0.0,
        'time_ms': 1000,
    }
    try:
        with open(SCENE_YAML_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if isinstance(scenes, dict) and scenes:
            scene_name = str(cfg.get('current_scene', 'scene_1'))
            if scene_name not in scenes:
                scene_name = next(iter(scenes.keys()))
            scene = scenes.get(scene_name, {}) if isinstance(scenes.get(scene_name), dict) else {}
            hp = scene.get('home_pose', {}) if isinstance(scene.get('home_pose'), dict) else {}
            for key, default_value in list(home.items()):
                if key == 'time_ms':
                    home[key] = int(float(hp.get(key, default_value)))
                else:
                    home[key] = float(hp.get(key, default_value))
    except Exception:
        pass
    return home


HOME_POSE = load_scene_home_pose()
HOME_X = HOME_POSE['x']
HOME_Y = HOME_POSE['y']
HOME_Z = HOME_POSE['z']
HOME_PITCH = HOME_POSE['pitch']
HOME_ROLL = HOME_POSE['roll']
HOME_CLAW = HOME_POSE['claw']
HOME_TIME_MS = HOME_POSE['time_ms']


class ArmControlNode(Node):
    def __init__(self, name):
        global init_finish
        if not rclpy.ok():
            rclpy.init()
        super().__init__(name)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.calibration_enable_client = self.create_client(SetBool, 'calibration/start_calibration')
        self.enter_calibration_client = self.create_client(Trigger, 'calibration/enter')
        self.start_calibration_client = self.create_client(Trigger, 'calibration/start')
        self.exit_calibration_client = self.create_client(Trigger, 'calibration/exit')
        while self.arm_pub.get_subscription_count() == 0:
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

    def send_request(self, client, msg):
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'服务不可用: {client.srv_name}')
            return None
        future = client.call_async(msg)
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def enter_calibration(self):
        return self.send_request(self.enter_calibration_client, Trigger.Request())

    def exit_calibration(self):
        return self.send_request(self.exit_calibration_client, Trigger.Request())

    def start_calibration(self):
        return self.send_request(self.start_calibration_client, Trigger.Request())

    def enable_calibration(self, enable):
        msg = SetBool.Request()
        msg.data = bool(enable)
        return self.send_request(self.calibration_enable_client, msg)

    def init_pose(self):
        self.publish_arm(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL, HOME_CLAW, HOME_TIME_MS)
        time.sleep(max(0.0, HOME_TIME_MS / 1000.0))
        self.publish_arm(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL, HOME_CLAW, HOME_TIME_MS)
        time.sleep(max(0.0, HOME_TIME_MS / 1000.0))

    def set_position(self, position, roll_deg, time_ms=1500):
        x_mm = float(position[0]) * 1000.0
        y_mm = float(position[1]) * 1000.0
        z_mm = float(position[2]) * 1000.0
        self.publish_arm(x_mm, y_mm, z_mm, HOME_PITCH, float(roll_deg), HOME_CLAW, time_ms)


class MainWindow(Ui_MainWindow, QMainWindow):
    def __init__(self):
        super().__init__()
        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.node = None
        threading.Thread(target=self.ros_node, daemon=True).start()
        while not init_finish:
            time.sleep(0.1)
        self.setupUi(self)

        self.pushButton_init.pressed.connect(lambda: self.button_clicked('init'))
        self.pushButton_reset.pressed.connect(lambda: self.button_clicked('reset'))
        self.pushButton_save.pressed.connect(lambda: self.button_clicked('save'))
        self.comboBox.currentIndexChanged.connect(self.index_changed)

        with open(POSITIONS_YAML_PATH, 'r', encoding='utf-8') as f:
            self.params = yaml.safe_load(f)

        self.offset_x.valueChanged.connect(lambda value: self.value_changed('offset', 0, value))
        self.offset_y.valueChanged.connect(lambda value: self.value_changed('offset', 1, value))
        self.offset_z.valueChanged.connect(lambda value: self.value_changed('offset', 2, value))
        self.scale_x.valueChanged.connect(lambda value: self.value_changed('scale', 0, value))
        self.scale_y.valueChanged.connect(lambda value: self.value_changed('scale', 1, value))
        self.scale_z.valueChanged.connect(lambda value: self.value_changed('scale', 2, value))

        self.button_object = {}
        for i in range(16):
            self.button_object[i] = self.findChild(QPushButton, f'pushButton_{i + 1}')
            self.button_object[i].pressed.connect(lambda i=i: self.button_clicked(self.button_object[i].text()))

        self.index_changed(0)

    def ros_node(self):
        self.node = ArmControlNode('calibration_main')
        rclpy.spin(self.node)
        self.node.destroy_node()

    def index_changed(self, i):
        if i == 0:
            self.node.enter_calibration()
            self.scale_x.setEnabled(False)
            self.scale_y.setEnabled(False)
            self.scale_z.setEnabled(False)
            self.offset_x.setEnabled(False)
            self.offset_y.setEnabled(False)
            self.offset_z.setEnabled(False)
            self.pushButton_reset.setEnabled(False)
            self.pushButton_save.setEnabled(False)
            if self.language == 'Chinese':
                self.pushButton_init.setText('标定')
            if self.language == 'English':
                self.pushButton_init.setText('Calibration')
            for button in self.button_object.values():
                button.setEnabled(False)
        else:
            self.node.exit_calibration()
            self.node.enable_calibration(False)
            self.scale_x.setEnabled(False)
            self.scale_y.setEnabled(True)
            self.scale_z.setEnabled(False)
            self.offset_x.setEnabled(True)
            self.offset_y.setEnabled(True)
            self.offset_z.setEnabled(True)
            self.pushButton_reset.setEnabled(True)
            self.pushButton_save.setEnabled(True)
            if self.language == 'Chinese':
                self.pushButton_init.setText('复位')
            if self.language == 'English':
                self.pushButton_init.setText('Init')

            if i == 1:
                for button in self.button_object.values():
                    button.setEnabled(self.resolve_target_position(button.text()) is not None)
                params = self.params.get('kinematics', {})
            elif i == 2:
                for button in self.button_object.values():
                    button.setEnabled(False)
                params = self.params.get('pixel', {})
            elif i == 3:
                for button in self.button_object.values():
                    button.setEnabled(False)
                params = self.params.get('depth', {})
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

    def init_pose(self):
        self.node.init_pose()

    def calibration_position(self, position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if yaw > 45:
            yaw = math.degrees(math.atan2(-position[0], position[1]))
            position = [
                position[0] * self.scale_y.value(),
                position[1] * self.scale_x.value(),
                position[2] * self.scale_z.value(),
            ]
            position = [
                position[0] - self.offset_y.value(),
                position[1] + self.offset_x.value(),
                position[2] + self.offset_z.value(),
            ]
        elif yaw < -45:
            yaw = math.degrees(math.atan2(position[0], -position[1]))
            position = [
                position[0] * self.scale_y.value(),
                position[1] * self.scale_x.value(),
                position[2] * self.scale_z.value(),
            ]
            position = [
                position[0] + self.offset_y.value(),
                position[1] - self.offset_x.value(),
                position[2] + self.offset_z.value(),
            ]
        else:
            position = [
                position[0] * self.scale_x.value(),
                position[1] * self.scale_y.value(),
                position[2] * self.scale_z.value(),
            ]
            position = [
                position[0] + self.offset_x.value(),
                position[1] + self.offset_y.value(),
                position[2] + self.offset_z.value(),
            ]
        return position, float(yaw)

    def move_to_position(self, position):
        self.init_pose()
        raw_pos = list(position)
        position, roll_deg = self.calibration_position(position)
        print(f'[MOVE] 地图坐标: [{raw_pos[0]:.4f}, {raw_pos[1]:.4f}, {raw_pos[2]:.4f}] -> 校准后: [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}] roll={roll_deg:.1f} -> mm: [{position[0]*1000:.1f}, {position[1]*1000:.1f}, {position[2]*1000:.1f}]')
        self.node.set_position(position, roll_deg)

    def resolve_target_position(self, button_label):
        label = str(button_label).strip()
        if not label:
            return None
        canonical_label = TARGET_LABEL_ALIASES.get(label, label)
        return TARGET_POSITION_MAP.get(canonical_label)

    def button_clicked(self, name):
        if name == 'init':
            if self.comboBox.currentIndex() == 0:
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
            with open(POSITIONS_YAML_PATH, 'w', encoding='utf-8') as f:
                yaml.safe_dump(self.params, f, sort_keys=False)
            self.pushButton_save.setEnabled(False)
        else:
            position = self.resolve_target_position(name)
            if position is not None:
                self.move_to_position(position)

    def value_changed(self, key, idx, value):
        if self.comboBox.currentIndex() == 1:
            self.params['kinematics'][key][idx] = value
        elif self.comboBox.currentIndex() == 2:
            self.params['pixel'][key][idx] = value
        elif self.comboBox.currentIndex() == 3:
            self.params['depth'][key][idx] = value
        if not self.pushButton_save.isEnabled():
            self.pushButton_save.setEnabled(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
