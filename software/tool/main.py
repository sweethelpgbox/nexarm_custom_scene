#!/usr/bin/env python3
# encoding: utf-8
# Date:2022/10/20
# Author:aiden
import os
import re
import sys
import ast
import distro
import psutil
import platform
import importlib
import subprocess
import time
import yaml
from PyQt5.QtCore import Qt, QThread, QEvent, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QMessageBox,
    QDesktopWidget,
    QLabel,
    QComboBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFrame,
    QAbstractItemView,
    QHeaderView,
)
if __name__ == '__main__':
    import ui
    import message_main
else:
    from tool import ui
    from tool import message_main

class ResetServiceThread(QThread):
    def __init__(self, restart_wifi=False):
        super(ResetServiceThread,self).__init__()
        self.restart_wifi = restart_wifi
    
    def run(self):
        os.system('sudo systemctl restart find_device.service')
        os.system('sudo systemctl restart start_app_node.service')
        if self.restart_wifi:
            os.system('sudo systemctl restart wifi.service')


class WifiScanThread(QThread):
    finished_scan = pyqtSignal(object, str)

    def run(self):
        restore_ap = False
        try:
            iw_info = subprocess.run(
                ['iw', 'dev', 'wlan0', 'info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=4,
            )
            restore_ap = iw_info.returncode == 0 and wifi_scan_requires_ap_cycle(iw_info.stdout)
            if restore_ap:
                active = subprocess.run(
                    ['nmcli', '-t', '-f', 'NAME,TYPE,DEVICE', 'con', 'show', '--active'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True,
                    timeout=4,
                )
                active_wifi = active_wifi_connection_from_nmcli_output(active.stdout) if active.returncode == 0 else ""
                subprocess.run(['sudo', 'systemctl', 'stop', 'wifi.service'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, universal_newlines=True, timeout=8)
                if active_wifi:
                    subprocess.run(['nmcli', 'con', 'down', active_wifi], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, universal_newlines=True, timeout=8)
                subprocess.run(['nmcli', 'dev', 'wifi', 'rescan', 'ifname', 'wlan0'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
                time.sleep(1.0)
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'dev', 'wifi', 'list', 'ifname', 'wlan0', '--rescan', 'yes'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=12,
            )
        except Exception as e:
            self.finished_scan.emit([], str(e))
            return
        finally:
            if restore_ap:
                subprocess.run(['sudo', 'systemctl', 'restart', 'wifi.service'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, universal_newlines=True, timeout=8)
        if result.returncode != 0:
            self.finished_scan.emit([], result.stderr.strip() or "nmcli scan failed")
            return
        self.finished_scan.emit(parse_wifi_scan_output(result.stdout), "")

HW_WIFI_AP_SSID = ""
EXPORT_RE = re.compile(r"^\s*export\s+([A-Za-z_][A-Za-z0-9_]*)=(.*)\s*$")
NEXARM_DOCKER_CONTAINER = os.environ.get("NEXARM_DOCKER_CONTAINER", "nexarm")
ROS2_WS_TYPERC_PATH = os.environ.get("CALIBRATION_TYPERC_PATH", "/home/ubuntu/ros2_ws/.typerc")
APP_SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_YAML_PATH = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
WIFI_CONFIG_PATH = "/home/pi/hiwonder-toolbox/wifi_conf.py"
DOCKER_TARGET_PREFIXES = (
    "/home/ubuntu/ros2_ws/",
)
SCENE_CHOICES = (
    ("scene_0", "Scene 0"),
    ("scene_1", "Scene 1"),
    ("scene_2", "Scene 2"),
    ("scene_3", "Scene 3"),
    ("scene_4", "Scene 4"),
    ("scene_5", "Scene 5"),
)
SCENE5_ARM_ROLE_CHOICES = (
    ("A", "A机械臂"),
    ("B", "B机械臂"),
)
MACHINE_TYPE_CHOICES = ("NexArm", "NexArm_Mecanum", "NexArm_Track")
MACHINE_TYPE_ALIASES = {
    "NexArm_Meacuam": "NexArm_Mecanum",
}


def is_docker_target_path(path):
    absolute = os.path.abspath(path)
    return any(absolute.startswith(prefix) for prefix in DOCKER_TARGET_PREFIXES)


def running_inside_container():
    return os.path.exists("/.dockerenv")


def docker_container_running(container=None):
    container = container or NEXARM_DOCKER_CONTAINER
    if not container:
        return False
    try:
        result = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", container],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=3,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return False
    return result.returncode == 0 and result.stdout.strip().lower() == "true"


def should_use_docker_path(path):
    return is_docker_target_path(path) and docker_container_running()


def docker_user_for_path(path):
    if os.path.abspath(path).startswith("/home/ubuntu/"):
        return "ubuntu"
    return None


def docker_exec(args, input_text=None, user=None):
    command = ["docker", "exec"]
    if input_text is not None:
        command.append("-i")
    if user:
        command.extend(["-u", user])
    command.extend([NEXARM_DOCKER_CONTAINER] + list(args))
    try:
        return subprocess.run(
            command,
            input=input_text,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
    except FileNotFoundError as e:
        raise RuntimeError("docker command not found") from e


def docker_path_exists(path):
    result = docker_exec(["test", "-e", path])
    return result.returncode == 0


def docker_read_text(path):
    result = docker_exec(["cat", path], user=docker_user_for_path(path))
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise RuntimeError(f"failed to read {NEXARM_DOCKER_CONTAINER}:{path}: {message}")
    return result.stdout


def docker_write_text(path, content):
    script = 'directory=$(dirname "$1") && mkdir -p "$directory" && cat > "$1"'
    result = docker_exec(
        ["/bin/sh", "-c", script, "sh", path],
        input_text=content,
        user=docker_user_for_path(path),
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise RuntimeError(f"failed to write {NEXARM_DOCKER_CONTAINER}:{path}: {message}")


def path_exists(path):
    if should_use_docker_path(path):
        return docker_path_exists(path)
    return os.path.exists(path)


def read_text_file(path):
    if should_use_docker_path(path):
        return docker_read_text(path)
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_text_file(path, content):
    if should_use_docker_path(path):
        docker_write_text(path, content)
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def scene_yaml_path_for(scene_id=None):
    if scene_id == "scene_4":
        return STEPPER_SCENE_YAML_PATH
    if scene_id == "scene_5":
        return APP_SCENE_YAML_PATH
    if os.environ.get("CHASSIS_TYPE") == "Slide_Rails":
        return STEPPER_SCENE_YAML_PATH
    return APP_SCENE_YAML_PATH


def typerc_path_for_tool(tool_path):
    candidates = [
        ROS2_WS_TYPERC_PATH,
        os.path.abspath(os.path.join(tool_path, "../../ros2_ws/.typerc")),
        os.path.abspath(os.path.join(tool_path, "../../.typerc")),
        "/home/pi/docker/tmp/.typerc",
    ]
    seen = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        if path_exists(path):
            return path
    return candidates[0]


def chassis_type_for_scene(scene_id):
    if scene_id == "scene_4":
        return "Slide_Rails"
    if scene_id == "scene_5":
        return "Conveyor_Belt"
    return "None"


def normalize_scene5_arm_role(role):
    role = str(role or "A").strip().upper()
    return role if role in {"A", "B"} else "A"


def normalize_machine_type(machine):
    machine = str(machine or "NexArm").strip()
    return MACHINE_TYPE_ALIASES.get(machine, machine) or "NexArm"


def clean_export_value(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        value = value[1:-1]
    return value


def parse_exports(text):
    exports = {}
    for line in text.splitlines():
        match = EXPORT_RE.match(line)
        if not match:
            continue
        exports[match.group(1)] = clean_export_value(match.group(2))
    return exports


def export_value(exports, *keys, default=""):
    for key in keys:
        value = exports.get(key)
        if value not in (None, ""):
            return value
    return default


def replace_or_append_export(text, key, value):
    value = str(value)
    line = f"export {key}={value}"
    pattern = re.compile(rf"^\s*export\s+{re.escape(key)}=.*$", re.MULTILINE)
    if pattern.search(text):
        return pattern.sub(line, text, count=1)
    if text and not text.endswith("\n"):
        text += "\n"
    echo_match = re.search(r"(?m)^echo\s", text)
    if echo_match:
        return text[:echo_match.start()] + line + "\n" + text[echo_match.start():]
    return text + line + "\n"


def remove_export(text, key):
    pattern = re.compile(rf"^\s*export\s+{re.escape(key)}=.*(?:\n|$)", re.MULTILINE)
    return pattern.sub("", text)


def update_scene_exports(text, scene_id, scene5_arm_role=None):
    text = remove_export(text, "CALIBRATION_DEFAULT_SCENE")
    text = replace_or_append_export(text, "CALIBRATION_CURRENT_SCENE", scene_id)
    text = replace_or_append_export(text, "CHASSIS_TYPE", chassis_type_for_scene(scene_id))
    if scene_id == "scene_5":
        text = replace_or_append_export(text, "SCENE5_ARM_ROLE", normalize_scene5_arm_role(scene5_arm_role))
    else:
        text = remove_export(text, "SCENE5_ARM_ROLE")
    return text


def write_scene_yaml_current_scene(scene_yaml_path, scene_id):
    data = {}
    if path_exists(scene_yaml_path):
        data = yaml.safe_load(read_text_file(scene_yaml_path)) or {}
    data.setdefault("scenes", {})
    data["current_scene"] = scene_id
    write_text_file(scene_yaml_path, yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def wifi_config_path():
    return WIFI_CONFIG_PATH


def ensure_combo_value(items, value):
    value = str(value)
    return items if value in items else items + [value]


def split_nmcli_terse_line(line):
    fields = []
    current = []
    escaped = False
    for char in line.rstrip("\n"):
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    fields.append("".join(current))
    return fields


def parse_wifi_scan_output(text):
    networks = {}
    for line in text.splitlines():
        fields = split_nmcli_terse_line(line)
        if len(fields) < 3:
            continue
        ssid = fields[0].strip()
        if not ssid:
            continue
        try:
            signal = int(fields[1] or 0)
        except ValueError:
            signal = 0
        security = ":".join(fields[2:]).strip() or "Open"
        if ssid not in networks or signal > networks[ssid]["signal"]:
            networks[ssid] = {"ssid": ssid, "signal": signal, "security": security}
    return sorted(networks.values(), key=lambda item: item["signal"], reverse=True)


def render_wifi_config(ssid, password):
    return (
        "#!/usr/bin/python3\n"
        "#coding:utf8\n\n"
        "WIFI_MODE = 2                              #wifi的工作模式， 1为AP模式， 2为STA模式\n"
        "#WIFI_AP_SSID = 'WN-Robot'                  #AP模式下的SSID。字符和数字构成,需要以 HW- 开头，否则app功能无法使用\n"
        "WIFI_AP_PASSWORD = 'hiwonder'        #AP模式下的WIFI密码,字符和数字构成\n"
        "# WIFI_STA_SSID = 'your_wifi_name'            #STA模式下的SSID\n"
        "# WIFI_STA_PASSWORD = 'your_wifi_password'    #STA模式下的WIFI密码 \n"
        f"WIFI_STA_SSID = {str(ssid)!r}            #STA模式下的SSID\n"
        f"WIFI_STA_PASSWORD = {str(password)!r}    #STA模式下的WIFI密码 \n"
    )


def replace_or_append_python_assignment(text, key, value):
    value_text = repr(value) if isinstance(value, str) else str(value)
    pattern = re.compile(
        rf"^(\s*{re.escape(key)}\s*=\s*)(.*?)(\s*(?:#.*)?)$",
        re.MULTILINE,
    )
    if pattern.search(text):
        return pattern.sub(lambda match: f"{match.group(1)}{value_text}{match.group(3)}", text, count=1)
    if text and not text.endswith("\n"):
        text += "\n"
    return text + f"{key} = {value_text}\n"


def update_wifi_config_text(text, ssid, password):
    text = replace_or_append_python_assignment(text, "WIFI_MODE", 2)
    text = replace_or_append_python_assignment(text, "WIFI_STA_SSID", ssid)
    text = replace_or_append_python_assignment(text, "WIFI_STA_PASSWORD", password)
    return text


def write_wifi_config_file(path, ssid, password):
    ssid = validate_wifi_ssid(ssid)
    if ssid is None:
        raise ValueError("WiFi SSID is empty")
    if path_exists(path):
        content = update_wifi_config_text(read_text_file(path), ssid, password)
    else:
        content = render_wifi_config(ssid, password)
    if should_use_docker_path(path):
        write_text_file(path, content)
        return
    directory = os.path.dirname(path)
    if directory and (not os.path.exists(directory) or os.access(directory, os.W_OK)):
        os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        return
    if os.path.abspath(path).startswith("/etc/"):
        if directory:
            subprocess.run(["sudo", "mkdir", "-p", directory], check=True)
        subprocess.run(
            ["sudo", "tee", path],
            input=content,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
        )
        return
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def read_wifi_config(path):
    values = {}
    if not path_exists(path):
        return values
    tree = ast.parse(read_text_file(path), filename=path)
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name):
            continue
        key = node.targets[0].id
        if key not in {"WIFI_MODE", "WIFI_STA_SSID", "WIFI_STA_PASSWORD"}:
            continue
        try:
            values[key] = ast.literal_eval(node.value)
        except (ValueError, SyntaxError):
            continue
    return values


def validate_wifi_ssid(ssid):
    ssid = str(ssid or "").strip()
    return ssid or None


def parse_iw_interface_type(text):
    for line in text.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and parts[0] == "type":
            return parts[1].strip()
    return ""


def wifi_scan_requires_ap_cycle(iw_info_text):
    return parse_iw_interface_type(iw_info_text) == "AP"


def active_wifi_connection_from_nmcli_output(text, ifname="wlan0"):
    for line in text.splitlines():
        fields = split_nmcli_terse_line(line)
        if len(fields) >= 3 and fields[1] == "802-11-wireless" and fields[2] == ifname:
            return fields[0]
    return ""


class MainWindow(QWidget, ui.Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.create_scene_controls()
        self.hide_unused_hardware_controls()
        self.config_file_name = "wifi_conf.py"
        self.external_config_file_dir_path = '/home/pi/hiwonder-toolbox'
        self.external_config_file_path = os.path.join(self.external_config_file_dir_path, self.config_file_name)
        self.haved_save = False
        self.reset_service_thread = None
        self.message = message_main.Message()
        
        self.pushButton_apply.pressed.connect(lambda: self.button_clicked('apply'))
        self.pushButton_save.pressed.connect(lambda: self.button_clicked('save'))
        self.pushButton_exit.pressed.connect(lambda: self.button_clicked('exit'))
        self.pushButton_apply.setToolTip("restart start_app_node.service")

        self.radioButton_zn.toggled.connect(lambda: self.change_language(self.radioButton_zn))
        self.radioButton_en.toggled.connect(lambda: self.change_language(self.radioButton_en))        
        self.chinese = True

        try:
            if os.environ['ASR_LANGUAGE'] == 'English':
                self.chinese = False
                self.radioButton_en.setChecked(True)
            else:               
                self.radioButton_zn.setChecked(True)
        except:
            self.radioButton_zn.setChecked(True)	

        self.wifi_scan_thread = None
        self.create_tab_pages()
        self.set_window_position()
        
        self.path = os.path.split(os.path.realpath(__file__))[0]
        self.typerc_path = typerc_path_for_tool(self.path)
        self.scene_yaml_path = scene_yaml_path_for()
        self.language = None
        self.depth_camera = None
        self.lidar = None
        self.machine = None
        self.version = None
        self.mic = None
        self.asr_mode = None
        self.scene = None
        self.scene5_arm_role = None
        self.get_typerc()
        
        self.depth_camera = 'aurora'
        self.lidar = 'None'
        depth_camera_list = ['aurora']
        machine_list = ensure_combo_value(list(MACHINE_TYPE_CHOICES), self.machine)
        language_list = ['Chinese', 'English']
        mic_type_list = ensure_combo_value(['WonderEchoPro', 'xf'], self.mic)
        asr_mode_list = ensure_combo_value(['offline', 'online'], self.asr_mode)
        scene_list = [scene_id for scene_id, _ in SCENE_CHOICES]

        self.comboBox_depth_camera.addItems(depth_camera_list)
        self.comboBox_machine.addItems(machine_list)
        self.comboBox_asr.addItems(language_list)
        self.comboBox_depth_camera.setCurrentIndex(depth_camera_list.index(self.depth_camera))
        self.comboBox_machine.setCurrentIndex(machine_list.index(self.machine))
        self.comboBox_asr.setCurrentIndex(language_list.index(self.language))
        self.comboBox_mic.addItems(mic_type_list)
        self.comboBox_mic.setCurrentIndex(mic_type_list.index(self.mic))
        self.comboBox_asr_mode.addItems(asr_mode_list)
        self.comboBox_asr_mode.setCurrentIndex(asr_mode_list.index(self.asr_mode))
        for scene_id, label in SCENE_CHOICES:
            self.comboBox_scene.addItem(label, scene_id)
        scene_index = scene_list.index(self.scene) if self.scene in scene_list else 0
        self.comboBox_scene.setCurrentIndex(scene_index)
        role_values = [role for role, _ in SCENE5_ARM_ROLE_CHOICES]
        for role, label in SCENE5_ARM_ROLE_CHOICES:
            self.comboBox_scene5_arm_role.addItem(label, role)
        role = normalize_scene5_arm_role(self.scene5_arm_role)
        self.comboBox_scene5_arm_role.setCurrentIndex(role_values.index(role))
        self.comboBox_scene.currentIndexChanged.connect(self.update_scene5_role_visibility)
        self.update_scene5_role_visibility()

        self.lineEdit_version.setFocusPolicy(Qt.NoFocus)
        
        self.label_platform.setText(self.get_platform())
        self.label_kernel.setText(self.get_kernel())
        self.label_disk.setText(self.get_disk_space())
        self.label_memory.setText(self.get_memory())
        self.label_wlan.setText(self.get_wlan())
        self.label_eth.setText(self.get_eth())
        self.lineEdit_wifi.setText(self.get_hw())
        self.lineEdit_version.setText(self.version)

    def create_scene_controls(self):
        self.splitter_scene = QSplitter(self.layoutWidget3)
        self.splitter_scene.setOrientation(Qt.Horizontal)
        self.splitter_scene.setObjectName("splitter_scene")
        self.label_scene = QLabel(self.splitter_scene)
        self.label_scene.setMinimumSize(self.label_mic.minimumSize())
        self.label_scene.setMaximumSize(self.label_mic.maximumSize())
        self.label_scene.setAlignment(Qt.AlignLeading | Qt.AlignLeft | Qt.AlignVCenter)
        self.comboBox_scene = QComboBox(self.splitter_scene)
        self.comboBox_scene.setMinimumSize(self.comboBox_mic.minimumSize())
        self.comboBox_scene.setMaximumSize(self.comboBox_mic.maximumSize())
        self.comboBox_scene.setStyleSheet(self.comboBox_mic.styleSheet())
        self.comboBox_scene.setObjectName("comboBox_scene")
        self.verticalLayout_2.addWidget(self.splitter_scene)

        self.splitter_scene5_arm_role = QSplitter(self.layoutWidget3)
        self.splitter_scene5_arm_role.setOrientation(Qt.Horizontal)
        self.splitter_scene5_arm_role.setObjectName("splitter_scene5_arm_role")
        self.label_scene5_arm_role = QLabel(self.splitter_scene5_arm_role)
        self.label_scene5_arm_role.setMinimumSize(self.label_mic.minimumSize())
        self.label_scene5_arm_role.setMaximumSize(self.label_mic.maximumSize())
        self.label_scene5_arm_role.setAlignment(Qt.AlignLeading | Qt.AlignLeft | Qt.AlignVCenter)
        self.comboBox_scene5_arm_role = QComboBox(self.splitter_scene5_arm_role)
        self.comboBox_scene5_arm_role.setMinimumSize(self.comboBox_mic.minimumSize())
        self.comboBox_scene5_arm_role.setMaximumSize(self.comboBox_mic.maximumSize())
        self.comboBox_scene5_arm_role.setStyleSheet(self.comboBox_mic.styleSheet())
        self.comboBox_scene5_arm_role.setObjectName("comboBox_scene5_arm_role")
        self.verticalLayout_2.addWidget(self.splitter_scene5_arm_role)

    def update_scene5_role_visibility(self):
        scene = self.comboBox_scene.currentData() if hasattr(self, 'comboBox_scene') else None
        visible = scene == "scene_5"
        if hasattr(self, 'splitter_scene5_arm_role'):
            self.splitter_scene5_arm_role.setVisible(visible)

    def hide_unused_hardware_controls(self):
        if hasattr(self, 'splitter_2'):
            self.splitter_2.hide()

    def create_tab_pages(self):
        self.resize(750, 520)
        self.setMinimumSize(750, 520)
        self.setMaximumSize(750, 520)
        self.widget.setMinimumSize(750, 520)
        self.widget.setMaximumSize(750, 520)

        self.pushButton_save.move(420, 430)
        self.pushButton_apply.move(520, 430)
        self.pushButton_exit.move(620, 430)

        self.tabWidget = QTabWidget(self.widget)
        self.tabWidget.setGeometry(10, 40, 720, 380)
        self.tabWidget.setObjectName("tabWidget")
        self.tabWidget.setStyleSheet(
            "QTabWidget::pane{border:1px solid #C9C9C9;background:#F7F7F7;}"
            "QTabBar::tab{min-width:118px;min-height:30px;padding:4px 14px;background:#D9D9D9;}"
            "QTabBar::tab:selected{background:#FFA500;color:#000;}"
        )

        self.params_tab = QWidget()
        self.params_tab.setObjectName("params_tab")
        self.wifi_tab = QWidget()
        self.wifi_tab.setObjectName("wifi_tab")
        self.tabWidget.addTab(self.params_tab, "")
        self.tabWidget.addTab(self.wifi_tab, "")

        self.layoutWidget1.setParent(self.params_tab)
        self.layoutWidget1.setGeometry(8, 8, 700, 122)
        self.layoutWidget.setParent(self.params_tab)
        self.layoutWidget.setGeometry(8, 148, 506, 190)
        self.create_wifi_page()

        self.radioButton_zn.raise_()
        self.radioButton_en.raise_()
        self.pushButton_save.raise_()
        self.pushButton_apply.raise_()
        self.pushButton_exit.raise_()
        self.update_wifi_page_language()

    def create_wifi_page(self):
        combo_style = self.comboBox_mic.styleSheet()
        button_style = (
            "QPushButton{background-color:#FFA500;color:#000;border-radius:5px;padding:4px 8px;}"
            "QPushButton:pressed{border:2px solid rgb(126,188,89);}"
            "QPushButton:disabled{background-color:#C8C8C8;color:#666;}"
        )

        layout = QVBoxLayout(self.wifi_tab)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        status_layout = QHBoxLayout()
        self.label_wifi_status_title = QLabel()
        self.label_wifi_status_title.setFixedWidth(95)
        self.label_wifi_status = QLabel()
        self.label_wifi_status.setFrameShape(QFrame.StyledPanel)
        self.label_wifi_status.setMinimumHeight(28)
        self.label_wifi_status.setStyleSheet("background:#FFFFFF;padding-left:8px;")
        self.pushButton_wifi_scan = QPushButton()
        self.pushButton_wifi_scan.setFixedSize(92, 30)
        self.pushButton_wifi_scan.setStyleSheet(button_style)
        self.pushButton_wifi_scan.pressed.connect(self.scan_wifi_networks)
        status_layout.addWidget(self.label_wifi_status_title)
        status_layout.addWidget(self.label_wifi_status, 1)
        status_layout.addWidget(self.pushButton_wifi_scan)
        layout.addLayout(status_layout)

        self.tableWidget_wifi_networks = QTableWidget(0, 3)
        self.tableWidget_wifi_networks.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableWidget_wifi_networks.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tableWidget_wifi_networks.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget_wifi_networks.verticalHeader().setVisible(False)
        self.tableWidget_wifi_networks.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tableWidget_wifi_networks.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.tableWidget_wifi_networks.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.tableWidget_wifi_networks.setMinimumHeight(112)
        self.tableWidget_wifi_networks.itemSelectionChanged.connect(self.on_wifi_network_selected)
        layout.addWidget(self.tableWidget_wifi_networks)

        form_layout = QGridLayout()
        form_layout.setHorizontalSpacing(10)
        form_layout.setVerticalSpacing(8)
        self.label_wifi_ssid = QLabel()
        self.lineEdit_wifi_ssid = QLineEdit()
        self.lineEdit_wifi_ssid.setMinimumHeight(28)
        self.lineEdit_wifi_ssid.setStyleSheet("background:#FFFFFF;")
        self.label_wifi_password = QLabel()
        self.lineEdit_wifi_password = QLineEdit()
        self.lineEdit_wifi_password.setMinimumHeight(28)
        self.lineEdit_wifi_password.setEchoMode(QLineEdit.Password)
        self.lineEdit_wifi_password.setInputMethodHints(Qt.ImhNoPredictiveText | Qt.ImhPreferLatin)
        self.lineEdit_wifi_password.setStyleSheet("background:#FFFFFF;")
        self.lineEdit_wifi_password.installEventFilter(self)
        self.pushButton_wifi_password_visible = QPushButton()
        self.pushButton_wifi_password_visible.setFixedSize(70, 28)
        self.pushButton_wifi_password_visible.setStyleSheet(button_style)
        self.pushButton_wifi_password_visible.pressed.connect(self.toggle_wifi_password_visible)
        form_layout.addWidget(self.label_wifi_ssid, 0, 0)
        form_layout.addWidget(self.lineEdit_wifi_ssid, 0, 1, 1, 2)
        form_layout.addWidget(self.label_wifi_password, 1, 0)
        form_layout.addWidget(self.lineEdit_wifi_password, 1, 1)
        form_layout.addWidget(self.pushButton_wifi_password_visible, 1, 2)
        layout.addLayout(form_layout)

        self.keyboard_frame = QFrame()
        self.keyboard_frame.setFrameShape(QFrame.StyledPanel)
        self.keyboard_frame.setStyleSheet("QFrame{background:#EFEFEF;border:1px solid #D0D0D0;}")
        keyboard_layout = QGridLayout(self.keyboard_frame)
        keyboard_layout.setContentsMargins(6, 6, 6, 6)
        keyboard_layout.setHorizontalSpacing(4)
        keyboard_layout.setVerticalSpacing(4)
        self.keyboard_buttons = []
        keyboard_rows = ["1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm.-_@"]
        for row, chars in enumerate(keyboard_rows):
            for col, char in enumerate(chars):
                button = QPushButton(char)
                button.setFixedHeight(24)
                button.setStyleSheet(combo_style)
                button.pressed.connect(lambda ch=char: self.insert_wifi_password_text(ch))
                keyboard_layout.addWidget(button, row, col)
                self.keyboard_buttons.append(button)
        actions = [
            ("Space", lambda: self.insert_wifi_password_text(" ")),
            ("Back", self.backspace_wifi_password),
            ("Clear", self.clear_wifi_password),
            ("Hide", self.keyboard_frame.hide),
        ]
        for col, (text, callback) in enumerate(actions):
            button = QPushButton(text)
            button.setFixedHeight(24)
            button.setStyleSheet(button_style)
            button.pressed.connect(callback)
            keyboard_layout.addWidget(button, 4, col * 2, 1, 2)
            self.keyboard_buttons.append(button)
        self.keyboard_frame.hide()
        layout.addWidget(self.keyboard_frame)

        self.load_wifi_config_to_fields()
        self.refresh_wifi_status()

    def eventFilter(self, obj, event):
        if obj is getattr(self, 'lineEdit_wifi_password', None) and event.type() == QEvent.FocusIn:
            self.keyboard_frame.show()
        return super().eventFilter(obj, event)

    def update_wifi_page_language(self):
        if not hasattr(self, 'tabWidget'):
            return
        if self.chinese:
            self.tabWidget.setTabText(0, "系统参数")
            self.tabWidget.setTabText(1, "局域网WiFi")
            self.label_wifi_status_title.setText("无线状态")
            self.pushButton_wifi_scan.setText("扫描")
            self.tableWidget_wifi_networks.setHorizontalHeaderLabels(["热点名称", "信号", "安全"])
            self.label_wifi_ssid.setText("热点名称")
            self.label_wifi_password.setText("WiFi密码")
            self.pushButton_wifi_password_visible.setText("显示")
        else:
            self.tabWidget.setTabText(0, "System")
            self.tabWidget.setTabText(1, "LAN WiFi")
            self.label_wifi_status_title.setText("WLAN")
            self.pushButton_wifi_scan.setText("Scan")
            self.tableWidget_wifi_networks.setHorizontalHeaderLabels(["SSID", "Signal", "Security"])
            self.label_wifi_ssid.setText("SSID")
            self.label_wifi_password.setText("Password")
            self.pushButton_wifi_password_visible.setText("Show")

    def refresh_wifi_status(self):
        if hasattr(self, 'label_wifi_status'):
            self.label_wifi_status.setText(self.get_wlan())

    def load_wifi_config_to_fields(self):
        values = read_wifi_config(wifi_config_path())
        if hasattr(self, 'lineEdit_wifi_ssid'):
            self.lineEdit_wifi_ssid.setText(str(values.get("WIFI_STA_SSID", "")))
        if hasattr(self, 'lineEdit_wifi_password'):
            self.lineEdit_wifi_password.setText(str(values.get("WIFI_STA_PASSWORD", "")))

    def scan_wifi_networks(self):
        self.pushButton_wifi_scan.setEnabled(False)
        self.tableWidget_wifi_networks.setRowCount(0)
        self.label_wifi_status.setText("扫描中..." if self.chinese else "Scanning...")
        self.wifi_scan_thread = WifiScanThread()
        self.wifi_scan_thread.finished_scan.connect(self.on_wifi_scan_finished)
        self.wifi_scan_thread.start()

    def on_wifi_scan_finished(self, networks, error):
        self.pushButton_wifi_scan.setEnabled(True)
        if error:
            self.label_wifi_status.setText(error)
            return
        self.populate_wifi_networks(networks)
        if networks:
            self.label_wifi_status.setText(("扫描到 %d 个热点" if self.chinese else "%d networks found") % len(networks))
        else:
            self.label_wifi_status.setText("未扫描到热点" if self.chinese else "No networks found")

    def populate_wifi_networks(self, networks):
        self.tableWidget_wifi_networks.setRowCount(len(networks))
        for row, network in enumerate(networks):
            self.tableWidget_wifi_networks.setItem(row, 0, QTableWidgetItem(network["ssid"]))
            self.tableWidget_wifi_networks.setItem(row, 1, QTableWidgetItem(str(network["signal"])))
            self.tableWidget_wifi_networks.setItem(row, 2, QTableWidgetItem(network["security"]))

    def on_wifi_network_selected(self):
        items = self.tableWidget_wifi_networks.selectedItems()
        if not items:
            return
        ssid_item = self.tableWidget_wifi_networks.item(items[0].row(), 0)
        if ssid_item is not None:
            self.lineEdit_wifi_ssid.setText(ssid_item.text())

    def toggle_wifi_password_visible(self):
        if self.lineEdit_wifi_password.echoMode() == QLineEdit.Password:
            self.lineEdit_wifi_password.setEchoMode(QLineEdit.Normal)
            self.pushButton_wifi_password_visible.setText("隐藏" if self.chinese else "Hide")
        else:
            self.lineEdit_wifi_password.setEchoMode(QLineEdit.Password)
            self.pushButton_wifi_password_visible.setText("显示" if self.chinese else "Show")

    def insert_wifi_password_text(self, text):
        self.lineEdit_wifi_password.insert(text)
        self.lineEdit_wifi_password.setFocus()

    def backspace_wifi_password(self):
        self.lineEdit_wifi_password.backspace()
        self.lineEdit_wifi_password.setFocus()

    def clear_wifi_password(self):
        self.lineEdit_wifi_password.clear()
        self.lineEdit_wifi_password.setFocus()
        
    def set_window_position(self):
        # 窗口居中
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def change_language(self, name):
        if self.radioButton_zn.isChecked() and name.text() == '中文':
            self.chinese = True
            self.label_depth_camera.setText('深度摄像头')
            self.label_machine.setText('机器类型')
            self.label_asr.setText('语音功能')
            self.label_version.setText('版本')
            self.label_platform1.setText('操作系统')
            self.label_kernel1.setText('内核版本')
            self.label_disk1.setText('磁盘容量')
            self.label_memory1.setText('内存占用')
            self.label_wlan1.setText('无线IP地址')
            self.label_eth1.setText('有线IP地址')
            self.label_wifi.setText('热点名称')
            self.pushButton_save.setText('保存')
            self.pushButton_apply.setText('生效')
            self.pushButton_exit.setText('退出')
            self.label_wlan.setText(self.get_wlan())
            self.label_eth.setText(self.get_eth())
            self.label_mic.setText('麦克风类型')
            self.label_asr_mode.setText('语音模式')
            if hasattr(self, 'label_scene'):
                self.label_scene.setText('场景')
            if hasattr(self, 'label_scene5_arm_role'):
                self.label_scene5_arm_role.setText('Scene5身份')
            self.update_wifi_page_language()
        elif self.radioButton_en.isChecked() and name.text() == 'English':
            self.chinese = False
            self.label_depth_camera.setText('Depth Camera')
            self.label_machine.setText('Machine')
            self.label_asr.setText('ASR')
            self.label_version.setText('Version')
            self.label_platform1.setText('Operating System')
            self.label_kernel1.setText('Kernel Version')
            self.label_disk1.setText('Disk Capacity')
            self.label_memory1.setText('Memory')
            self.label_wlan1.setText('WLAN')
            self.label_eth1.setText('Ethemet')
            self.label_wifi.setText('AP Name')
            self.pushButton_save.setText('Save')
            self.pushButton_apply.setText('Apply')
            self.pushButton_exit.setText('Quit')
            self.label_wlan.setText(self.get_wlan())
            self.label_eth.setText(self.get_eth())
            self.label_mic.setText('Mic Type')
            self.label_asr_mode.setText('ASR Mode')
            if hasattr(self, 'label_scene'):
                self.label_scene.setText('Scene')
            if hasattr(self, 'label_scene5_arm_role'):
                self.label_scene5_arm_role.setText('Scene5 Role')
            self.update_wifi_page_language()

    # 弹窗提示函数
    def message_from(self, string):
        try:
            QMessageBox.about(self, '', string)
        except:
            pass
    
    # 窗口退出
    def closeEvent(self, e):    
        result = QMessageBox.question(self,
                                    "Prompt box",
                                    "quit?",
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)
        if result == QMessageBox.Yes:
            # 退出前先把节点退出
            QWidget.closeEvent(self, e)
        else:
            e.ignore()
   
    def update_globals(self, module):
        if module in sys.modules:
            mdl = importlib.reload(sys.modules[module])
        else:
            mdl = importlib.import_module(module)
        if "__all" in mdl.__dict__:
            names = mdl.__dict__["__all__"]
        else:
            names = [x for x in mdl.__dict__ if not x.startswith("_")]
        globals().update({k: getattr(mdl, k) for k in names})

    def get_hw(self):
        global HW_WIFI_AP_SSID

        # address = "/sys/class/net/eth0/address"
        # if os.path.exists(address):
            # with open(address, 'r') as f:
                # serial_num = f.read().replace('\n', '').replace(':', '').upper()
                # serial_num = serial_num[-8:]
        # else:
        device_serial_number = open("/proc/device-tree/serial-number")
        serial_num = device_serial_number.readlines()[0][-10:-1]

        HW_WIFI_AP_SSID = ''.join(["HW-", serial_num[0:8]])
        if os.path.exists(self.external_config_file_path):
            sys.path.insert(0, self.external_config_file_dir_path)
            self.update_globals(os.path.splitext(self.config_file_name)[0])
        
        return HW_WIFI_AP_SSID

    def get_platform(self):

        with open("/etc/os-release") as f:
            content = f.readlines()

        content = [x.strip().split('=') for x in content]
        os_info = {x[0]:x[1].strip('\"') for x in content if len(x)==2}
        return '{} {}+ ROS2 {}'.format(os_info['NAME'], os_info['VERSION'].replace('(', ' ').replace(')', ' '), 'Humble')
    
    def get_kernel(self):
        result = platform.uname()
        return '{}_{}_{}'.format(result.system, result.release, result.machine)

    def get_memory(self):
        mem = psutil.virtual_memory()
        mem_total = round(mem.total / 1024 / 1024 / 1024, 2)
        mem_free = round(mem.available / 1024 / 1024 / 1024, 2)

        return 'Total:{}G  Free:{}G'.format(mem_total, mem_free)

    def get_disk_space(self):
        disk = psutil.disk_usage('/')
        disk_total = round(disk.total / 1024/ 1024 / 1024, 2)
        disk_free = round(disk.free / 1024 / 1024 / 1024, 2)

        return 'Total:{}G  Free:{}G'.format(disk_total, disk_free)
    
    def get_wlan(self):
        ip = ''
        try:
            info = psutil.net_if_addrs()
        except Exception:
            info = {}
        for k, v in info.items():
            if 'wlan0' in k:
                for i in v:
                    if i[2] is not None:
                        ip = i[1]
                        break
                    else:
                        ip = None

        if ip != '' and ip is not None:
            return ip
        elif ip is None:
            if self.chinese:
                return '未连接无线网络'
            else:
                return 'Not connected to wireless network' 
        else:
            if self.chinese:
                return '未检测到无线网卡'
            else:
                return 'Wireless card not detected'

    def get_eth(self):
        ip = ''
        try:
            info = psutil.net_if_addrs()
        except Exception:
            info = {}
        for k, v in info.items():
            if 'eth' in k:
                for j in v:
                    if j[1] != '127.0.0.1' and j[2] is not None:
                        ip = j[1]
                        break
                if ip != '':
                    break
        if ip != '':
            return ip
        else:
            if self.chinese:
                return '未连接有线网络'
            else:
                return 'Wired network not connected'

    def get_typerc(self):
        data = read_text_file(self.typerc_path)
        exports = parse_exports(data)
        version_parts = export_value(exports, "VERSION", default="|unknown|unknown|").split('|')
        if len(version_parts) >= 3:
            version_date = version_parts[2] if len(version_parts) > 2 else ""
            self.version = version_parts[1] + '  ' + version_date
        else:
            self.version = export_value(exports, "VERSION", default="unknown")
        self.language = export_value(exports, "ASR_LANGUAGE", default="Chinese")
        self.lidar = export_value(exports, "LIDAR_TYPE", default="None")
        self.depth_camera = export_value(exports, "CAMERA_TYPE", "DEPTH_CAMERA_TYPE", default="aurora")
        self.machine = normalize_machine_type(export_value(exports, "MACHINE_TYPE", default="NexArm"))
        self.mic = export_value(exports, "MICROPHONE_TYPE", "MIC_TYPE", default="WonderEchoPro")
        self.asr_mode = export_value(exports, "ASR_MODE", default="offline")
        self.scene = export_value(exports, "CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", default="scene_0")
        self.scene5_arm_role = export_value(exports, "SCENE5_ARM_ROLE", default="A")
    
    def set_typerc(self):
        self.get_typerc()
        data = read_text_file(self.typerc_path)

        depth_camera = self.comboBox_depth_camera.currentText()
        data = replace_or_append_export(data, "CAMERA_TYPE", depth_camera)

        data = remove_export(data, "LIDAR_TYPE")
        data = remove_export(data, "DEPTH_CAMERA_TYPE")
        data = remove_export(data, "SCENE5_ARM_A_PREFIX")
        data = remove_export(data, "SCENE5_ARM_B_PREFIX")
        data = remove_export(data, "SCENE5_CONVEYOR_TOPIC")
        machine = normalize_machine_type(self.comboBox_machine.currentText())
        data = replace_or_append_export(data, "MACHINE_TYPE", machine)

        language = self.comboBox_asr.currentText()
        data = replace_or_append_export(data, "ASR_LANGUAGE", language)
        mic = self.comboBox_mic.currentText()
        data = replace_or_append_export(data, "MICROPHONE_TYPE", mic)

        asr_mode = self.comboBox_asr_mode.currentText()
        data = replace_or_append_export(data, "ASR_MODE", asr_mode)
        scene = self.comboBox_scene.currentData() or self.comboBox_scene.currentText() or "scene_0"
        role = self.comboBox_scene5_arm_role.currentData() or self.comboBox_scene5_arm_role.currentText() or "A"
        data = update_scene_exports(data, scene, role)
        write_text_file(self.typerc_path, data)
        write_scene_yaml_current_scene(scene_yaml_path_for(scene), scene)

    def set_wifi_config(self):
        ssid = validate_wifi_ssid(self.lineEdit_wifi_ssid.text())
        if ssid is None:
            raise ValueError("WiFi SSID is empty")
        password = self.lineEdit_wifi_password.text()
        write_wifi_config_file(wifi_config_path(), ssid, password)

    def is_wifi_tab_active(self):
        return hasattr(self, 'tabWidget') and self.tabWidget.currentWidget() == self.wifi_tab

    def save_configs_for_active_tab(self):
        try:
            self.set_typerc()
            if self.is_wifi_tab_active():
                self.set_wifi_config()
        except Exception as e:
            if self.chinese:
                self.message_from("保存失败: {}".format(e))
            else:
                self.message_from("Save failed: {}".format(e))
            return False
        return True

    def button_clicked(self, name):
        if name == 'save':
            if self.save_configs_for_active_tab():
                self.haved_save = True
                if self.chinese:
                    self.message.tips("保存成功", 500)
                else:
                    self.message.tips("Save Success", 500)
        elif name == 'apply':
            restart_wifi = self.is_wifi_tab_active()
            if self.save_configs_for_active_tab():
                self.haved_save = False
                self.reset_service_thread = ResetServiceThread(restart_wifi=restart_wifi)
                self.reset_service_thread.start()
                if self.chinese:
                    self.message.tips("正在重启服务...", 15000)
                else:
                    self.message.tips("Restart Service...", 15000)
        elif name == 'exit':
            sys.exit(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
