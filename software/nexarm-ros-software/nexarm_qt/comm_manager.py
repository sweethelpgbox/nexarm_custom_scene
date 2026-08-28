import serial
import serial.tools.list_ports
import threading
import time
import socket
import subprocess
import os
import sys
import glob
import struct
from PyQt5.QtCore import QObject, pyqtSignal
from nexarm_qt.constants import *

class CommManager(QObject):
    # Define signals
    connection_status_changed = pyqtSignal(bool, str) # is_connected, message
    log_message_received = pyqtSignal(str)
    hex_log_received = pyqtSignal(str, bytes) # prefix, data
    packet_received = pyqtSignal(int, int, bytes) # id, cmd, data
    
    # Specific signals for easier handling in tabs
    coord_updated = pyqtSignal(int, int, int, float, int, float, list) # x, y, z, pitch, roll, claw, servo_angles
    firmware_version_received = pyqtSignal(str, str) # esp_ver, at32_ver
    servo_offset_received = pyqtSignal(int, int) # id, offset
    
    servo_pid_received = pyqtSignal(int, int, int, int, int) # id, p, i, d, minf
    battery_level_received = pyqtSignal(int)
    wifi_scan_finished = pyqtSignal(list)
    
    # CMD 71/73/75/77/80 responses
    servo_overload_received = pyqtSignal(int, int, int, int) # id, torque, time, thresh
    servo_baud_received = pyqtSignal(int, int) # id, baud
    servo_max_torque_received = pyqtSignal(int, int) # id, torque
    servo_angle_limit_received = pyqtSignal(int, int, int) # id, cw, ccw
    coord_limits_received = pyqtSignal(int, int, int, int, int, int, int, int, int, int, int, int) # xmin,xmax,ymin,ymax,zmin,zmax,pmin,pmax,rmin,rmax,cmin,cmax
    chassis_config_received = pyqtSignal(int, float, float, float, int) # type, wheel_dia, wheel_base, track_width, max_speed
    kinematics_config_received = pyqtSignal(list) # 9 floats
    channel_scan_received = pyqtSignal(list) # [(ap_count, rssi), ...] x 13
    action_edit_status_received = pyqtSignal(int, int, int, int) # mode, recording, playing, count
    sync_teach_status_received = pyqtSignal(int, int, int, int, int) # mode, recording, playing, count, overflow

    def __init__(self):
        super().__init__()
        self.ser = None
        self.sock = None
        self.connection_type = 'serial' # 'serial' or 'wifi'
        self.is_connected = False
        self.stop_threads = False
        self.rx_buffer = bytearray()
        self.tx_lock = threading.Lock()

        # Cache for partial updates
        self.last_p = 0.0
        self.last_r = 0
        self.last_claw = 0.0
        self.last_servos = []
        self.servo_offsets = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        self._ros_warned_cmds = set()
        self._ros_last_battery = None
        self._ros_last_full_state = None
        self._ros_controller_name = "ros_robot_controller"
        self._ros_esp_state = {"channel": 1, "global_acc": 20, "sync_enable": False}
        self._rclpy = None
        self._ros_node = None
        self._ros_executor = None
        self._ros_spin_thread = None
        self._ros_shutdown_flag = False
        self._ros_clients = {}
        self._ros_pubs = {}
        self._ros_types = {}
        self._serial_restore_ros_reception = False
        self._ros_reception_wait_sec = 0.3

    def get_available_ports(self):
        ports = []
        seen = set()

        # 1) Standard pyserial discovery
        try:
            for p in serial.tools.list_ports.comports():
                dev = p.device
                if dev and dev not in seen:
                    ports.append(dev)
                    seen.add(dev)
        except Exception:
            pass

        # 2) Linux fallback: include custom udev device names (e.g. /dev/ttyCH341USB0)
        if os.name != 'nt':
            patterns = [
                "/dev/ttyCH341USB*",
                "/dev/ttyUSB*",
                "/dev/ttyACM*",
            ]
            for pat in patterns:
                for dev in sorted(glob.glob(pat)):
                    if dev not in seen:
                        ports.append(dev)
                        seen.add(dev)

        return ports

    @staticmethod
    def _u8_to_i8(v):
        v = int(v) & 0xFF
        return v - 256 if v > 127 else v

    @staticmethod
    def _u16_to_i16(lo, hi):
        return struct.unpack("<h", bytes([int(lo) & 0xFF, int(hi) & 0xFF]))[0]

    def _emit_action_edit_status(self, values):
        if len(values) < 5:
            return False
        mode = int(values[0]) & 0xFF
        recording = int(values[1]) & 0xFF
        playing = int(values[2]) & 0xFF
        count = (int(values[3]) & 0xFF) | ((int(values[4]) & 0xFF) << 8)
        self.action_edit_status_received.emit(mode, recording, playing, count)
        return True

    def _emit_sync_teach_status(self, values):
        if len(values) < 6:
            return False
        mode = int(values[0]) & 0xFF
        recording = int(values[1]) & 0xFF
        playing = int(values[2]) & 0xFF
        count = (int(values[3]) & 0xFF) | ((int(values[4]) & 0xFF) << 8)
        overflow = int(values[5]) & 0xFF
        self.sync_teach_status_received.emit(mode, recording, playing, count, overflow)
        return True

    def _ros_topic(self, suffix):
        return f"/{self._ros_controller_name}{suffix}"

    def _ros_reception_topic(self):
        controller_name = os.getenv("NEXARM_CONTROLLER_NODE", self._ros_controller_name or "ros_robot_controller")
        return f"/{controller_name}/enable_reception"

    def _ros_reception_has_subscription(self):
        topic = self._ros_reception_topic()
        try:
            output = subprocess.check_output(
                ["ros2", "topic", "info", topic],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
        except Exception:
            return False

        for line in output.splitlines():
            if "Subscription count:" not in line:
                continue
            try:
                return int(line.split(":", 1)[1].strip()) > 0
            except (ValueError, IndexError):
                break
        return "Subscription" in output

    def _publish_ros_reception(self, enable):
        topic = self._ros_reception_topic()
        data = "true" if enable else "false"
        try:
            result = subprocess.run(
                ["ros2", "topic", "pub", "--once", topic, "std_msgs/Bool", f"data: {data}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def _prepare_serial_reception_takeover(self):
        self._serial_restore_ros_reception = False
        if not self._ros_reception_has_subscription():
            return False

        self._publish_ros_reception(False)
        self._serial_restore_ros_reception = True
        time.sleep(self._ros_reception_wait_sec)
        return True

    def _restore_ros_reception_if_needed(self):
        if not self._serial_restore_ros_reception:
            return
        self._serial_restore_ros_reception = False
        self._publish_ros_reception(True)

    def _ros_warn_unsupported(self, key, detail):
        if key in self._ros_warned_cmds:
            return
        self._ros_warned_cmds.add(key)
        self.log_message_received.emit(f"[ROS mode] Unsupported command: {detail}")

    def _ros_call(self, name, request, timeout_sec=1.5):
        client = self._ros_clients.get(name)
        if client is None:
            return None
        try:
            if not client.service_is_ready():
                if not client.wait_for_service(timeout_sec=timeout_sec):
                    return None
            future = client.call_async(request)
            deadline = time.time() + max(0.2, float(timeout_sec))
            while time.time() < deadline and not future.done():
                time.sleep(0.01)
            if not future.done():
                return None
            return future.result()
        except Exception:
            return None

    def _ros_cleanup(self):
        self._ros_shutdown_flag = True
        try:
            if self._ros_executor is not None:
                self._ros_executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            if self._ros_node is not None:
                self._ros_node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
        self._ros_node = None
        self._ros_executor = None
        self._ros_spin_thread = None
        self._ros_clients = {}
        self._ros_pubs = {}
        self._ros_types = {}

    def connect_ros(self, controller_name=None):
        if self.is_connected:
            return

        try:
            # Ensure ROS logging paths are writable across different launch environments.
            ros_home = os.environ.get("ROS_HOME", "/tmp/.ros")
            ros_log_dir = os.environ.get("ROS_LOG_DIR", os.path.join(ros_home, "log"))
            os.makedirs(ros_home, exist_ok=True)
            os.makedirs(ros_log_dir, exist_ok=True)
            os.environ["ROS_HOME"] = ros_home
            os.environ["ROS_LOG_DIR"] = ros_log_dir

            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from std_msgs.msg import UInt16, Int8
            from std_msgs.msg import UInt8MultiArray
            from ros_robot_controller_msgs.msg import ArmFullState
            from ros_robot_controller_msgs.msg import (
                BuzzerState, OLEDState, MotorState, MotorsState, TankState, StepperRun, ArmCoords,
                ArmMoveInc, ArmServoSingle, EspnowState, ServoId, ServoMode, ServoOffset, ServoPID,
                ServoOverload, ServoBaudRate, ServoMaxTorque, ServoAngleLimit, CoordLimits,
                ChassisConfig, KinematicsParam, SyncWriteServos, MacAddress, ServoPosition, ServosPosition,
            )
            from std_msgs.msg import UInt8, Empty
            from geometry_msgs.msg import Twist
            from ros_robot_controller_msgs.srv import (
                BusServoCtrl, GetArmFullState, GetFirmwareVersion, GetServoOffset, GetServoPID,
                GetServoOverload, GetServoBaud, GetServoMaxTorque, GetServoAngleLimit,
                GetCoordLimits, GetChassisConfig, GetKinematicsParam, ScanWifiChannels,
            )

            self._rclpy = rclpy
            self._ros_controller_name = controller_name or os.getenv("NEXARM_CONTROLLER_NODE", "ros_robot_controller")

            if not rclpy.ok():
                rclpy.init(args=None)

            self._ros_node = rclpy.create_node(f"nexarm_qt_bridge_{os.getpid()}")
            self._ros_executor = MultiThreadedExecutor(num_threads=2)
            self._ros_executor.add_node(self._ros_node)
            self._ros_shutdown_flag = False

            self._ros_types = {
                "UInt8": UInt8, "Int8": Int8, "Empty": Empty, "Twist": Twist,
                "BuzzerState": BuzzerState, "OLEDState": OLEDState, "MotorState": MotorState,
                "MotorsState": MotorsState, "TankState": TankState, "StepperRun": StepperRun,
                "ArmCoords": ArmCoords, "ArmMoveInc": ArmMoveInc, "ArmServoSingle": ArmServoSingle,
                "EspnowState": EspnowState, "ServoId": ServoId, "ServoMode": ServoMode,
                "ServoOffset": ServoOffset, "ServoPID": ServoPID, "ServoOverload": ServoOverload,
                "ServoBaudRate": ServoBaudRate, "ServoMaxTorque": ServoMaxTorque,
                "ServoAngleLimit": ServoAngleLimit, "CoordLimits": CoordLimits,
                "ChassisConfig": ChassisConfig, "KinematicsParam": KinematicsParam,
                "SyncWriteServos": SyncWriteServos, "MacAddress": MacAddress,
                "ServoPosition": ServoPosition, "ServosPosition": ServosPosition,
                "UInt8MultiArray": UInt8MultiArray,
                "BusServoCtrlReq": BusServoCtrl.Request, "GetArmFullStateReq": GetArmFullState.Request,
                "GetFirmwareVersionReq": GetFirmwareVersion.Request, "GetServoOffsetReq": GetServoOffset.Request,
                "GetServoPIDReq": GetServoPID.Request, "GetServoOverloadReq": GetServoOverload.Request,
                "GetServoBaudReq": GetServoBaud.Request, "GetServoMaxTorqueReq": GetServoMaxTorque.Request,
                "GetServoAngleLimitReq": GetServoAngleLimit.Request, "GetCoordLimitsReq": GetCoordLimits.Request,
                "GetChassisConfigReq": GetChassisConfig.Request, "GetKinematicsParamReq": GetKinematicsParam.Request,
                "ScanWifiChannelsReq": ScanWifiChannels.Request,
            }

            # Publishers
            pub_specs = {
                "set_buzzer": (self._ros_topic("/set_buzzer"), BuzzerState),
                "set_oled": (self._ros_topic("/set_oled"), OLEDState),
                "chassis_single": (self._ros_topic("/chassis/set_single_motor"), MotorState),
                "chassis_motors": (self._ros_topic("/chassis/set_motors"), MotorsState),
                "chassis_stop": (self._ros_topic("/chassis/stop_motors"), Empty),
                "cmd_vel": (self._ros_topic("/cmd_vel"), Twist),
                "chassis_tank": (self._ros_topic("/chassis/set_tank"), TankState),
                "conveyor_set": (self._ros_topic("/conveyor/set"), Int8),
                "stepper_reset": (self._ros_topic("/stepper/reset"), Empty),
                "stepper_div": (self._ros_topic("/stepper/set_div"), UInt8),
                "stepper_run": (self._ros_topic("/stepper/run"), StepperRun),
                "arm_set_coords": (self._ros_topic("/arm/set_coords"), ArmCoords),
                "arm_move_inc": (self._ros_topic("/arm/move_inc"), ArmMoveInc),
                "arm_servo_single": (self._ros_topic("/arm/servo_single"), ArmServoSingle),
                "bus_servo_set_position": (self._ros_topic("/bus_servo/set_position"), ServosPosition),
                "espnow_set": (self._ros_topic("/espnow/set"), EspnowState),
                "arm_set_acc": (self._ros_topic("/arm/set_move_acc"), UInt8),
                "servo_set_id": (self._ros_topic("/servo/set_id"), ServoId),
                "servo_set_mode": (self._ros_topic("/servo/set_mode"), ServoMode),
                "servo_set_offset": (self._ros_topic("/servo/set_offset"), ServoOffset),
                "servo_set_pid": (self._ros_topic("/servo/set_pid"), ServoPID),
                "servo_set_overload": (self._ros_topic("/servo/set_overload"), ServoOverload),
                "servo_set_baud": (self._ros_topic("/servo/set_baud"), ServoBaudRate),
                "servo_set_max_torque": (self._ros_topic("/servo/set_max_torque"), ServoMaxTorque),
                "servo_set_angle_limit": (self._ros_topic("/servo/set_angle_limit"), ServoAngleLimit),
                "servo_calibrate": (self._ros_topic("/servo/calibrate"), UInt8),
                "servo_torque": (self._ros_topic("/servo/torque"), ServoMode),
                "arm_set_torque": (self._ros_topic("/arm/set_torque"), UInt8),
                "arm_reset": (self._ros_topic("/arm/reset"), Empty),
                "servo_sync_write": (self._ros_topic("/servo/sync_write"), SyncWriteServos),
                "arm_interp_mode": (self._ros_topic("/arm/set_interp_mode"), UInt8),
                "arm_coord_limits": (self._ros_topic("/arm/set_coord_limits"), CoordLimits),
                "chassis_config": (self._ros_topic("/chassis/set_config"), ChassisConfig),
                "arm_kinematics": (self._ros_topic("/arm/set_kinematics"), KinematicsParam),
                "set_bt_mode": (self._ros_topic("/set_bt_mode"), UInt8),
                "set_ps3_mac": (self._ros_topic("/set_ps3_mac"), MacAddress),
                "factory_reset": (self._ros_topic("/factory_reset"), Empty),
                "set_lerobot_mode": (self._ros_topic("/set_lerobot_mode"), UInt8),
                "set_pc_sync_teach": (self._ros_topic("/set_pc_sync_teach"), UInt8),
                "action_edit_enter": (self._ros_topic("/action_edit/enter"), Empty),
                "action_edit_exit": (self._ros_topic("/action_edit/exit"), Empty),
                "action_edit_start": (self._ros_topic("/action_edit/start"), Empty),
                "action_edit_stop": (self._ros_topic("/action_edit/stop"), Empty),
                "action_edit_play": (self._ros_topic("/action_edit/play"), Empty),
                "action_edit_play_stop": (self._ros_topic("/action_edit/play_stop"), Empty),
                "action_edit_clear": (self._ros_topic("/action_edit/clear"), Empty),
                "action_edit_query": (self._ros_topic("/action_edit/query"), Empty),
                "sync_teach_enter": (self._ros_topic("/sync_teach/enter"), Empty),
                "sync_teach_exit": (self._ros_topic("/sync_teach/exit"), Empty),
                "sync_teach_rec_start": (self._ros_topic("/sync_teach/rec_start"), Empty),
                "sync_teach_rec_stop": (self._ros_topic("/sync_teach/rec_stop"), Empty),
                "sync_teach_play": (self._ros_topic("/sync_teach/play"), Empty),
                "sync_teach_play_stop": (self._ros_topic("/sync_teach/play_stop"), Empty),
                "sync_teach_clear": (self._ros_topic("/sync_teach/clear"), Empty),
                "sync_teach_query": (self._ros_topic("/sync_teach/query"), Empty),
                "action_group_run": (self._ros_topic("/action_group/run"), UInt8),
                "action_group_stop": (self._ros_topic("/action_group/stop"), Empty),
                "action_group_erase": (self._ros_topic("/action_group/erase"), UInt8),
                "action_group_download": (self._ros_topic("/action_group/download"), UInt8MultiArray),
            }
            self._ros_pubs = {
                key: self._ros_node.create_publisher(msg_t, topic, 10)
                for key, (topic, msg_t) in pub_specs.items()
            }

            # Subscribers
            def _on_battery(msg):
                try:
                    self._ros_last_battery = int(msg.data)
                except Exception:
                    self._ros_last_battery = None
            self._ros_node.create_subscription(UInt16, self._ros_topic("/battery"), _on_battery, 10)

            def _on_full_state(msg):
                try:
                    servos = [int(v) for v in list(msg.servos)[:6]]
                    joints = [float(v) for v in list(msg.joint_angles)[:6]]
                    self._ros_last_full_state = {
                        "x": float(msg.x), "y": float(msg.y), "z": float(msg.z),
                        "pitch": float(msg.pitch), "roll": float(msg.roll), "claw": float(msg.claw),
                        "servos": servos, "joint_angles": joints,
                    }
                    self.last_p = float(msg.pitch)
                    self.last_r = int(msg.roll)
                    self.last_claw = float(msg.claw)
                    self.last_servos = servos
                    self.coord_updated.emit(int(msg.x), int(msg.y), int(msg.z), self.last_p, self.last_r, self.last_claw, servos)
                except Exception:
                    pass
            self._ros_node.create_subscription(ArmFullState, self._ros_topic("/arm/full_state"), _on_full_state, 10)

            def _on_action_edit_status(msg):
                try:
                    self._emit_action_edit_status(list(msg.data))
                except Exception:
                    pass
            self._ros_node.create_subscription(UInt8MultiArray, self._ros_topic("/action_edit/status"), _on_action_edit_status, 10)

            def _on_sync_teach_status(msg):
                try:
                    self._emit_sync_teach_status(list(msg.data))
                except Exception:
                    pass
            self._ros_node.create_subscription(UInt8MultiArray, self._ros_topic("/sync_teach/status"), _on_sync_teach_status, 10)

            # Service clients
            self._ros_clients = {
                "bus_servo_ctrl": self._ros_node.create_client(BusServoCtrl, self._ros_topic("/bus_servo/ctrl")),
                "get_full_state": self._ros_node.create_client(GetArmFullState, self._ros_topic("/arm/get_full_state")),
                "get_fw": self._ros_node.create_client(GetFirmwareVersion, self._ros_topic("/get_firmware_version")),
                "get_offset": self._ros_node.create_client(GetServoOffset, self._ros_topic("/servo/get_offset")),
                "get_pid": self._ros_node.create_client(GetServoPID, self._ros_topic("/servo/get_pid")),
                "get_overload": self._ros_node.create_client(GetServoOverload, self._ros_topic("/servo/get_overload")),
                "get_baud": self._ros_node.create_client(GetServoBaud, self._ros_topic("/servo/get_baud")),
                "get_max_torque": self._ros_node.create_client(GetServoMaxTorque, self._ros_topic("/servo/get_max_torque")),
                "get_angle_limit": self._ros_node.create_client(GetServoAngleLimit, self._ros_topic("/servo/get_angle_limit")),
                "get_coord_limits": self._ros_node.create_client(GetCoordLimits, self._ros_topic("/arm/get_coord_limits")),
                "get_chassis_config": self._ros_node.create_client(GetChassisConfig, self._ros_topic("/chassis/get_config")),
                "get_kinematics": self._ros_node.create_client(GetKinematicsParam, self._ros_topic("/arm/get_kinematics")),
                "scan_wifi_channels": self._ros_node.create_client(ScanWifiChannels, self._ros_topic("/scan_wifi_channels")),
            }

            def _spin():
                while not self._ros_shutdown_flag and self._rclpy and self._rclpy.ok():
                    try:
                        self._ros_executor.spin_once(timeout_sec=0.1)
                    except Exception:
                        time.sleep(0.05)

            self._ros_spin_thread = threading.Thread(target=_spin, daemon=True)
            self._ros_spin_thread.start()

            # Preflight: verify target controller service exists.
            svc = self._ros_clients.get("get_full_state")
            target_service = self._ros_topic("/arm/get_full_state")
            deadline = time.time() + 6.0
            ready = False
            candidates = []
            while time.time() < deadline and not ready:
                try:
                    if svc is not None and svc.service_is_ready():
                        ready = True
                        break
                    if svc is not None and svc.wait_for_service(timeout_sec=0.4):
                        ready = True
                        break
                except Exception:
                    pass

                try:
                    candidates = []
                    for sname, _stypes in self._ros_node.get_service_names_and_types():
                        if sname.endswith("/arm/get_full_state"):
                            candidates.append(sname)
                    if target_service in candidates:
                        ready = True
                        break
                except Exception:
                    pass

                time.sleep(0.05)

            if not ready:
                hint = ""
                if candidates:
                    cand = candidates[0].replace("/arm/get_full_state", "").lstrip("/")
                    hint = f" Found: {', '.join(candidates[:3])}. Try: export NEXARM_CONTROLLER_NODE={cand}"
                self._ros_cleanup()
                self.connection_type = 'serial'
                self.is_connected = False
                self.connection_status_changed.emit(
                    False,
                    f"ROS service not found: {target_service} (check node name / sourced env).{hint}"
                )
                return

            self.connection_type = 'ros'
            self.is_connected = True
            self.stop_threads = False
            self.connection_status_changed.emit(True, f"Connected via ROS node {self._ros_controller_name}")
            self._read_all_offsets()
        except Exception as e:
            self.connection_type = 'serial'
            self.is_connected = False
            self.connection_status_changed.emit(False, f"ROS connect failed: {e}")

    def connect(self, port_name, baudrate=1000000):
        if self.is_connected:
            return

        restore_ros_on_fail = self._prepare_serial_reception_takeover()
        try:
            self.connection_type = 'serial'
            kwargs = {"timeout": 0.01}
            # Linux 可显式关闭串口独占，减少“设备忙”问题
            if os.name != 'nt':
                kwargs["exclusive"] = False
            self.ser = serial.Serial(port_name, int(baudrate), **kwargs)
            self.is_connected = True
            self.stop_threads = False

            # Start RX thread
            threading.Thread(target=self.rx_loop, daemon=True).start()

            # 连接后自动读取所有舵机偏差
            self._read_all_offsets()

            self.connection_status_changed.emit(True, f"Connected to {port_name} @ {baudrate}")
        except serial.SerialException as e:
            if restore_ros_on_fail:
                self._restore_ros_reception_if_needed()
            self.ser = None
            self.is_connected = False
            msg = f"Serial Error: {e}"
            if os.name != 'nt':
                msg += " (Linux请检查用户是否在dialout组、端口是否被占用)"
            self.connection_status_changed.emit(False, msg)
        except PermissionError:
            if restore_ros_on_fail:
                self._restore_ros_reception_if_needed()
            self.ser = None
            self.is_connected = False
            msg = f"Port {port_name} is busy or no permission"
            if os.name != 'nt':
                msg += " (Linux可执行: sudo usermod -aG dialout $USER 后重新登录)"
            self.connection_status_changed.emit(False, msg)
        except Exception as e:
            if restore_ros_on_fail:
                self._restore_ros_reception_if_needed()
            self.ser = None
            self.is_connected = False
            self.connection_status_changed.emit(False, str(e))

    def connect_wifi(self, ip, port=8080, silent=False):
        if self.is_connected:
            return

        try:
            self.connection_type = 'wifi'
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((ip, int(port)))
            
            self.sock.settimeout(0.01) # Short timeout for loop
            
            self.is_connected = True
            self.stop_threads = False

            threading.Thread(target=self.rx_loop, daemon=True).start()

            # 连接后自动读取所有舵机偏差
            self._read_all_offsets()

            self.connection_status_changed.emit(True, f"Connected to {ip}:{port}")
        except Exception as e:
            self.sock = None
            if not silent:
                self.connection_status_changed.emit(False, str(e))

    def disconnect(self):
        if not self.is_connected:
            return
        
        self.is_connected = False
        self.stop_threads = True
        
        if self.ser:
            try:
                self.ser.close()
            except: pass
            self.ser = None

        if self.connection_type == 'serial':
            self._restore_ros_reception_if_needed()
            
        if self.sock:
            try:
                self.sock.close()
            except: pass
            self.sock = None

        if self.connection_type == 'ros':
            self._ros_cleanup()
            
        self.connection_status_changed.emit(False, "Disconnected")

    def rx_loop(self):
        while self.is_connected and not self.stop_threads:
            try:
                data = b''
                if self.connection_type == 'serial' and self.ser:
                    try:
                        _ = self.ser.in_waiting
                    except (serial.SerialException, OSError):
                        print("Serial port disconnected (cable removed)")
                        self.disconnect()
                        break
                    try:
                        if self.ser.in_waiting:
                            data = self.ser.read(self.ser.in_waiting)
                    except (serial.SerialException, OSError):
                        print("Serial read error")
                        self.disconnect()
                        break
                
                elif self.connection_type == 'wifi' and self.sock:
                    try:
                        chunk = self.sock.recv(1024)
                        if chunk:
                            data = chunk
                        else:
                            # Connection closed
                            self.disconnect()
                            break
                    except socket.timeout:
                        pass
                    except Exception as e:
                        print(f"Socket Error: {e}")
                        self.disconnect()
                        break

                if data:
                    self.rx_buffer.extend(data)
                    
                    # Try to decode text logs
                    # 避免将含有 0x5A (字符 'Z') 或 0xFF 的二进制协议包错误当做文本打印
                    if b'\xff' not in data:
                        try:
                            text = data.decode('utf-8', errors='ignore')
                            clean = "".join([c for c in text if ord(c) >= 32 or c in '\r\n'])
                            if clean.strip() and not all(c in 'Z \r\n' for c in clean):
                                self.log_message_received.emit(clean)
                        except:
                            pass
                    
                    self.process_buffer()
            except Exception as e:
                print(f"RX Error: {e}")
            time.sleep(0.005)

    def process_buffer(self):
        while len(self.rx_buffer) >= 7:
            if self.rx_buffer[0] != 0xFF or self.rx_buffer[1] != 0xFF:
                self.rx_buffer.pop(0)
                continue

            length = self.rx_buffer[3]
            total_len = length + 4  # FF FF + payload

            if len(self.rx_buffer) < total_len:
                return

            packet = self.rx_buffer[:total_len]
            self.rx_buffer = self.rx_buffer[total_len:]

            self.hex_log_received.emit("RX", bytes(packet))

            id_val = packet[2]
            cmd = packet[4]
            data = packet[5:-1] # Exclude checksum

            self.handle_packet(id_val, cmd, data)

    def _decode_servo_offset(self, data):
        if len(data) < 3:
            raise ValueError("offset response too short")
        import struct
        sid = int(data[0])
        raw = struct.unpack('<H', bytes(data[1:3]))[0]
        offset = struct.unpack('<h', bytes(data[1:3]))[0]

        if not (-2047 <= offset <= 2047):
            raw11 = raw & 0x0FFF
            if raw11 & 0x0800:
                offset = -(raw11 & 0x07FF)
            else:
                offset = raw11 & 0x07FF

        if offset < -2047:
            offset = -2047
        elif offset > 2047:
            offset = 2047

        return sid, offset

    def handle_packet(self, id_val, cmd, data):
        self.packet_received.emit(id_val, cmd, bytes(data))
        
        if (id_val == 0xFF or id_val == 0x5A) and cmd in (CMD_GET_CUR_COORDS, CMD_FKINE_RESULT_GET):
            # 只处理AT32回的完整包(>=24字节)，忽略中层的6字节缓存值
            if len(data) >= 24:
                import struct
                try:
                    x, y, z = struct.unpack('<hhh', bytes(data[:6]))
                    raw_p, raw_r, raw_c = struct.unpack('<hhh', bytes(data[6:12]))
                    self.last_p = raw_p / 10.0
                    self.last_r = raw_r
                    self.last_claw = float(raw_c)
                    self.last_servos = list(struct.unpack('<hhhhhh', bytes(data[12:24])))
                    self.coord_updated.emit(x, y, z, self.last_p, self.last_r, self.last_claw, self.last_servos)
                except Exception as e:
                    pass
        
        elif id_val == 0xFF or id_val == AT32_SYS_ID:
            # Debug: 打印所有收到的包原始数据
            debug_cmds = (CMD_GET_POS_OFFSET, CMD_GET_PID_PARAM, 
                          CMD_READ_OVERLOAD, CMD_READ_BAUD, CMD_READ_MAX_TORQUE, 
                          CMD_READ_ANGLE_LIMIT, CMD_GET_COORD_LIMITS)
            if cmd in debug_cmds:
                hex_str = ' '.join(f'{b:02X}' for b in data)
                print(f"[DEBUG] id=0x{id_val:02X} cmd={cmd} len={len(data)} data=[{hex_str}]")
            
            if cmd == CMD_FIRMWARE_VERSION_CHECK and len(data) >= 3:
                try:
                   esp_ver = f"{data[0]}.{data[1]}.{data[2]}"
                   at32_ver = ""
                   if len(data) >= 6:
                       at32_ver = f"{data[3]}.{data[4]}.{data[5]}"
                   self.firmware_version_received.emit(esp_ver, at32_ver)
                except: pass
            
            elif cmd == CMD_CHECK_BAT_LEVEL_CHECK and len(data) >= 2:
                import struct
                try:
                    vol = struct.unpack('<H', bytes(data[:2]))[0]
                    self.battery_level_received.emit(vol)
                except: pass
            
            elif cmd == CMD_GET_POS_OFFSET and len(data) >= 3:
                try:
                    sid, offset = self._decode_servo_offset(data)
                    self.servo_offsets[sid] = offset
                    print(f"[Offset] ID{sid} offset={offset}")
                    self.servo_offset_received.emit(sid, offset)
                except Exception as e:
                    print(f"Offset Parse Error: {e}")
                
            elif cmd == CMD_GET_PID_PARAM:
                try:
                    import struct
                    sid, p, i, d, minf = 0, 0, 0, 0, 0
                    if len(data) >= 6:
                        sid, p, i, d, minf = struct.unpack('<BBBBH', bytes(data[:6]))
                    else:
                        if len(data) > 0: sid = data[0]
                    self.servo_pid_received.emit(sid, p, i, d, minf)
                except: pass

            elif cmd == CMD_READ_OVERLOAD and len(data) >= 3:
                try:
                    sid, torque, t = data[0], data[1], data[2]
                    thresh = data[3] if len(data) >= 4 else 0
                    self.servo_overload_received.emit(sid, torque, t, thresh)
                except: pass

            elif cmd == CMD_READ_BAUD and len(data) >= 2:
                try:
                    self.servo_baud_received.emit(data[0], data[1])
                except: pass

            elif cmd == CMD_READ_MAX_TORQUE and len(data) >= 3:
                try:
                    import struct
                    sid = data[0]
                    torque = struct.unpack('<H', bytes(data[1:3]))[0]
                    self.servo_max_torque_received.emit(sid, torque)
                except: pass

            elif cmd == CMD_READ_ANGLE_LIMIT and len(data) >= 3:
                try:
                    self.servo_angle_limit_received.emit(data[0], data[1], data[2])
                except: pass

            elif cmd == CMD_GET_COORD_LIMITS and len(data) >= 24:
                try:
                    import struct
                    vals = struct.unpack('<hhhhhhhhhhhh', bytes(data[:24]))
                    self.coord_limits_received.emit(*vals)
                except: pass

            elif cmd == CMD_GET_CHASSIS_CONFIG and len(data) >= 18:
                try:
                    import struct
                    ctype = data[0]
                    wheel_dia = struct.unpack('<f', bytes(data[1:5]))[0]
                    wheel_base = struct.unpack('<f', bytes(data[5:9]))[0]
                    track_width = struct.unpack('<f', bytes(data[9:13]))[0]
                    max_speed = data[17]
                    self.chassis_config_received.emit(ctype, wheel_dia, wheel_base, track_width, max_speed)
                except: pass

            elif cmd == CMD_GET_KINEMATICS_PARAM and len(data) >= 36:
                try:
                    import struct
                    params = [struct.unpack('<f', bytes(data[i*4:(i+1)*4]))[0] for i in range(9)]
                    self.kinematics_config_received.emit(params)
                except: pass

            elif cmd == CMD_GET_KINEMATICS_PARAM and len(data) >= 36:
                try:
                    import struct
                    params = [struct.unpack('<f', bytes(data[i*4:(i+1)*4]))[0] for i in range(9)]
                    self.kinematics_config_received.emit(params)
                except: pass

            elif cmd == CMD_ACTION_EDIT_QUERY:
                self._emit_action_edit_status(data)

            elif cmd == CMD_SYNC_TEACH_QUERY:
                self._emit_sync_teach_status(data)

            elif cmd == CMD_SCAN_WIFI_CHANNELS and len(data) >= 26:
                try:
                    import struct
                    results = []
                    for ch in range(13):
                        ap_count = data[ch*2]
                        rssi = struct.unpack('b', bytes([data[ch*2+1]]))[0]  # signed int8
                        results.append((ap_count, rssi))
                    self.channel_scan_received.emit(results)
                except: pass

    def _ros_emit_realtime_joint_packet(self, servos, joint_angles=None):
        try:
            payload = bytearray()
            angles = joint_angles if joint_angles is not None else [0.0] * 6
            for i in range(6):
                pos = int(servos[i]) if i < len(servos) else 0
                ang = float(angles[i]) if i < len(angles) else 0.0
                payload.extend(struct.pack("<hh", int(pos), int(round(ang * 10.0))))
            self.packet_received.emit(0xFF, CMD_GET_REAL_JOINT_ANGLES, bytes(payload))
        except Exception:
            pass

    def _ros_refresh_full_state(self):
        req_cls = self._ros_types.get("GetArmFullStateReq")
        if req_cls is None:
            return None
        res = self._ros_call("get_full_state", req_cls(), timeout_sec=1.0)
        if res is None or not getattr(res, "success", False):
            return None
        state = {
            "x": float(res.x), "y": float(res.y), "z": float(res.z),
            "pitch": float(res.pitch), "roll": float(res.roll), "claw": float(res.claw),
            "servos": [int(v) for v in list(res.servos)[:6]],
            "joint_angles": [float(v) for v in list(res.joint_angles)[:6]],
        }
        self._ros_last_full_state = state
        self.last_p = state["pitch"]
        self.last_r = int(state["roll"])
        self.last_claw = float(state["claw"])
        self.last_servos = list(state["servos"])
        self.coord_updated.emit(int(state["x"]), int(state["y"]), int(state["z"]), self.last_p, self.last_r, self.last_claw, self.last_servos)
        return state

    def _ros_publish(self, key, msg):
        pub = self._ros_pubs.get(key)
        if pub is None:
            return False
        pub.publish(msg)
        return True

    def _send_packet_ros(self, id_val, cmd, args):
        args = [int(a) for a in args]
        try:
            if id_val == 0xFF and cmd not in (SERVO_CMD_WRITE, SERVO_CMD_READ):
                self._send_sys_ros(cmd, args)
                return

            if cmd == SERVO_CMD_WRITE:
                # Torque write: [SERVO_REG_TORQUE, enable]
                if len(args) >= 2 and args[0] == SERVO_REG_TORQUE:
                    msg = self._ros_types["ServoMode"]()
                    msg.id = int(id_val)
                    msg.mode = 1 if int(args[1]) else 0
                    self._ros_publish("servo_torque", msg)
                    return

                # Position write: [SERVO_REG_ACC, acc, posL, posH, 0, 0, spdL, spdH]
                if len(args) >= 8 and args[0] == SERVO_REG_ACC:
                    pos_msg = self._ros_types["ServoPosition"]()
                    pos_msg.id = int(id_val)
                    pos_msg.position = self._u16_to_i16(args[2], args[3])
                    msg = self._ros_types["ServosPosition"]()
                    msg.position = [pos_msg]
                    self._ros_publish("bus_servo_set_position", msg)
                    return

                self._ros_warn_unsupported(f"servo_write_{args[0] if args else 'none'}", f"SERVO_CMD_WRITE args={args}")
                return

            if cmd == SERVO_CMD_READ and len(args) >= 2 and args[0] == SERVO_REG_PRESENT_POS and args[1] == 2:
                req_cls = self._ros_types.get("BusServoCtrlReq")
                if req_cls is None:
                    return
                req = req_cls()
                req.id = int(id_val)
                res = self._ros_call("bus_servo_ctrl", req, timeout_sec=0.6)
                if res is not None and getattr(res, "success", False):
                    data = struct.pack("<h", int(res.current_position))
                    self.packet_received.emit(int(id_val), 0, data)
                return

            self._ros_warn_unsupported(f"packet_{id_val}_{cmd}", f"id={id_val} cmd={cmd} args={args}")
        except Exception as e:
            self.log_message_received.emit(f"[ROS mode] packet route error: {e}")

    def _send_sys_ros(self, cmd, args):
        try:
            T = self._ros_types

            if cmd == CMD_FIRMWARE_VERSION_CHECK:
                req = T["GetFirmwareVersionReq"]()
                res = self._ros_call("get_fw", req, timeout_sec=1.0)
                if res is not None and getattr(res, "success", False):
                    self.firmware_version_received.emit(str(res.esp_version), str(res.at32_version))
                return

            if cmd == CMD_CHECK_BAT_LEVEL_CHECK:
                if self._ros_last_battery is not None:
                    self.battery_level_received.emit(int(self._ros_last_battery))
                return

            if cmd in (CMD_GET_CUR_COORDS,):
                state = self._ros_last_full_state or self._ros_refresh_full_state()
                if state:
                    self.coord_updated.emit(int(state["x"]), int(state["y"]), int(state["z"]), float(state["pitch"]), int(state["roll"]), float(state["claw"]), list(state["servos"]))
                return

            if cmd in (CMD_GET_REAL_JOINT_ANGLES, CMD_READ_ALL_SERVOS):
                state = self._ros_last_full_state or self._ros_refresh_full_state()
                if state:
                    self._ros_emit_realtime_joint_packet(state.get("servos", []), state.get("joint_angles", []))
                return

            if cmd == CMD_COORDINATE_SET and len(args) >= 14:
                p10, x, y, z, r, c, t = struct.unpack("<hhhhhhH", bytes(args[:14]))
                msg = T["ArmCoords"]()
                msg.x = float(x)
                msg.y = float(y)
                msg.z = float(z)
                msg.pitch = float(p10) / 10.0
                msg.roll = float(r)
                msg.claw = float(c)
                msg.time_ms = int(t)
                self._ros_publish("arm_set_coords", msg)
                return

            if cmd == CMD_ARM_MOVE_INC and len(args) >= 14:
                dx, dy, dz, dp10, dr, dc, t = struct.unpack("<hhhhhhH", bytes(args[:14]))
                msg = T["ArmMoveInc"]()
                msg.dx = int(dx)
                msg.dy = int(dy)
                msg.dz = int(dz)
                msg.dpitch = int(round(float(dp10) / 10.0))
                msg.droll = int(dr)
                msg.dclaw = int(dc)
                msg.time_ms = int(t)
                self._ros_publish("arm_move_inc", msg)
                return

            if cmd == CMD_ARM_SERVO_SINGLE and len(args) >= 5:
                sid, pos, t = struct.unpack("<BhH", bytes(args[:5]))
                msg = T["ArmServoSingle"]()
                msg.id = int(sid)
                msg.pos = int(pos)
                msg.time_ms = int(t)
                self._ros_publish("arm_servo_single", msg)
                return

            if cmd == CMD_SET_MOVE_ACC and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("arm_set_acc", msg)
                return

            if cmd == CMD_BUZZER_SET and len(args) >= 12:
                on_ms, off_ms, repeat, freq = struct.unpack("<IIHH", bytes(args[:12]))
                msg = T["BuzzerState"]()
                msg.freq = int(freq)
                msg.on_time = float(on_ms) / 1000.0
                msg.off_time = float(off_ms) / 1000.0
                msg.repeat = int(repeat)
                self._ros_publish("set_buzzer", msg)
                return

            if cmd == CMD_OLED_SET and len(args) >= 2:
                line = int(args[0]) & 0xFF
                n = int(args[1]) & 0xFF
                text_bytes = bytes([int(v) & 0xFF for v in args[2:2 + n]])
                msg = T["OLEDState"]()
                msg.index = line
                msg.text = text_bytes.decode("utf-8", errors="ignore")
                self._ros_publish("set_oled", msg)
                return

            if cmd in (CMD_SET_SINGLE_MOTOR,) and len(args) >= 2:
                msg = T["MotorState"]()
                msg.id = int(args[0]) & 0xFF
                msg.speed = self._u8_to_i8(args[1])
                self._ros_publish("chassis_single", msg)
                return

            if cmd == CMD_SET_MOTOR_SPEED and len(args) >= 4:
                msg = T["MotorsState"]()
                msg.speed1 = self._u8_to_i8(args[0])
                msg.speed2 = self._u8_to_i8(args[1])
                msg.speed3 = self._u8_to_i8(args[2])
                msg.speed4 = self._u8_to_i8(args[3])
                self._ros_publish("chassis_motors", msg)
                return

            if cmd in (CMD_MOTOR_STOP, CMD_STOP_ALL_MOTOR):
                self._ros_publish("chassis_stop", T["Empty"]())
                return

            if cmd in (CMD_MECANUM_RUN, CMD_MECANUM_CONTROL) and len(args) >= 3:
                msg = T["Twist"]()
                msg.linear.x = float(self._u8_to_i8(args[0]))
                msg.linear.y = float(self._u8_to_i8(args[1]))
                msg.angular.z = float(self._u8_to_i8(args[2]))
                self._ros_publish("cmd_vel", msg)
                return

            if cmd in (CMD_TANK_RUN, CMD_TANK_CONTROL) and len(args) >= 2:
                msg = T["TankState"]()
                msg.speed = self._u8_to_i8(args[0])
                msg.turn = self._u8_to_i8(args[1])
                self._ros_publish("chassis_tank", msg)
                return

            if cmd == CMD_CONVEYOR_SET and len(args) >= 1:
                msg = T["Int8"]()
                msg.data = self._u8_to_i8(args[0])
                self._ros_publish("conveyor_set", msg)
                return

            if cmd == CMD_STEPPER_RESET:
                self._ros_publish("stepper_reset", T["Empty"]())
                return

            if cmd in (CMD_STEPPER_SET_DIV, CMD_STEPPER_DIV) and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("stepper_div", msg)
                return

            if cmd == CMD_STEPPER_RUN and len(args) >= 4:
                steps = struct.unpack("<i", bytes(args[:4]))[0]
                msg = T["StepperRun"]()
                msg.steps = int(steps)
                self._ros_publish("stepper_run", msg)
                return

            if cmd in (CMD_SET_ESPNOW_CHANNEL, CMD_SET_GLOBAL_ACC, CMD_ESPNOW_SYNC_CTRL):
                if cmd == CMD_SET_ESPNOW_CHANNEL and len(args) >= 1:
                    self._ros_esp_state["channel"] = int(args[0]) & 0xFF
                elif cmd == CMD_SET_GLOBAL_ACC and len(args) >= 1:
                    self._ros_esp_state["global_acc"] = int(args[0]) & 0xFF
                elif cmd == CMD_ESPNOW_SYNC_CTRL and len(args) >= 1:
                    self._ros_esp_state["sync_enable"] = bool(int(args[0]) & 0xFF)
                msg = T["EspnowState"]()
                msg.channel = int(self._ros_esp_state["channel"])
                msg.global_acc = int(self._ros_esp_state["global_acc"])
                msg.sync_enable = bool(self._ros_esp_state["sync_enable"])
                self._ros_publish("espnow_set", msg)
                return

            if cmd in (CMD_SCAN_WIFI_CHANNELS, CMD_ESPNOW_SCAN_CHANNEL):
                req = T["ScanWifiChannelsReq"]()
                res = self._ros_call("scan_wifi_channels", req, timeout_sec=1.2)
                if res is not None and getattr(res, "success", False):
                    ap = list(res.ap_counts)
                    rs = list(res.rssi_values)
                    n = min(len(ap), len(rs), 13)
                    self.channel_scan_received.emit([(int(ap[i]), int(rs[i])) for i in range(n)])
                return

            if cmd == CMD_SET_SERVO_ID and len(args) >= 2:
                msg = T["ServoId"]()
                msg.old_id = int(args[0]) & 0xFF
                msg.new_id = int(args[1]) & 0xFF
                self._ros_publish("servo_set_id", msg)
                return

            if cmd == CMD_SET_SERVO_MODE and len(args) >= 2:
                msg = T["ServoMode"]()
                msg.id = int(args[0]) & 0xFF
                msg.mode = int(args[1]) & 0xFF
                self._ros_publish("servo_set_mode", msg)
                return

            if cmd == CMD_SET_POS_OFFSET and len(args) >= 3:
                sid, offset = struct.unpack("<Bh", bytes(args[:3]))
                msg = T["ServoOffset"]()
                msg.id = int(sid) & 0xFF
                msg.offset = int(offset)
                self._ros_publish("servo_set_offset", msg)
                return

            if cmd == CMD_GET_POS_OFFSET and len(args) >= 1:
                req = T["GetServoOffsetReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_offset", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    sid = int(res.id)
                    off = int(res.offset)
                    self.servo_offsets[sid] = off
                    self.servo_offset_received.emit(sid, off)
                return

            if cmd == CMD_SET_PID_PARAM and len(args) >= 6:
                sid, p, i, d, minf = struct.unpack("<BBBBH", bytes(args[:6]))
                msg = T["ServoPID"]()
                msg.id = int(sid) & 0xFF
                msg.p = int(p) & 0xFF
                msg.i = int(i) & 0xFF
                msg.d = int(d) & 0xFF
                msg.min_f = int(minf) & 0xFFFF
                self._ros_publish("servo_set_pid", msg)
                return

            if cmd == CMD_GET_PID_PARAM and len(args) >= 1:
                req = T["GetServoPIDReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_pid", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    self.servo_pid_received.emit(int(res.id), int(res.p), int(res.i), int(res.d), int(res.min_f))
                return

            if cmd == CMD_SET_TORQUE and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("arm_set_torque", msg)
                return

            if cmd == CMD_SERVO_CALI_POS and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("servo_calibrate", msg)
                return

            if cmd == CMD_READ_OVERLOAD and len(args) >= 1:
                req = T["GetServoOverloadReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_overload", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    self.servo_overload_received.emit(int(res.id), int(res.torque), int(res.time_val), int(res.thresh))
                return

            if cmd == CMD_SET_OVERLOAD and len(args) >= 4:
                msg = T["ServoOverload"]()
                msg.id = int(args[0]) & 0xFF
                msg.torque = int(args[1]) & 0xFF
                msg.time_val = int(args[2]) & 0xFF
                msg.thresh = int(args[3]) & 0xFF
                self._ros_publish("servo_set_overload", msg)
                return

            if cmd == CMD_READ_BAUD and len(args) >= 1:
                req = T["GetServoBaudReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_baud", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    self.servo_baud_received.emit(int(res.id), int(res.baud))
                return

            if cmd == CMD_SET_BAUD and len(args) >= 2:
                msg = T["ServoBaudRate"]()
                msg.id = int(args[0]) & 0xFF
                msg.baud = int(args[1]) & 0xFF
                self._ros_publish("servo_set_baud", msg)
                return

            if cmd == CMD_READ_MAX_TORQUE and len(args) >= 1:
                req = T["GetServoMaxTorqueReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_max_torque", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    self.servo_max_torque_received.emit(int(res.id), int(res.torque))
                return

            if cmd == CMD_SET_MAX_TORQUE and len(args) >= 3:
                msg = T["ServoMaxTorque"]()
                msg.id = int(args[0]) & 0xFF
                msg.torque = ((int(args[2]) & 0xFF) << 8) | (int(args[1]) & 0xFF)
                self._ros_publish("servo_set_max_torque", msg)
                return

            if cmd == CMD_READ_ANGLE_LIMIT and len(args) >= 1:
                req = T["GetServoAngleLimitReq"]()
                req.id = int(args[0]) & 0xFF
                res = self._ros_call("get_angle_limit", req, timeout_sec=0.8)
                if res is not None and getattr(res, "success", False):
                    self.servo_angle_limit_received.emit(int(res.id), int(res.cw_limit), int(res.ccw_limit))
                return

            if cmd == CMD_SET_ANGLE_LIMIT and len(args) >= 3:
                msg = T["ServoAngleLimit"]()
                msg.id = int(args[0]) & 0xFF
                msg.cw_limit = int(args[1]) & 0xFF
                msg.ccw_limit = int(args[2]) & 0xFF
                self._ros_publish("servo_set_angle_limit", msg)
                return

            if cmd == CMD_ARM_RESET:
                self._ros_publish("arm_reset", T["Empty"]())
                return

            if cmd == CMD_SYNC_WRITE_SERVOS and len(args) >= 14:
                positions = []
                for i in range(6):
                    lo = int(args[i * 2]) & 0xFF
                    hi = int(args[i * 2 + 1]) & 0xFF
                    positions.append(self._u16_to_i16(lo, hi))
                t = (int(args[12]) & 0xFF) | ((int(args[13]) & 0xFF) << 8)
                msg = T["SyncWriteServos"]()
                msg.positions = [int(v) for v in positions]
                msg.time_ms = int(t)
                self._ros_publish("servo_sync_write", msg)
                return

            if cmd == CMD_SET_INTERP_MODE and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("arm_interp_mode", msg)
                return

            if cmd == CMD_SET_COORD_LIMITS and len(args) >= 24:
                vals = struct.unpack("<hhhhhhhhhhhh", bytes(args[:24]))
                msg = T["CoordLimits"]()
                (msg.xmin, msg.xmax, msg.ymin, msg.ymax, msg.zmin, msg.zmax,
                 msg.pmin, msg.pmax, msg.rmin, msg.rmax, msg.cmin, msg.cmax) = [int(v) for v in vals]
                self._ros_publish("arm_coord_limits", msg)
                return

            if cmd == CMD_GET_COORD_LIMITS:
                req = T["GetCoordLimitsReq"]()
                res = self._ros_call("get_coord_limits", req, timeout_sec=1.0)
                if res is not None and getattr(res, "success", False):
                    self.coord_limits_received.emit(
                        int(res.xmin), int(res.xmax), int(res.ymin), int(res.ymax),
                        int(res.zmin), int(res.zmax), int(res.pmin), int(res.pmax),
                        int(res.rmin), int(res.rmax), int(res.cmin), int(res.cmax),
                    )
                return

            if cmd == CMD_SET_CHASSIS_CONFIG and len(args) >= 18:
                msg = T["ChassisConfig"]()
                msg.chassis_type = int(args[0]) & 0xFF
                msg.wheel_dia = struct.unpack("<f", bytes(args[1:5]))[0]
                msg.wheel_base = struct.unpack("<f", bytes(args[5:9]))[0]
                msg.track_width = struct.unpack("<f", bytes(args[9:13]))[0]
                msg.max_speed = int(args[17]) & 0xFF
                self._ros_publish("chassis_config", msg)
                return

            if cmd == CMD_GET_CHASSIS_CONFIG:
                req = T["GetChassisConfigReq"]()
                res = self._ros_call("get_chassis_config", req, timeout_sec=1.0)
                if res is not None and getattr(res, "success", False):
                    self.chassis_config_received.emit(
                        int(res.chassis_type), float(res.wheel_dia), float(res.wheel_base),
                        float(res.track_width), int(res.max_speed),
                    )
                return

            if cmd == CMD_SET_KINEMATICS_PARAM and len(args) >= 36:
                vals = [struct.unpack("<f", bytes(args[i * 4:(i + 1) * 4]))[0] for i in range(9)]
                msg = T["KinematicsParam"]()
                msg.params = [float(v) for v in vals]
                self._ros_publish("arm_kinematics", msg)
                return

            if cmd == CMD_GET_KINEMATICS_PARAM:
                req = T["GetKinematicsParamReq"]()
                res = self._ros_call("get_kinematics", req, timeout_sec=1.0)
                if res is not None and getattr(res, "success", False):
                    self.kinematics_config_received.emit([float(v) for v in list(res.params)])
                return

            if cmd == CMD_SET_BT_MODE and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("set_bt_mode", msg)
                return

            if cmd == CMD_SET_PS3_MAC and len(args) >= 6:
                msg = T["MacAddress"]()
                msg.mac = [int(v) & 0xFF for v in args[:6]]
                self._ros_publish("set_ps3_mac", msg)
                return

            if cmd == CMD_FACTORY_RESET:
                self._ros_publish("factory_reset", T["Empty"]())
                return

            if cmd == CMD_LEROBOT_MODE and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("set_lerobot_mode", msg)
                return

            if cmd == CMD_PC_SYNC_TEACH and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("set_pc_sync_teach", msg)
                return

            teach_empty_topics = {
                CMD_ACTION_EDIT_ENTER: "action_edit_enter",
                CMD_ACTION_EDIT_EXIT: "action_edit_exit",
                CMD_ACTION_EDIT_START: "action_edit_start",
                CMD_ACTION_EDIT_STOP: "action_edit_stop",
                CMD_ACTION_EDIT_PLAY: "action_edit_play",
                CMD_ACTION_EDIT_PLAY_STOP: "action_edit_play_stop",
                CMD_ACTION_EDIT_CLEAR: "action_edit_clear",
                CMD_ACTION_EDIT_QUERY: "action_edit_query",
                CMD_SYNC_TEACH_ENTER: "sync_teach_enter",
                CMD_SYNC_TEACH_EXIT: "sync_teach_exit",
                CMD_SYNC_TEACH_REC_START: "sync_teach_rec_start",
                CMD_SYNC_TEACH_REC_STOP: "sync_teach_rec_stop",
                CMD_SYNC_TEACH_PLAY: "sync_teach_play",
                CMD_SYNC_TEACH_PLAY_STOP: "sync_teach_play_stop",
                CMD_SYNC_TEACH_CLEAR: "sync_teach_clear",
                CMD_SYNC_TEACH_QUERY: "sync_teach_query",
            }
            if cmd in teach_empty_topics:
                self._ros_publish(teach_empty_topics[cmd], T["Empty"]())
                return

            if cmd == CMD_ACTION_GROUP_RUN and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("action_group_run", msg)
                return

            if cmd == CMD_ACTION_GROUP_STOP:
                self._ros_publish("action_group_stop", T["Empty"]())
                return

            if cmd == CMD_ACTION_GROUP_ERASE and len(args) >= 1:
                msg = T["UInt8"]()
                msg.data = int(args[0]) & 0xFF
                self._ros_publish("action_group_erase", msg)
                return

            if cmd == CMD_ACTION_GROUP_DOWNLOAD:
                msg = T["UInt8MultiArray"]()
                msg.data = [int(v) & 0xFF for v in args]
                self._ros_publish("action_group_download", msg)
                return

            # Not mapped in ROS transport: action group / AI / peer-mac, etc.
            self._ros_warn_unsupported(f"sys_{cmd}", f"cmd={cmd}, args={args}")
        except Exception as e:
            self.log_message_received.emit(f"[ROS mode] sys route error cmd={cmd}: {e}")

    def send_packet(self, id_val, cmd, args=[]):
        if not self.is_connected:
            return
        if self.connection_type == 'ros':
            self._send_packet_ros(id_val, cmd, args)
            return
        
        try:
            length = 2 + len(args)
            # Ensure all args are integers
            args = [int(a) for a in args]
            payload = [id_val, length, cmd] + args
            checksum = (~sum(payload)) & 0xFF
            packet = bytes([0xFF, 0xFF] + payload + [checksum])
            
            with self.tx_lock:
                if self.connection_type == 'serial' and self.ser:
                    self.ser.write(packet)
                elif self.connection_type == 'wifi' and self.sock:
                    self.sock.sendall(packet)
            
            self.hex_log_received.emit("TX", packet)
        except Exception as e:
            print(f"TX Error: {e}")

    def send_sys(self, cmd, args=[]):
        if not self.is_connected:
            return
        if self.connection_type == 'ros':
            self._send_sys_ros(cmd, args)
            return
        self.send_packet(0xFF, cmd, args)

    def _read_all_offsets(self):
        """连接后读取所有舵机偏差"""
        import time
        for sid in range(1, 7):
            self.send_sys(CMD_GET_POS_OFFSET, [sid])
            time.sleep(0.05)
        print(f"[Offset] requesting offsets for servo 1-6")

    def scan_for_devices(self):
        def _scan():
            networks = []

            # Linux: use NetworkManager (nmcli)
            if os.name != 'nt':
                try:
                    raw = subprocess.check_output(
                        ["nmcli", "-t", "-f", "SSID", "dev", "wifi", "list", "--rescan", "yes"],
                        stderr=subprocess.STDOUT,
                        timeout=10
                    ).decode("utf-8", errors="ignore")
                    for line in raw.splitlines():
                        ssid = line.strip()
                        if ssid and ssid not in networks:
                            networks.append(ssid)
                except FileNotFoundError:
                    self.log_message_received.emit("[WiFi] nmcli not found. Please install/enable NetworkManager.")
                except subprocess.CalledProcessError as e:
                    err = (e.output or b"").decode("utf-8", errors="ignore")
                    self.log_message_received.emit(f"[WiFi] nmcli scan failed: {err.strip()}")
                except Exception as e:
                    self.log_message_received.emit(f"[WiFi] scan error: {e}")

                self.wifi_scan_finished.emit(networks)
                return

            wlan_ok = False

            # 检查 WlanSvc 服务状态
            try:
                svc_out = subprocess.check_output(
                    "sc query WlanSvc", shell=True,
                    stderr=subprocess.STDOUT, timeout=5
                ).decode('gbk', errors='ignore')
                # print(f"[WiFi] WlanSvc status:\n{svc_out}")
                if ("RUNNING" in svc_out
                        or "运行" in svc_out
                        or "STATE" in svc_out
                        and "4" in svc_out):
                    wlan_ok = True
                else:
                    ret = subprocess.call(
                        "net start WlanSvc", shell=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
                    wlan_ok = (ret == 0)
                    # print(f"[WiFi] net start WlanSvc "
                    #       f"ret={ret} ok={wlan_ok}")
            except Exception:
                pass

            def _try_decode(raw_bytes):
                """多编码尝试解码"""
                for enc in ('utf-8', 'gbk', 'cp936',
                            'cp437', 'latin-1'):
                    try:
                        return raw_bytes.decode(enc)
                    except (UnicodeDecodeError, LookupError):
                        continue
                return raw_bytes.decode('latin-1',
                                        errors='ignore')

            # 检查是否有无线接口
            has_iface = False
            try:
                iface_raw = subprocess.check_output(
                    "netsh wlan show interfaces",
                    shell=True, stderr=subprocess.STDOUT,
                    timeout=5)
                iface_out = _try_decode(iface_raw)
                # print(f"[WiFi] interfaces:\n{iface_out}")
                has_iface = ("GUID" in iface_out
                             or "WLAN" in iface_out
                             or "Wireless" in iface_out
                             or "接口" in iface_out)
                if not has_iface:
                    pass  # no wireless interface
            except subprocess.CalledProcessError:
                pass
            except Exception:
                has_iface = False

            # 方法1: netsh 扫描
            need_location = False
            need_admin = False
            cmds = [
                "netsh wlan show networks mode=bssid",
                "netsh wlan show networks",
            ]
            for cmd in cmds:
                try:
                    raw = subprocess.check_output(
                        cmd, shell=True,
                        stderr=subprocess.STDOUT,
                        timeout=10)
                    output = _try_decode(raw)
                    # print(f"[WiFi] netsh output:\n"
                    #       f"{output[:500]}")

                    for line in output.split('\n'):
                        line = line.strip()
                        if (("SSID" in line
                             or "网络名" in line)
                                and "BSSID" not in line
                                and ":" in line):
                            parts = line.split(":", 1)
                            if len(parts) > 1:
                                ssid = parts[1].strip()
                                if (ssid and
                                        ssid not in networks):
                                    networks.append(ssid)
                    if networks:
                        break
                except subprocess.CalledProcessError as e:
                    err = _try_decode(
                        e.output) if e.output else ""
                    # print(f"[WiFi] netsh FAILED "
                    #       f"(exit {e.returncode}): {err}")
                    if ("位置" in err
                            or "location" in err.lower()):
                        need_location = True
                    if ("管理员" in err or "提升" in err
                            or "admin" in err.lower()
                            or "elevat" in err.lower()):
                        need_admin = True
                    continue
                except Exception as e:
                    # print(f"[WiFi] netsh error: {e}")
                    continue

            # 方法2: pywifi 备用
            if not networks:
                try:
                    import pywifi
                    wifi = pywifi.PyWiFi()
                    ifaces = wifi.interfaces()
                    if not ifaces:
                        pass  # no interface found by pywifi
                    for iface in ifaces:
                        iface.scan()
                        import time; time.sleep(3)
                        for r in iface.scan_results():
                            ssid = r.ssid.strip()
                            if (ssid and
                                    ssid not in networks):
                                networks.append(ssid)
                        if networks:
                            break
                except ImportError:
                    pass  # pywifi not installed
                except Exception as e:
                    pass  # pywifi error

            if not networks:
                diag = []
                if need_location:
                    diag.append(
                        "Windows Location Service disabled"
                        " - open Settings > Privacy > "
                        "Location, turn ON")
                if need_admin:
                    diag.append("Need Run as Administrator")
                if not wlan_ok:
                    diag.append("WlanSvc not running")
                if not has_iface:
                    diag.append("No wireless adapter")
                if not diag:
                    diag.append("Unknown reason")
                msg = ("[WiFi] SCAN FAILED: "
                       + "; ".join(diag))
                # print(msg)
                self.log_message_received.emit(msg)

            self.wifi_scan_finished.emit(networks)

        threading.Thread(target=_scan, daemon=True).start()

    def connect_to_ap_and_socket(self, ssid, password):
        def _connect():
            try:
                if os.name != 'nt':
                    # Linux: try connecting AP via nmcli, then open socket to controller.
                    nmcli_ok = True
                    try:
                        if ssid:
                            if password:
                                subprocess.run(
                                    ["nmcli", "dev", "wifi", "connect", ssid, "password", password],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    check=False
                                )
                            else:
                                subprocess.run(
                                    ["nmcli", "dev", "wifi", "connect", ssid],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    check=False
                                )
                            # DHCP / route stabilize
                            time.sleep(1.5)
                    except FileNotFoundError:
                        nmcli_ok = False
                        self.log_message_received.emit("[WiFi] nmcli not found, skip AP connect and try direct socket.")
                    except Exception as e:
                        self.log_message_received.emit(f"[WiFi] AP connect error: {e}")

                    # Retry socket connect for a few seconds.
                    connected = False
                    for _ in range(15):
                        self.connect_wifi("192.168.4.1", 8080, silent=True)
                        if self.is_connected:
                            connected = True
                            break
                        time.sleep(0.6)

                    if not connected:
                        if not nmcli_ok:
                            self.connection_status_changed.emit(
                                False,
                                "WiFi connect failed: nmcli unavailable and socket to 192.168.4.1:8080 failed"
                            )
                        else:
                            self.connect_wifi("192.168.4.1", 8080, silent=False)
                    return

                # 0. Check if already connected (Avoid re-connect glitch)
                already_connected = False
                try:
                    raw = subprocess.check_output(
                        "netsh wlan show interfaces",
                        shell=True, stderr=subprocess.DEVNULL)
                    output = raw.decode('utf-8', errors='ignore')
                    if ssid not in output:
                        output = raw.decode('gbk', errors='ignore')
                    if ssid in output:
                         already_connected = True
                except Exception: pass

                if not already_connected:
                    # 1. Connect to WiFi AP
                    # Delete existing profile to ensure clean state and avoid conflicts
                    subprocess.run(f'netsh wlan delete profile name="{ssid}"', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                    # SSID hex encoding (required by Win11 for reliable matching)
                    ssid_hex = ssid.encode('utf-8').hex().upper()

                    # Determine security settings
                    if password:
                        auth = "WPA2PSK"
                        encrypt = "AES"
                        key_section = f"""            <sharedKey>
                <keyType>passPhrase</keyType>
                <protected>false</protected>
                <keyMaterial>{password}</keyMaterial>
            </sharedKey>"""
                    else:
                        auth = "open"
                        encrypt = "none"
                        key_section = ""

                    # Generate XML profile (Win10 + Win11 compatible)
                    xml = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid}</name>
    <SSIDConfig>
        <SSID>
            <hex>{ssid_hex}</hex>
            <name>{ssid}</name>
        </SSID>
        <nonBroadcast>false</nonBroadcast>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <MSM>
        <security>
            <authEncryption>
                <authentication>{auth}</authentication>
                <encryption>{encrypt}</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
{key_section}
        </security>
    </MSM>
</WLANProfile>"""
                    
                    # Write profile with BOM for Win11 compatibility
                    with open("temp_wifi.xml", "w", encoding='utf-8-sig') as f:
                        f.write(xml)
                    
                    # user=all needed on Win11 to apply for all users
                    result = subprocess.run(
                        f'netsh wlan add profile filename="temp_wifi.xml" user=all',
                        shell=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
                    if result.returncode != 0:
                        subprocess.run(f'netsh wlan add profile filename="temp_wifi.xml"',
                                       shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    try: os.remove("temp_wifi.xml")
                    except: pass
                    
                    # Connect — specify interface if possible for Win11 reliability
                    result = subprocess.run(
                        f'netsh wlan connect name="{ssid}"',
                        shell=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
                    if result.returncode != 0:
                        pass
                
                # Wait for connection (DHCP takes time)
                # Retry loop (up to 20 seconds)
                connected = False
                for i in range(20):
                    try:
                        # 1. Ping Check (Windows)
                        # returns 0 if success. -n 1 = 1 packet, -w 500 = 500ms timeout
                        ret = subprocess.call("ping -n 1 -w 500 192.168.4.1", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        
                        if ret == 0:
                            # Ping successful, network is up. Now try socket.
                            # We still try/catch because server might be busy
                            time.sleep(0.5) # Give a small moment for stack to settle
                            try:
                                # Silent attempt to avoid spamming UI with "Refused" while Windows probes
                                self.connect_wifi("192.168.4.1", 8080, silent=True)
                                if self.is_connected:
                                    connected = True
                                    break
                            except:
                                time.sleep(1)
                        else:
                            time.sleep(1)
                    except:
                        time.sleep(1)
                
                if not connected:
                    # Try one last time with logging enabled to show the actual error
                    self.connect_wifi("192.168.4.1", 8080, silent=False)
                
            except Exception as e:
                self.connection_status_changed.emit(False, str(e))
        
        threading.Thread(target=_connect, daemon=True).start()
