#!/usr/bin/env python3
import os
import time
import queue
import struct
import serial
import threading

class ESP32Cmd:
    FIRMWARE_VERSION_CHECK = 1
    CHECK_BAT_LEVEL = 2
    ACTION_GROUP_RUN = 3
    ACTION_GROUP_STOP = 4
    ACTION_GROUP_DOWNLOAD = 5
    FKINE_RESULT_GET = 6
    IKINE_RESULT_GET = 7
    COORDINATE_SET = 8
    BUZZER_SET = 9
    OLED_SET = 10
    GET_CUR_COORDS = 11
    OLED_ICON = 12 
    SET_SINGLE_MOTOR = 13
    STOP_ALL_MOTOR = 14
    SET_MOTOR_SPEED = 15
    CONVEYOR_SET = 16
    STEPPER_RESET = 17
    STEPPER_DIV = 18
    STEPPER_RUN = 19
    BUTTON_EVENT = 22
    ACTION_GROUP_ERASE = 23
    SET_ESPNOW_CHANNEL = 30
    SET_GLOBAL_ACC = 31
    ESPNOW_SYNC_CTRL = 33
    MECANUM_CONTROL = 34
    TANK_CONTROL = 35
    SET_PEER_MAC = 36
    ARM_MOVE_INC = 50
    ARM_SERVO_SINGLE = 51
    SET_SERVO_ID = 52
    SET_SERVO_MODE = 53
    READ_ALL_SERVOS = 55
    SET_POS_OFFSET = 57
    GET_POS_OFFSET = 58
    SET_PID_PARAM = 59
    GET_PID_PARAM = 60
    SET_TORQUE = 61
    SET_BT_MODE = 62
    SET_KINEMATICS_PARAM = 63
    GET_KINEMATICS_PARAM = 64
    GET_REAL_JOINT_ANGLES = 65
    GET_REAL_TCP_POSE = 66
    LEROBOT_MODE = 68
    PC_SYNC_TEACH = 69
    SYNC_WRITE_SERVOS = 70
    SERVO_READ_OVERLOAD = 71
    SERVO_WRITE_OVERLOAD = 72
    SERVO_READ_BAUD = 73
    SERVO_WRITE_BAUD = 74
    SERVO_READ_MAX_TORQUE = 75
    SERVO_WRITE_MAX_TORQUE = 76
    SERVO_READ_ANGLE_LIMIT = 77
    SERVO_WRITE_ANGLE_LIMIT = 78
    SET_COORD_LIMITS = 79
    GET_COORD_LIMITS = 80
    MOVE_INC = 82
    SET_PS3_MAC = 83
    FACTORY_RESET = 84
    SET_CHASSIS_CONFIG = 85
    GET_CHASSIS_CONFIG = 86
    SCAN_WIFI_CHANNELS = 87
    SERVO_CALI_POS = 88
    SET_INTERP_MODE = 89
    ACTION_EDIT_ENTER = 120
    ACTION_EDIT_EXIT = 121
    ACTION_EDIT_START = 122
    ACTION_EDIT_STOP = 123
    ACTION_EDIT_PLAY = 124
    ACTION_EDIT_PLAY_STOP = 125
    ACTION_EDIT_CLEAR = 126
    ACTION_EDIT_QUERY = 127
    SYNC_TEACH_ENTER = 128
    SYNC_TEACH_EXIT = 129
    SYNC_TEACH_REC_START = 130
    SYNC_TEACH_REC_STOP = 131
    SYNC_TEACH_PLAY = 132
    SYNC_TEACH_PLAY_STOP = 133
    SYNC_TEACH_CLEAR = 134
    SYNC_TEACH_QUERY = 135

class ServoCmd:
    READ = 2
    WRITE = 3

class ServoReg:
    ID = 5
    MODE = 33
    TORQUE_ENABLE = 40
    ACC = 41
    GOAL_POSITION_L = 42
    PWM_SPEED_L = 44
    GOAL_SPEED_L = 46
    PRESENT_POSITION_L = 56
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63

def checksum_inv8(data):
    return (~sum(data)) & 0xFF

class PacketState:
    START1 = 0
    START2 = 1
    ID = 2
    LENGTH = 3
    CMD = 4
    DATA = 5
    CHECKSUM = 6

class Board:
    def __init__(self, device=None, baudrate=1000000, timeout=0.1):
        device = device or "/dev/rrc"
        self.enable_recv = False
        self.frame =[]
        self.port = serial.Serial(None, baudrate, timeout=timeout)
        self.port.rts = False
        self.port.dtr = False
        self.port.setPort(device)
        self.port.open()
        time.sleep(3.0) 
        self.port.reset_input_buffer() 
        
        # self.state = PacketState.START1
        self.state = 0 
        self.data_len = 0
        self.recv_count = 0
        
        self.sys_queue = queue.Queue(maxsize=10)
        self.arm_queue = queue.Queue(maxsize=10)
        self.key_queue = queue.Queue(maxsize=10)
        self.bus_servo_queue = queue.Queue(maxsize=10)
        self.real_joint_queue = queue.Queue(maxsize=10)
        self.real_tcp_queue = queue.Queue(maxsize=10)
        self.servo_read_lock = threading.Lock()
        self.offset_queue = queue.Queue(maxsize=10)
        self.pid_queue = queue.Queue(maxsize=10)
        self.overload_queue = queue.Queue(maxsize=10)
        self.baud_queue = queue.Queue(maxsize=10)
        self.max_torque_queue = queue.Queue(maxsize=10)
        self.angle_limit_queue = queue.Queue(maxsize=10)
        self.coord_limits_queue = queue.Queue(maxsize=10)
        self.chassis_config_queue = queue.Queue(maxsize=10)
        self.kinematics_queue = queue.Queue(maxsize=10)
        self.firmware_version_queue = queue.Queue(maxsize=10)
        self.wifi_channel_queue = queue.Queue(maxsize=10)
        self.action_edit_queue = queue.Queue(maxsize=10)
        self.sync_teach_queue = queue.Queue(maxsize=10)
        self.request_lock = threading.RLock()
        self.debug = os.environ.get("RRC_SDK_DEBUG", os.environ.get("RRC_DEBUG", "")).lower() in ("1", "true", "yes", "on")
        
        time.sleep(0.1)
        threading.Thread(target=self.recv_task, daemon=True).start()
        
    def set_oled_icon(self, icon_id):
        self.buf_write(0xFF, ESP32Cmd.OLED_ICON, struct.pack('<B', int(icon_id)))

    def _debug(self, message):
        if self.debug:
            print(f"[RRC SDK] {message}", flush=True)

    @staticmethod
    def _hex(data, limit=48):
        values = list(data[:limit])
        suffix = " ..." if len(data) > limit else ""
        return " ".join(f"{int(v) & 0xFF:02X}" for v in values) + suffix

    @staticmethod
    def _decode_pose_servos(data):
        pose = struct.unpack('<hhhhhh', bytes(data[:12]))
        result = {
            "x": pose[0],
            "y": pose[1],
            "z": pose[2],
            "pitch": pose[3] / 10.0,
            "roll": pose[4],
            "claw": pose[5],
            "servos": []
        }
        if len(data) >= 24:
            result["servos"] = list(struct.unpack('<hhhhhh', bytes(data[12:24])))
        return result

    def buf_write(self, target_id, cmd, data=[]):
        length = 2 + len(data)
        buf = [0xFF, 0xFF, target_id, length, cmd] + list(data)
        check_data = buf[2:]
        buf.append(checksum_inv8(check_data))
        with self.request_lock:
            self._debug(f"TX id=0x{target_id:02X} cmd={cmd} len={len(data)} data=[{self._hex(data)}]")
            self.port.write(buf)

    def dispatch_packet(self, pid, cmd, data):
        self._debug(f"RX id=0x{pid:02X} cmd={cmd} len={len(data)} data=[{self._hex(data)}]")
        if pid == 0xFF or pid == 0x5A:
            if cmd == ESP32Cmd.CHECK_BAT_LEVEL:
                try: self.sys_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd in (ESP32Cmd.GET_CUR_COORDS, ESP32Cmd.FKINE_RESULT_GET, ESP32Cmd.IKINE_RESULT_GET):
                try: self.arm_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_REAL_JOINT_ANGLES:
                try: self.real_joint_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_REAL_TCP_POSE:
                try: self.real_tcp_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.BUTTON_EVENT:
                try: self.key_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.FIRMWARE_VERSION_CHECK:
                try: self.firmware_version_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_POS_OFFSET:
                try: self.offset_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_PID_PARAM:
                try: self.pid_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SERVO_READ_OVERLOAD:
                try: self.overload_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SERVO_READ_BAUD:
                try: self.baud_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SERVO_READ_MAX_TORQUE:
                try: self.max_torque_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SERVO_READ_ANGLE_LIMIT:
                try: self.angle_limit_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_COORD_LIMITS:
                try: self.coord_limits_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_CHASSIS_CONFIG:
                try: self.chassis_config_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_KINEMATICS_PARAM:
                try: self.kinematics_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SCAN_WIFI_CHANNELS:
                try: self.wifi_channel_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.ACTION_EDIT_QUERY:
                try: self.action_edit_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.SYNC_TEACH_QUERY:
                try: self.sync_teach_queue.put_nowait(data)
                except queue.Full: pass
        else:
            try: self.bus_servo_queue.put_nowait([cmd] + list(data))
            except queue.Full: pass

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def recv_task(self):
        while True:
            if self.enable_recv and self.port.in_waiting > 0:
                try:
                    data_bytes = self.port.read(self.port.in_waiting)
                    for dat in data_bytes:
                        if self.state == PacketState.START1:
                            if dat == 0xFF: self.state = PacketState.START2
                        elif self.state == PacketState.START2:
                            if dat == 0xFF: self.state = PacketState.ID
                            else: self.state = PacketState.START1
                        elif self.state == PacketState.ID:
                            self.frame = [dat]
                            self.state = PacketState.LENGTH
                        elif self.state == PacketState.LENGTH:
                            self.frame.append(dat)
                            self.data_len = dat - 2
                            if self.data_len < 0 or self.data_len > 250:
                                self.state = PacketState.START1
                            else:
                                self.state = PacketState.CMD
                        elif self.state == PacketState.CMD:
                            self.frame.append(dat)
                            self.recv_count = 0
                            self.state = PacketState.DATA if self.data_len > 0 else PacketState.CHECKSUM
                        elif self.state == PacketState.DATA:
                            self.frame.append(dat)
                            self.recv_count += 1
                            if self.recv_count >= self.data_len:
                                self.state = PacketState.CHECKSUM
                        elif self.state == PacketState.CHECKSUM:
                            if checksum_inv8(self.frame) == dat:
                                self.dispatch_packet(self.frame[0], self.frame[2], self.frame[3:])
                            else:
                                self._debug(
                                    f"RX checksum mismatch frame=[{self._hex(self.frame)}] "
                                    f"got=0x{dat:02X} expected=0x{checksum_inv8(self.frame):02X}"
                                )
                            self.state = PacketState.START1
                except Exception as exc:
                    self._debug(f"recv_task exception: {exc}")
            else:
                time.sleep(0.005)
    
    def set_move_acc(self, acc):
        """
        【新增】：设置底层 10ms 插补算法的起步加速度 
        :param acc: 加速度值 (0~254，0为最快/无缓冲，数值越大缓冲越平滑)
        """
        acc_val = int(acc)
        if acc_val > 254: acc_val = 254
        if acc_val < 0: acc_val = 0
        
        self.buf_write(0xFF, ESP32Cmd.SET_MOVE_ACC, struct.pack('<B', acc_val))
        
    def get_battery(self):
        if not self.enable_recv: return None
        with self.request_lock:
            while not self.sys_queue.empty(): self.sys_queue.get()
            self.buf_write(0xFF, ESP32Cmd.CHECK_BAT_LEVEL)
            try:
                data = self.sys_queue.get(timeout=0.5)
                if len(data) >= 2:
                    return struct.unpack('<H', bytes(data[:2]))[0]
                self._debug(f"battery response too short: len={len(data)}")
            except queue.Empty:
                self._debug("battery response timeout")
                return None

    def get_button(self):
        if not self.enable_recv: return None
        try:
            data = self.key_queue.get_nowait()
            if len(data) >= 2:
                return data[0], data[1]
        except queue.Empty: return None

    def set_buzzer(self, freq, on_time_s, off_time_s, repeat=1):
        on_ms = int(on_time_s * 1000)
        off_ms = int(off_time_s * 1000)
        data = struct.pack("<IIHH", on_ms, off_ms, int(repeat), int(freq))
        self.buf_write(0xFF, ESP32Cmd.BUZZER_SET, data)

    def set_oled_text(self, line, text):
        text_bytes = text.encode('utf-8')[:20]
        data =[int(line), len(text_bytes)] + list(text_bytes)
        self.buf_write(0xFF, ESP32Cmd.OLED_SET, data)

    # ── Action Group ──────────────────────────────────────
    def action_group_run(self, group_id):
        self.buf_write(0xFF, ESP32Cmd.ACTION_GROUP_RUN, [int(group_id) & 0xFF])

    def action_group_stop(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_GROUP_STOP)

    def action_group_erase(self, group_id):
        self.buf_write(0xFF, ESP32Cmd.ACTION_GROUP_ERASE, [int(group_id) & 0xFF])

    def action_group_download_raw(self, payload):
        data = [int(v) & 0xFF for v in payload]
        self.buf_write(0xFF, ESP32Cmd.ACTION_GROUP_DOWNLOAD, data)

    def get_arm_coords(self):
        if not self.enable_recv: return None
        with self.request_lock:
            while not self.arm_queue.empty(): self.arm_queue.get()
            self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
            time.sleep(0.1)
            last_coords = None
            while not self.arm_queue.empty():
                data = self.arm_queue.get()
                if len(data) >= 6:
                    last_coords = struct.unpack('<hhh', bytes(data[:6]))
                else:
                    self._debug(f"coords response too short: len={len(data)}")
            return last_coords

    # def set_arm_coords(self, x, y, z, pitch, roll=0, time_ms=1000):
    #     """设置5轴绝对坐标: X, Y, Z, Pitch(俯仰), Roll(横滚)"""
    #     pitch_val = int(pitch * 10)  # 协议要求放大10倍
    #     # 格式: pitch(h), x(h), y(h), z(h), roll(h), time(h) -> 共12字节
    #     data = struct.pack("<hhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(time_ms))
    #     self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
        
    # def arm_move_inc(self, dx, dy, dz, dpitch, droll=0, time_ms=1000):
    #     """5轴坐标增量控制"""
    #     dpitch_val = int(dpitch * 10)
    #     # 格式: dx, dy, dz, dpitch, droll, time -> 共12字节
    #     data = struct.pack("<hhhhhh", int(dx), int(dy), int(dz), dpitch_val, int(droll), int(time_ms))
    #     self.buf_write(0xFF, ESP32Cmd.ARM_MOVE_INC, data)
    def set_arm_coords(self, x, y, z, pitch, roll=0, claw=0, time_ms=1000):
        pitch_val = int(pitch * 10)
        # 格式: pitch(h), x(h), y(h), z(h), roll(h), claw(h), time(h) -> 14字节
        data = struct.pack("<hhhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
        
    def set_arm_coords(self, x, y, z, pitch, roll=0, claw=0, time_ms=1000, calc_only=False):
        pitch_val = int(pitch * 10)
        
        if calc_only:
            if not self.enable_recv: return None
            with self.request_lock:
                while not self.arm_queue.empty(): self.arm_queue.get()

                data = struct.pack("<hhhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw), int(time_ms))
                self.buf_write(0xFF, ESP32Cmd.IKINE_RESULT_GET, data)

                try:
                    res_data = self.arm_queue.get(timeout=1.0)
                    if len(res_data) >= 24:
                        return self._decode_pose_servos(res_data)
                    if len(res_data) >= 12:
                        return {
                            "x": int(x), "y": int(y), "z": int(z),
                            "pitch": float(pitch), "roll": int(roll), "claw": int(claw),
                            "servos": list(struct.unpack('<hhhhhh', bytes(res_data[:12])))
                        }
                    self._debug(f"IK response too short: len={len(res_data)} data=[{self._hex(res_data)}]")
                except queue.Empty:
                    self._debug("IK response timeout")
                    return None
                return None
            
        else:
            # === 2. 正常运动 ===
            data = struct.pack("<hhhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw), int(time_ms))
            self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
            return True
    
    def get_fk_coords(self, j1, j2, j3, j4, roll=0, claw=0):
        """正运动学计算：传入4个关节角度(度)及roll/claw，计算目标坐标，不运动"""
        if not self.enable_recv: return None
        with self.request_lock:
            while not self.arm_queue.empty(): self.arm_queue.get()

            # AT32 期望 12 字节: j1, j2, j3, j4, roll, claw (关节角为了精度放大10倍)
            p1, p2, p3, p4 = int(j1*10), int(j2*10), int(j3*10), int(j4*10)
            data = struct.pack("<hhhhhh", p1, p2, p3, p4, int(roll), int(claw))
            self.buf_write(0xFF, ESP32Cmd.FKINE_RESULT_GET, data)

            try:
                res_data = self.arm_queue.get(timeout=1.0)
                if len(res_data) >= 12:
                    return self._decode_pose_servos(res_data)
                self._debug(f"FK response too short: len={len(res_data)} data=[{self._hex(res_data)}]")
            except queue.Empty:
                self._debug("FK response timeout")
                return None
            return None
    
    def arm_move_inc(self, dx, dy, dz, dpitch, droll=0, dclaw=0, time_ms=1000):
        dpitch_val = int(dpitch * 10)
        # 格式: dx, dy, dz, dpitch, droll, dclaw, time -> 14字节
        data = struct.pack("<hhhhhhh", int(dx), int(dy), int(dz), dpitch_val, int(droll), int(dclaw), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.ARM_MOVE_INC, data)
    def arm_all_reset(self, time_ms=2000):
        """机械臂所有关节回中/复位"""
        # 对应中位机指令 54
        data = struct.pack("<H", int(time_ms))
        self.buf_write(0xFF, 54, data)
    
    # def get_full_state(self):
    #     """
    #     通过运动学接口读取当前位姿及5个舵机的实时脉冲值
    #     返回: (dict) {坐标信息, 舵机列表}
    #     """
    #     if not self.enable_recv: return None
    #     # 清空队列旧数据
    #     while not self.arm_queue.empty(): self.arm_queue.get()
        
    #     # 发送读取指令 (CMD 11)
    #     self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
        
    #     try:
    #         # 等待底层返回 20 字节数据
    #         data = self.arm_queue.get(timeout=0.5)
    #         if len(data) >= 20:
    #             # 解析前10字节：位姿
    #             pose = struct.unpack('<hhhhh', bytes(data[:10]))
    #             # 解析后10字节：5个舵机脉冲 (ID 1-5)
    #             servos = struct.unpack('<hhhhh', bytes(data[10:20]))
                
    #             return {
    #                 "x": pose[0], "y": pose[1], "z": pose[2], 
    #                 "pitch": pose[3]/10.0, "roll": pose[4],
    #                 "servos": list(servos) # [s1, s2, s3, s4, s5]
    #             }
    #     except queue.Empty:
    #         return None
    #     return None
    def get_full_state(self):
        if not self.enable_recv:
            return None
        with self.request_lock:
            while not self.arm_queue.empty():
                self.arm_queue.get()
            self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
            try:
                data = self.arm_queue.get(timeout=0.5)
                if len(data) >= 24:
                    # 当前底层 GET_CUR_COORDS 返回顺序为:
                    # X, Y, Z, Pitch(x10), Roll, Claw
                    pose = struct.unpack('<hhhhhh', bytes(data[:12]))
                    servos = struct.unpack('<hhhhhh', bytes(data[12:24]))

                    return {
                        "x": pose[0],
                        "y": pose[1],
                        "z": pose[2],
                        "pitch": pose[3] / 10.0,
                        "roll": pose[4],
                        "claw": pose[5],
                        "servos": list(servos)
                    }
                self._debug(f"full state response too short: len={len(data)} data=[{self._hex(data)}]")
            except queue.Empty:
                self._debug("full state response timeout")
                return None
            return None
    
    def get_arm_servos(self):
        """专门用于快速获取6个舵机当前脉冲位置的简易函数"""
        res = self.get_full_state()
        return res["servos"] if res else None

    def get_real_joint_angles(self):
        """
        查询6个关节的真实角度和脉冲值 (基于舵机实际读回位置, 非目标值)
        返回: dict { "joints": [ {"pulse": int, "angle": float}, ... ] }  (6个关节)
              或 None (超时)
        """
        if not self.enable_recv:
            return None
        with self.request_lock:
            while not self.real_joint_queue.empty():
                self.real_joint_queue.get()
            self.buf_write(0xFF, ESP32Cmd.GET_REAL_JOINT_ANGLES)
            try:
                data = self.real_joint_queue.get(timeout=0.5)
                if len(data) >= 24:
                    joints = []
                    for i in range(6):
                        pulse = struct.unpack('<h', bytes(data[i*4:i*4+2]))[0]
                        angle_x10 = struct.unpack('<h', bytes(data[i*4+2:i*4+4]))[0]
                        joints.append({"pulse": pulse, "angle": angle_x10 / 10.0})
                    return {"joints": joints}
                self._debug(f"real joint response too short: len={len(data)} data=[{self._hex(data)}]")
            except queue.Empty:
                self._debug("real joint response timeout")
                return None
            return None

    def get_real_tcp_pose(self):
        """
        查询真实TCP位姿 (基于舵机实际读回位置做FK, 含yaw)
        返回: dict { "x", "y", "z", "yaw", "pitch", "roll", "claw" }
              角度单位: 度, 坐标单位: mm
              或 None (超时)
        """
        if not self.enable_recv:
            return None
        with self.request_lock:
            while not self.real_tcp_queue.empty():
                self.real_tcp_queue.get()
            self.buf_write(0xFF, ESP32Cmd.GET_REAL_TCP_POSE)
            try:
                data = self.real_tcp_queue.get(timeout=0.5)
                if len(data) >= 14:
                    vals = struct.unpack('<hhhhhhh', bytes(data[:14]))
                    return {
                        "x": vals[0],
                        "y": vals[1],
                        "z": vals[2],
                        "yaw": vals[3] / 10.0,
                        "pitch": vals[4] / 10.0,
                        "roll": vals[5] / 10.0,
                        "claw": vals[6] / 10.0
                    }
                self._debug(f"real TCP response too short: len={len(data)} data=[{self._hex(data)}]")
            except queue.Empty:
                self._debug("real TCP response timeout")
                return None
            return None
    
    def arm_move_servo_single(self, servo_id, pos, time_ms=1000):
        data = struct.pack("<BhH", int(servo_id), int(pos), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.ARM_SERVO_SINGLE, data)

    def bus_servo_set_position(self, servo_id, position, acc=0, speed=0):
        data = [ServoReg.ACC, acc] + list(struct.pack("<hHh", int(position), 0, int(speed)))
        self.buf_write(servo_id, ServoCmd.WRITE, data)

    def _bus_servo_read(self, servo_id, reg, length, unpack_format):
        with self.servo_read_lock:
            while not self.bus_servo_queue.empty(): self.bus_servo_queue.get()
            self.buf_write(servo_id, ServoCmd.READ,[reg, length])
            try:
                res = self.bus_servo_queue.get(block=True, timeout=0.2)
                if len(res) >= 1 + length:
                    values = struct.unpack(unpack_format, bytes(res[1:1+length]))
                    return values[0] if len(values) == 1 else values
            except queue.Empty: return None
        return None

    def bus_servo_read_position(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_POSITION_L, 2, '<h')

    def bus_servo_read_voltage(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_VOLTAGE, 1, '<B')

    def bus_servo_read_temperature(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_TEMPERATURE, 1, '<B')

    def bus_servo_read_mode(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.MODE, 1, '<B')

    def set_single_motor(self, motor_idx, speed):
        self.buf_write(0xFF, ESP32Cmd.SET_SINGLE_MOTOR, struct.pack("<Bb", int(motor_idx), int(speed)))

    def set_motor_speed(self, s1, s2, s3, s4):
        self.buf_write(0xFF, ESP32Cmd.SET_MOTOR_SPEED, struct.pack("<bbbb", s1, s2, s3, s4))

    def stop_all_motors(self):
        self.buf_write(0xFF, ESP32Cmd.STOP_ALL_MOTOR)

    def set_mecanum(self, vx, vy, vz):
        self.buf_write(0xFF, ESP32Cmd.MECANUM_CONTROL, struct.pack("<bbb", vx, vy, vz))

    def set_tank(self, speed, turn):
        self.buf_write(0xFF, ESP32Cmd.TANK_CONTROL, struct.pack("<bb", speed, turn))

    def set_conveyor(self, speed):
        self.buf_write(0xFF, ESP32Cmd.CONVEYOR_SET, struct.pack("<b", speed))

    def stepper_reset(self):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_RESET)

    def stepper_set_div(self, code):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_DIV, struct.pack('<B', int(code)))

    def stepper_run(self, steps):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_RUN, struct.pack("<i", int(steps)))

    def espnow_set_channel(self, channel):
        self.buf_write(0xFF, ESP32Cmd.SET_ESPNOW_CHANNEL, struct.pack('<B', channel))
        
    def espnow_set_global_acc(self, acc):
        self.buf_write(0xFF, ESP32Cmd.SET_GLOBAL_ACC, struct.pack('<B', acc))
        
    def espnow_sync_ctrl(self, enable):
        self.buf_write(0xFF, ESP32Cmd.ESPNOW_SYNC_CTRL, struct.pack('<B', 1 if enable else 0))

    # ── Firmware Version ─────────────────────────────────
    def get_firmware_version(self):
        if not self.enable_recv:
            return None
        while not self.firmware_version_queue.empty():
            self.firmware_version_queue.get()
        self.buf_write(0xFF, ESP32Cmd.FIRMWARE_VERSION_CHECK)
        try:
            data = self.firmware_version_queue.get(timeout=0.5)
            if len(data) >= 3:
                esp_ver = f"{data[0]}.{data[1]}.{data[2]}"
                at32_ver = f"{data[3]}.{data[4]}.{data[5]}" if len(data) >= 6 else ""
                return {"esp": esp_ver, "at32": at32_ver}
        except queue.Empty:
            return None
        return None

    # ── Servo Configuration ──────────────────────────────
    def set_servo_id(self, old_id, new_id):
        self.buf_write(0xFF, ESP32Cmd.SET_SERVO_ID, [int(old_id), int(new_id)])

    def set_servo_mode(self, servo_id, mode):
        self.buf_write(0xFF, ESP32Cmd.SET_SERVO_MODE, [int(servo_id), int(mode)])

    def set_torque(self, enable):
        self.buf_write(0xFF, ESP32Cmd.SET_TORQUE, [1 if enable else 0])

    def set_pos_offset(self, servo_id, offset):
        self.buf_write(0xFF, ESP32Cmd.SET_POS_OFFSET, struct.pack('<Bh', int(servo_id), int(offset)))

    def get_pos_offset(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.offset_queue.empty():
            self.offset_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_POS_OFFSET, [int(servo_id)])
        try:
            data = self.offset_queue.get(timeout=0.3)
            if len(data) >= 3:
                sid = data[0]
                offset = struct.unpack('<h', bytes(data[1:3]))[0]
                if not (-2047 <= offset <= 2047):
                    raw = struct.unpack('<H', bytes(data[1:3]))[0] & 0x0FFF
                    offset = -(raw & 0x07FF) if raw & 0x0800 else raw & 0x07FF
                return {"id": sid, "offset": max(-2047, min(2047, offset))}
        except queue.Empty:
            return None
        return None

    def set_pid_param(self, servo_id, p, i, d, min_f):
        self.buf_write(0xFF, ESP32Cmd.SET_PID_PARAM,
                       struct.pack('<BBBBH', int(servo_id), int(p), int(i), int(d), int(min_f)))

    def get_pid_param(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.pid_queue.empty():
            self.pid_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_PID_PARAM, [int(servo_id)])
        try:
            data = self.pid_queue.get(timeout=0.3)
            if len(data) >= 6:
                sid, p, i, d, minf = struct.unpack('<BBBBH', bytes(data[:6]))
                return {"id": sid, "p": p, "i": i, "d": d, "min_f": minf}
        except queue.Empty:
            return None
        return None

    def servo_read_overload(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.overload_queue.empty():
            self.overload_queue.get()
        self.buf_write(0xFF, ESP32Cmd.SERVO_READ_OVERLOAD, [int(servo_id)])
        try:
            data = self.overload_queue.get(timeout=0.3)
            if len(data) >= 3:
                sid, torque, t = data[0], data[1], data[2]
                thresh = data[3] if len(data) >= 4 else 0
                return {"id": sid, "torque": torque, "time": t, "thresh": thresh}
        except queue.Empty:
            return None
        return None

    def servo_write_overload(self, servo_id, torque, time_val, thresh):
        self.buf_write(0xFF, ESP32Cmd.SERVO_WRITE_OVERLOAD,
                       [int(servo_id), int(torque), int(time_val), int(thresh)])

    def servo_read_baud(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.baud_queue.empty():
            self.baud_queue.get()
        self.buf_write(0xFF, ESP32Cmd.SERVO_READ_BAUD, [int(servo_id)])
        try:
            data = self.baud_queue.get(timeout=0.3)
            if len(data) >= 2:
                return {"id": data[0], "baud": data[1]}
        except queue.Empty:
            return None
        return None

    def servo_write_baud(self, servo_id, baud_code):
        self.buf_write(0xFF, ESP32Cmd.SERVO_WRITE_BAUD, [int(servo_id), int(baud_code)])

    def servo_read_max_torque(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.max_torque_queue.empty():
            self.max_torque_queue.get()
        self.buf_write(0xFF, ESP32Cmd.SERVO_READ_MAX_TORQUE, [int(servo_id)])
        try:
            data = self.max_torque_queue.get(timeout=0.3)
            if len(data) >= 3:
                sid = data[0]
                torque = struct.unpack('<H', bytes(data[1:3]))[0]
                return {"id": sid, "torque": torque}
        except queue.Empty:
            return None
        return None

    def servo_write_max_torque(self, servo_id, torque):
        val = int(torque)
        self.buf_write(0xFF, ESP32Cmd.SERVO_WRITE_MAX_TORQUE,
                       [int(servo_id), val & 0xFF, (val >> 8) & 0xFF])

    def servo_read_angle_limit(self, servo_id):
        if not self.enable_recv:
            return None
        while not self.angle_limit_queue.empty():
            self.angle_limit_queue.get()
        self.buf_write(0xFF, ESP32Cmd.SERVO_READ_ANGLE_LIMIT, [int(servo_id)])
        try:
            data = self.angle_limit_queue.get(timeout=0.3)
            if len(data) >= 3:
                return {"id": data[0], "cw": data[1], "ccw": data[2]}
        except queue.Empty:
            return None
        return None

    def servo_write_angle_limit(self, servo_id, cw_limit, ccw_limit):
        self.buf_write(0xFF, ESP32Cmd.SERVO_WRITE_ANGLE_LIMIT,
                       [int(servo_id), int(cw_limit), int(ccw_limit)])

    def servo_cali_pos(self, servo_id):
        self.buf_write(0xFF, ESP32Cmd.SERVO_CALI_POS, [int(servo_id)])

    def sync_write_servos(self, positions, time_ms=1000):
        data = []
        for pos in positions[:6]:
            data += [int(pos) & 0xFF, (int(pos) >> 8) & 0xFF]
        data += [int(time_ms) & 0xFF, (int(time_ms) >> 8) & 0xFF]
        self.buf_write(0xFF, ESP32Cmd.SYNC_WRITE_SERVOS, data)

    def read_all_servos(self):
        self.buf_write(0xFF, ESP32Cmd.READ_ALL_SERVOS)

    def set_interp_mode(self, mode):
        self.buf_write(0xFF, ESP32Cmd.SET_INTERP_MODE, [int(mode)])

    # ── Coordinate Limits ────────────────────────────────
    def set_coord_limits(self, xmin, xmax, ymin, ymax, zmin, zmax,
                         pmin, pmax, rmin, rmax, cmin, cmax):
        data = struct.pack('<hhhhhhhhhhhh',
                           int(xmin), int(xmax), int(ymin), int(ymax),
                           int(zmin), int(zmax), int(pmin), int(pmax),
                           int(rmin), int(rmax), int(cmin), int(cmax))
        self.buf_write(0xFF, ESP32Cmd.SET_COORD_LIMITS, data)

    def get_coord_limits(self):
        if not self.enable_recv:
            return None
        while not self.coord_limits_queue.empty():
            self.coord_limits_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_COORD_LIMITS)
        try:
            data = self.coord_limits_queue.get(timeout=0.5)
            if len(data) >= 24:
                vals = struct.unpack('<hhhhhhhhhhhh', bytes(data[:24]))
                return {
                    "xmin": vals[0], "xmax": vals[1], "ymin": vals[2], "ymax": vals[3],
                    "zmin": vals[4], "zmax": vals[5], "pmin": vals[6], "pmax": vals[7],
                    "rmin": vals[8], "rmax": vals[9], "cmin": vals[10], "cmax": vals[11],
                }
        except queue.Empty:
            return None
        return None

    # ── Chassis Config ───────────────────────────────────
    def set_chassis_config(self, chassis_type, wheel_dia, wheel_base, track_width, max_speed):
        data = [int(chassis_type)]
        data += list(struct.pack('<f', float(wheel_dia)))
        data += list(struct.pack('<f', float(wheel_base)))
        data += list(struct.pack('<f', float(track_width)))
        data += [0, 0, 0, 0]
        data += [int(max_speed)]
        self.buf_write(0xFF, ESP32Cmd.SET_CHASSIS_CONFIG, data)

    def get_chassis_config(self):
        if not self.enable_recv:
            return None
        while not self.chassis_config_queue.empty():
            self.chassis_config_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_CHASSIS_CONFIG)
        try:
            data = self.chassis_config_queue.get(timeout=0.5)
            if len(data) >= 18:
                ctype = data[0]
                wheel_dia = struct.unpack('<f', bytes(data[1:5]))[0]
                wheel_base = struct.unpack('<f', bytes(data[5:9]))[0]
                track_width = struct.unpack('<f', bytes(data[9:13]))[0]
                max_speed = data[17]
                return {
                    "type": ctype, "wheel_dia": wheel_dia,
                    "wheel_base": wheel_base, "track_width": track_width,
                    "max_speed": max_speed,
                }
        except queue.Empty:
            return None
        return None

    # ── Kinematics Params ────────────────────────────────
    def set_kinematics_param(self, params):
        data = struct.pack('<fffffffff', *[float(p) for p in params[:9]])
        self.buf_write(0xFF, ESP32Cmd.SET_KINEMATICS_PARAM, data)

    def get_kinematics_param(self, timeout=0.5):
        if not self.enable_recv:
            return None
        with self.request_lock:
            while not self.kinematics_queue.empty():
                self.kinematics_queue.get()
            self.buf_write(0xFF, ESP32Cmd.GET_KINEMATICS_PARAM)
            try:
                data = self.kinematics_queue.get(timeout=max(float(timeout), 0.0))
                if len(data) >= 36:
                    return [struct.unpack('<f', bytes(data[i*4:(i+1)*4]))[0] for i in range(9)]
                self._debug(f"kinematics response too short: len={len(data)} data=[{self._hex(data)}]")
            except queue.Empty:
                self._debug(f"kinematics response timeout: {float(timeout):.2f}s")
                return None
            return None

    # ── WiFi Channel Scan ────────────────────────────────
    def scan_wifi_channels(self):
        if not self.enable_recv:
            return None
        while not self.wifi_channel_queue.empty():
            self.wifi_channel_queue.get()
        self.buf_write(0xFF, ESP32Cmd.SCAN_WIFI_CHANNELS)
        try:
            data = self.wifi_channel_queue.get(timeout=2.0)
            if len(data) >= 26:
                results = []
                for ch in range(13):
                    ap_count = data[ch * 2]
                    rssi = struct.unpack('b', bytes([data[ch * 2 + 1]]))[0]
                    results.append({"ap_count": ap_count, "rssi": rssi})
                return results
        except queue.Empty:
            return None
        return None

    # ── Teaching / LeRobot ───────────────────────────────
    def set_lerobot_mode(self, mode):
        self.buf_write(0xFF, ESP32Cmd.LEROBOT_MODE, [int(mode)])

    def set_pc_sync_teach(self, mode):
        self.buf_write(0xFF, ESP32Cmd.PC_SYNC_TEACH, [int(mode)])

    def action_edit_enter(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_ENTER)

    def action_edit_exit(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_EXIT)

    def action_edit_start(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_START)

    def action_edit_stop(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_STOP)

    def action_edit_play(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_PLAY)

    def action_edit_play_stop(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_PLAY_STOP)

    def action_edit_clear(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_CLEAR)

    def action_edit_query(self):
        self.buf_write(0xFF, ESP32Cmd.ACTION_EDIT_QUERY)

    def get_action_edit_status(self, timeout=1.0):
        if not self.enable_recv:
            return None
        while not self.action_edit_queue.empty():
            self.action_edit_queue.get()
        self.action_edit_query()
        try:
            data = self.action_edit_queue.get(timeout=max(float(timeout), 0.0))
        except queue.Empty:
            return None
        if len(data) < 5:
            return None
        return {
            "mode": int(data[0]),
            "recording": int(data[1]),
            "playing": int(data[2]),
            "count": int(data[3]) + int(data[4]) * 256,
        }

    def sync_teach_enter(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_ENTER)

    def sync_teach_exit(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_EXIT)

    def sync_teach_rec_start(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_REC_START)

    def sync_teach_rec_stop(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_REC_STOP)

    def sync_teach_play(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_PLAY)

    def sync_teach_play_stop(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_PLAY_STOP)

    def sync_teach_clear(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_CLEAR)

    def sync_teach_query(self):
        self.buf_write(0xFF, ESP32Cmd.SYNC_TEACH_QUERY)

    def get_sync_teach_status(self, timeout=1.0):
        if not self.enable_recv:
            return None
        while not self.sync_teach_queue.empty():
            self.sync_teach_queue.get()
        self.sync_teach_query()
        try:
            data = self.sync_teach_queue.get(timeout=max(float(timeout), 0.0))
        except queue.Empty:
            return None
        if len(data) < 6:
            return None
        return {
            "mode": int(data[0]),
            "recording": int(data[1]),
            "playing": int(data[2]),
            "count": int(data[3]) + int(data[4]) * 256,
            "overflow": int(data[5]),
        }

    # ── Misc ─────────────────────────────────────────────
    def set_bt_mode(self, mode):
        self.buf_write(0xFF, ESP32Cmd.SET_BT_MODE, [int(mode)])

    def set_ps3_mac(self, mac_bytes):
        self.buf_write(0xFF, ESP32Cmd.SET_PS3_MAC, list(mac_bytes[:6]))

    def factory_reset(self):
        self.buf_write(0xFF, ESP32Cmd.FACTORY_RESET)

    def bus_servo_enable_torque(self, servo_id, enable):
        self.buf_write(int(servo_id), ServoCmd.WRITE,
                       [ServoReg.TORQUE_ENABLE, 1 if enable else 0])

    def bus_servo_set_mode(self, servo_id, mode):
        self.buf_write(int(servo_id), ServoCmd.WRITE,
                       [ServoReg.MODE, int(mode)])


# ================== 综合功能测试套件 ==================
def run_robot_comprehensive_test(board, servo_id=1):
    print("\n================== 机械臂硬件与运动学综合测试启动 ==================")
    
    # --- 1. 读取基础信息 ---
    print("\n[1] 系统与状态读取测试")
    vol_mv = board.get_battery()
    if vol_mv is not None:
        print(f" -> 读取电压成功: {vol_mv/1000.0:.2f} V")
    else:
        print(" -> [警告] 电压读取超时，请检查固件通讯。")

    coords = board.get_arm_coords()
    if coords is not None:
        print(f" -> 初始全局坐标读取成功: X={coords[0]}, Y={coords[1]}, Z={coords[2]}")
    else:
        print(" -> [警告] 初始全局坐标读取超时。")

    # --- 2. 运动学坐标控制测试 ---
    print("\n[2] 运动学坐标控制 (逆解) 测试")
    print(" -> 绝对坐标指令: 移动到 (0, 150, 150), Pitch=0度，耗时 2000ms ...")
    board.set_arm_coords(x=150, y=0, z=150, pitch=0, time_ms=1000)
    time.sleep(2.5)

    print(" -> 增量坐标指令: Z轴上升 30mm, 俯仰角增加 5度，耗时 1500ms ...")
    board.arm_move_inc(dx=0, dy=0, dz=30, dpitch=5, time_ms=1500)
    time.sleep(2.0)
    
    # 获取移动后的新坐标
    coords_new = board.get_arm_coords()
    if coords_new:
        print(f" -> 移动后坐标为: X={coords_new[0]}, Y={coords_new[1]}, Z={coords_new[2]}")

    # --- 3.  ---
    print("\n[3] 舵机高级运动控制与角度读取")
    print(f" -> 控制舵机 {servo_id} 运动到位置 800 (加加速度=50, 速度=1200)...")
    board.bus_servo_set_position(servo_id, position=800, acc=50, speed=1200)
    time.sleep(2.0)
    
    print(f" -> 请求读取舵机 {servo_id} 的实时返回角度...")
    pos = board.bus_servo_read_position(servo_id)
    if pos is not None:
        print(f" -> [成功] 返回的舵机角度: {pos}")
    else:
        print(" -> [警告] 获取舵机角度超时。（注：目前的ESP32固件在C++层面可能未将舵机透传数据推送到USB，SDK端已完美适配并开放该函数）")
        
    # 读取温度和电压
    temp = board.bus_servo_read_temperature(servo_id)
    if temp is not None:
        print(f" -> [成功] 返回的舵机温度: {temp} °C")
        
    # 舵机回中
    board.bus_servo_set_position(servo_id, position=0, acc=100, speed=2500)
    time.sleep(1.0)

    # --- 4. 传送带测试 ---
    print("\n[4] 传送带测试")
    print(" -> 正转 (速度50)...")
    board.set_conveyor(50)
    time.sleep(1.5)
    print(" -> 停止传送带...")
    board.set_conveyor(0)
    time.sleep(0.5)

    # --- 5. 步进电机(滑杆)测试 ---
    print("\n[5] 步进电机(滑杆)测试")
    print(" -> 步进电机复位...")
    board.stepper_reset()
    time.sleep(0.5)
    print(" -> 运行正向 2000 步...")
    board.stepper_run(2000)
    time.sleep(2.0)

    # --- 6. 多路/单路电机控制测试 ---
    print("\n[6] 驱动电机控制测试")
    print(" -> 单路驱动 1 号电机 (速度 60)...")
    board.set_single_motor(1, 60)
    time.sleep(1.5)
    print(" -> 一键停止所有电机...")
    board.stop_all_motors()
    time.sleep(0.5)

    # --- 7. 底盘综合测试 (履带与麦轮) ---
    print("\n[7] 底盘协议层级测试")
    print(" -> 麦克纳姆轮底盘: 侧向平移 (vx=50, vy=0, vz=0)...")
    board.set_mecanum(50, 0, 0)
    time.sleep(1.5)
    print(" -> 停止麦轮...")
    board.set_mecanum(0, 0, 0)
    time.sleep(0.5)
    
    print(" -> 履带底盘: 原地转向 (转速 40)...")
    board.set_tank(0, 40)
    time.sleep(1.5)
    print(" -> 停止履带...")
    board.set_tank(0, 0)
    time.sleep(0.5)

    print("\n================== 测试完毕 ==================")


if __name__ == "__main__":
    board = Board(device="/dev/rrc") 
    board.enable_reception(True)
    
    print("程序已启动，正在等待串口初始化...")
    time.sleep(1)
    # board.set_move_acc(20)  #
    
    # 用一声蜂鸣声提示准备就绪
    board.set_buzzer(freq=2500, on_time_s=0.2, off_time_s=0.0, repeat=1)
    board.set_arm_coords(x=200, y=0, z=200, pitch=0, time_ms=500)
    time.sleep(0.5)
    board.set_arm_coords(x=250, y=0, z=200, pitch=0, time_ms=500)
    time.sleep(0.5)
    # 运行全面的测试流程
    # run_robot_comprehensive_test(board, servo_id=1)

    print("\n进入挂起等待状态，可用 Ctrl+C 安全退出程序...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序退出。")
