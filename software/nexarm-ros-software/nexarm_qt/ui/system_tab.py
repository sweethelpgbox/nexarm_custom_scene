from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QPushButton,
    QLabel, QSpinBox, QGridLayout, QSlider, QComboBox, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, QSettings
from PyQt5 import uic
import struct
import os
from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S


UI_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'ui_files', 'system_tab.ui')

class SystemTab(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        
        # Initialize storage for manual widgets (fallback)
        self.buz_vars = []
        self.buz_labels_widgets = []
        
        self.setup_ui()
        self.connect_signals()
        self._connect_fw_signals()

    def setup_ui(self):
        if os.path.exists(UI_PATH):
            try:
                uic.loadUi(UI_PATH, self)
                self.setup_ui_from_file()
                print(f"Loaded UI from {UI_PATH}")
            except Exception as e:
                print(f"Error loading UI file: {e}")
                self.setup_ui_manual()
        else:
            self.setup_ui_manual()

    def setup_ui_from_file(self):
        # Bind System Info Group
        if hasattr(self, 'btn_ver'):
            self.btn_ver.clicked.connect(lambda: self.comm_manager.send_sys(CMD_FIRMWARE_VERSION_CHECK))
        
        if hasattr(self, 'btn_bat'):
            self.btn_bat.clicked.connect(lambda: self.comm_manager.send_sys(CMD_CHECK_BAT_LEVEL_CHECK))
            
        # Bind Buzzer Group
        # Expecting spinboxes: spin_buz_on, spin_buz_off, spin_buz_count, spin_buz_freq
        self.buz_vars = []
        # We need to map the file widgets to the list expected by set_buzzer
        # Order: On, Off, Count, Freq
        expected_spins = ['spin_buz_on', 'spin_buz_off', 'spin_buz_count', 'spin_buz_freq']
        for name in expected_spins:
            if hasattr(self, name):
                self.buz_vars.append(getattr(self, name))
            else:
                # If a specific spinbox is missing, we append None or handle it gracefully
                # For safety in set_buzzer, we should probably check if it's None
                self.buz_vars.append(None)
                
        if hasattr(self, 'btn_buz'):
            self.btn_buz.clicked.connect(self.set_buzzer)

    def setup_ui_manual(self):
        from PyQt5.QtWidgets import QScrollArea
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(4)
        
        # System Info
        self.grp_sys = QGroupBox(STRINGS[self.lang]["grp_sys_params"])
        sys_layout = QHBoxLayout(self.grp_sys)
        
        self.btn_ver = QPushButton(STRINGS[self.lang]["btn_read_ver"])
        self.btn_ver.clicked.connect(self._read_firmware_versions)
        sys_layout.addWidget(self.btn_ver)
        
        self.btn_bat = QPushButton(STRINGS[self.lang]["btn_read_bat"])
        self.btn_bat.clicked.connect(lambda: self.comm_manager.send_sys(CMD_CHECK_BAT_LEVEL_CHECK))
        sys_layout.addWidget(self.btn_bat)
        
        self.btn_ps3 = QPushButton(STRINGS[self.lang].get("btn_switch_ps3", "切换到PS3模式"))
        self.btn_ps3.clicked.connect(lambda: self.comm_manager.send_sys(CMD_SET_BT_MODE, [1]))
        sys_layout.addWidget(self.btn_ps3)
        
        self.btn_ble = QPushButton(STRINGS[self.lang].get("btn_switch_ble", "切换回BLE模式"))
        self.btn_ble.clicked.connect(lambda: self.comm_manager.send_sys(CMD_SET_BT_MODE, [0]))
        sys_layout.addWidget(self.btn_ble)
        
        self.btn_factory_reset = QPushButton(STRINGS[self.lang].get("btn_factory_reset", "恢复出厂设置"))
        self.btn_factory_reset.setStyleSheet("background-color: #D32F2F; color: white;")
        self.btn_factory_reset.clicked.connect(lambda: self.comm_manager.send_sys(CMD_FACTORY_RESET))
        sys_layout.addWidget(self.btn_factory_reset)
        
        sys_layout.addSpacing(15)
        
        self.lbl_firmware = QLabel(STRINGS[self.lang].get("lbl_firmware_esp", "ESP32固件") + ":  --")
        self.lbl_firmware.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 10pt; margin-left: 10px;")
        sys_layout.addWidget(self.lbl_firmware)
        
        self.lbl_firmware_at32 = QLabel(STRINGS[self.lang].get("lbl_firmware_at32", "AT32固件") + ":  --")
        self.lbl_firmware_at32.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 10pt; margin-left: 10px;")
        sys_layout.addWidget(self.lbl_firmware_at32)
        
        sys_layout.addSpacing(15)
        
        self.lbl_battery = QLabel(STRINGS[self.lang].get("lbl_battery", "电压") + ":  --")
        self.lbl_battery.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 10pt; margin-left: 10px;")
        sys_layout.addWidget(self.lbl_battery)
        
        sys_layout.addStretch()
        
        layout.addWidget(self.grp_sys)

        # Global Motion Acceleration
        self.grp_move_acc = QGroupBox(STRINGS[self.lang].get("grp_move_acc", "Global Motion Acceleration"))
        acc_layout = QHBoxLayout(self.grp_move_acc)
        
        self.lbl_acc_val = QLabel(STRINGS[self.lang].get("lbl_acc_val", "Acceleration Value (0-254):"))
        acc_layout.addWidget(self.lbl_acc_val)
        
        self.slider_move_acc = QSlider(Qt.Horizontal)
        self.slider_move_acc.setRange(1, 254)
        acc_layout.addWidget(self.slider_move_acc)
        
        self.spin_move_acc = QSpinBox()
        self.spin_move_acc.setRange(1, 254)
        acc_layout.addWidget(self.spin_move_acc)
        
        # 读取保存的加速度值，默认254
        self._acc_settings = QSettings("NexArm", "NexArmQt")
        saved_acc = self._acc_settings.value("move_acc", 254, type=int)
        self.slider_move_acc.setValue(saved_acc)
        self.spin_move_acc.setValue(saved_acc)
        
        # Connect slider and spinbox
        self.slider_move_acc.valueChanged.connect(self.spin_move_acc.setValue)
        self.spin_move_acc.valueChanged.connect(self.slider_move_acc.setValue)
        
        self.btn_set_acc = QPushButton(STRINGS[self.lang].get("btn_set_acc", "Set/Apply"))
        self.btn_set_acc.clicked.connect(self.set_move_acc)
        acc_layout.addWidget(self.btn_set_acc)
        
        layout.addWidget(self.grp_move_acc)

        # Buzzer
        self.grp_buz = QGroupBox(STRINGS[self.lang]["grp_buzzer"])
        buz_layout = QHBoxLayout(self.grp_buz)
        
        self.buz_vars = []
        self.buz_labels_widgets = []
        labels = ["lbl_buz_on", "lbl_buz_off", "lbl_buz_count", "lbl_buz_freq"]
        vals = [100, 100, 2, 2000]
        
        for key, v in zip(labels, vals):
            lbl = QLabel(STRINGS[self.lang][key])
            lbl.setProperty("class", "aux-text")
            self.buz_labels_widgets.append(lbl)
            buz_layout.addWidget(lbl)
            
            sp = QSpinBox()
            sp.setRange(0, 10000)
            sp.setValue(v)
            self.buz_vars.append(sp)
            buz_layout.addWidget(sp)
            
        self.btn_buz = QPushButton(STRINGS[self.lang]["btn_buz_send"])
        self.btn_buz.clicked.connect(self.set_buzzer)
        buz_layout.addWidget(self.btn_buz)
        
        layout.addWidget(self.grp_buz)

        # App及手柄限位 (CMD 79/80)
        self.grp_coord_limits = QGroupBox(STRINGS[self.lang]["grp_coord_limits"])
        self._build_coord_limits()
        layout.addWidget(self.grp_coord_limits)

        # 底盘参数设置 (CMD 85/86)
        self.grp_chassis = QGroupBox(STRINGS[self.lang].get("grp_chassis_config", "底盘参数设置"))
        self._build_chassis_config()
        layout.addWidget(self.grp_chassis)

        # 运动学参数设置 (CMD 63/64)
        self.grp_kinematics = QGroupBox(STRINGS[self.lang].get("grp_kinematics", "运动学参数设置"))
        self._build_kinematics_config()
        layout.addWidget(self.grp_kinematics)

        layout.addStretch()
        scroll.setWidget(container)
        outer.addWidget(scroll)

    def _connect_fw_signals(self):
        pass

    def update_language(self, lang):
        self.lang = lang
        
        # Only update if manual UI elements exist or if we found them in the file and they have the same names
        # Manual mode uses specific attribute names for groups
        if hasattr(self, 'grp_sys'):
            self.grp_sys.setTitle(STRINGS[lang]["grp_sys_params"])
        if hasattr(self, 'btn_ver'):
            self.btn_ver.setText(STRINGS[lang]["btn_read_ver"])
        if hasattr(self, 'btn_bat'):
            self.btn_bat.setText(STRINGS[lang]["btn_read_bat"])
        if hasattr(self, 'btn_factory_reset'):
            self.btn_factory_reset.setText(STRINGS[lang].get("btn_factory_reset", "恢复出厂设置"))
        if hasattr(self, 'btn_ps3'):
            self.btn_ps3.setText(STRINGS[lang].get("btn_switch_ps3", "Switch to PS3"))
        if hasattr(self, 'btn_ble'):
            self.btn_ble.setText(STRINGS[lang].get("btn_switch_ble", "Switch to BLE"))
        if hasattr(self, 'lbl_firmware'):
            self.lbl_firmware.setText(STRINGS[lang].get("lbl_firmware_esp", "ESP32 FW") + ": --")
        if hasattr(self, 'lbl_firmware_at32'):
            self.lbl_firmware_at32.setText(STRINGS[lang].get("lbl_firmware_at32", "AT32 FW") + ": --")
        if hasattr(self, 'lbl_battery'):
            self.lbl_battery.setText(STRINGS[lang].get("lbl_battery", "Voltage") + ": --")
        
        if hasattr(self, 'grp_buz'):
            self.grp_buz.setTitle(STRINGS[lang]["grp_buzzer"])
            
        if hasattr(self, 'grp_move_acc'):
            self.grp_move_acc.setTitle(STRINGS[lang].get("grp_move_acc", "Global Motion Acceleration"))
        if hasattr(self, 'lbl_acc_val'):
            self.lbl_acc_val.setText(STRINGS[lang].get("lbl_acc_val", "Acceleration Value (0-254):"))
        if hasattr(self, 'btn_set_acc'):
            self.btn_set_acc.setText(STRINGS[lang].get("btn_set_acc", "Set/Apply"))
        
        # Update manual labels if they exist
        labels = ["lbl_buz_on", "lbl_buz_off", "lbl_buz_count", "lbl_buz_freq"]
        if hasattr(self, 'buz_labels_widgets') and self.buz_labels_widgets:
             for lbl, key in zip(self.buz_labels_widgets, labels):
                lbl.setText(STRINGS[lang][key])
            
        if hasattr(self, 'btn_buz'):
            self.btn_buz.setText(STRINGS[lang]["btn_buz_send"])

        if hasattr(self, 'grp_coord_limits'):
            self.grp_coord_limits.setTitle(STRINGS[lang]["grp_coord_limits"])
        if hasattr(self, 'btn_read_cl'):
            self.btn_read_cl.setText(STRINGS[lang]["btn_read_coord_limits"])
        if hasattr(self, 'btn_set_cl'):
            self.btn_set_cl.setText(STRINGS[lang]["btn_set_coord_limits"])

        if hasattr(self, 'grp_chassis'):
            self.grp_chassis.setTitle(STRINGS[lang].get("grp_chassis_config", "底盘参数设置"))
        if hasattr(self, 'btn_read_chassis'):
            self.btn_read_chassis.setText(STRINGS[lang].get("btn_read_chassis", "读取底盘参数"))
        if hasattr(self, 'btn_set_chassis'):
            self.btn_set_chassis.setText(STRINGS[lang].get("btn_set_chassis", "Write Chassis"))
        s = STRINGS[lang]
        if hasattr(self, 'lbl_chassis_type'):
            self.lbl_chassis_type.setText(s.get("lbl_chassis_type", "Type:"))
        if hasattr(self, 'lbl_wheel_dia'):
            self.lbl_wheel_dia.setText(s.get("lbl_wheel_diameter", "Wheel Dia:"))
        if hasattr(self, 'lbl_wheel_base'):
            self.lbl_wheel_base.setText(s.get("lbl_wheel_base", "Wheelbase:"))
        if hasattr(self, 'lbl_track_width'):
            self.lbl_track_width.setText(s.get("lbl_track_width", "Track Width:"))
        if hasattr(self, 'lbl_max_speed'):
            self.lbl_max_speed.setText(s.get("lbl_max_speed", "Max Speed:"))
        if hasattr(self, 'cb_chassis_type'):
            items = [
                s.get("chassis_none", "None"),
                s.get("chassis_mecanum", "Mecanum"),
                s.get("chassis_tank", "Tank/Diff"),
            ]
            for i, txt in enumerate(items):
                if i < self.cb_chassis_type.count():
                    self.cb_chassis_type.setItemText(i, txt)

        if hasattr(self, 'grp_kinematics'):
            self.grp_kinematics.setTitle(s.get("grp_kinematics", "Kinematics Parameters"))
        if hasattr(self, 'kin_labels'):
            for key, (lbl, label_key) in self.kin_labels.items():
                lbl.setText(s.get(label_key, key))
        if hasattr(self, 'btn_set_kin'):
            self.btn_set_kin.setText(s.get("btn_set_kin", "Write Params"))
        if hasattr(self, 'btn_read_kin'):
            self.btn_read_kin.setText(s.get("btn_read_kin", "Read Params"))

    def connect_signals(self):
        self.comm_manager.firmware_version_received.connect(self.update_firmware)
        self.comm_manager.battery_level_received.connect(self.update_battery)
        self.comm_manager.coord_limits_received.connect(self.update_coord_limits)
        self.comm_manager.chassis_config_received.connect(self.update_chassis_config)
        if hasattr(self.comm_manager, 'kinematics_config_received'):
            self.comm_manager.kinematics_config_received.connect(self.update_kinematics_config)
        if hasattr(self.comm_manager, 'kinematics_config_received'):
            self.comm_manager.kinematics_config_received.connect(self.update_kinematics_config)

    def update_firmware(self, esp_ver, at32_ver):
        lbl = STRINGS[self.lang].get("lbl_firmware_esp", "ESP32固件")
        if hasattr(self, 'lbl_firmware'):
            self.lbl_firmware.setText(f"{lbl}:  V {esp_ver}")
        if hasattr(self, 'lbl_firmware_at32') and at32_ver:
            lbl2 = STRINGS[self.lang].get("lbl_firmware_at32", "AT32固件")
            self.lbl_firmware_at32.setText(f"{lbl2}:  V {at32_ver}")

    def _read_firmware_versions(self):
        self.comm_manager.send_sys(CMD_FIRMWARE_VERSION_CHECK)

    def update_battery(self, vol):
        lbl = STRINGS[self.lang].get("lbl_battery", "电压")
        voltage_v = vol / 1000.0
        if hasattr(self, 'lbl_battery'):
            self.lbl_battery.setText(f"{lbl}:  {voltage_v:.2f} V")

    def set_move_acc(self):
        try:
            acc_val = self.spin_move_acc.value()
            self.comm_manager.send_sys(CMD_SET_MOVE_ACC, [acc_val])
            # 保存到本地
            self._acc_settings.setValue("move_acc", acc_val)
            # 设置成功变黄提示
            self.btn_set_acc.setStyleSheet("background-color: #FA8F01; color: white; font-weight: bold;")
            self.btn_set_acc.setText(STRINGS[self.lang].get("btn_acc_ok", "设置成功"))
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(1500, self._reset_acc_btn)
        except Exception as e:
            print(f"Set Move Acc Error: {e}")

    def _reset_acc_btn(self):
        self.btn_set_acc.setStyleSheet("")
        self.btn_set_acc.setText(STRINGS[self.lang].get("btn_set_acc", "Set/Apply"))

    def set_buzzer(self):
        try:
            # Handle case where setup_ui_from_file might have failed to find some spinboxes
            if not self.buz_vars or len(self.buz_vars) < 4:
                print("Buzzer widgets not properly initialized")
                return

            vals = []
            for sp in self.buz_vars:
                if sp is None:
                    vals.append(0) # Default if missing
                else:
                    vals.append(int(sp.value()))

            on_t, off_t, cnt, freq = vals
            args = list(struct.pack('<IIHH', on_t, off_t, cnt, freq))
            self.comm_manager.send_sys(CMD_BUZZER_SET, args)
        except Exception as e:
            print(f"Buzzer Error: {e}")

    def _build_coord_limits(self):
        layout = QVBoxLayout(self.grp_coord_limits)

        grid = QGridLayout()
        labels_row0 = ["X min", "X max", "Y min", "Y max", "Z min", "Z max"]
        defaults_row0 = [50, 450, -350, 350, 0, 500]
        labels_row1 = ["Pitch min", "Pitch max", "Roll min", "Roll max", "Claw min", "Claw max"]
        defaults_row1 = [-90, 90, -90, 90, -60, 30]

        self.coord_limit_spins = []
        for i, (lbl, val) in enumerate(zip(labels_row0, defaults_row0)):
            l = QLabel(lbl + ":")
            l.setProperty("class", "aux-text")
            grid.addWidget(l, 0, i * 2)
            sp = QSpinBox(); sp.setRange(-1000, 1000); sp.setValue(val); sp.setFixedWidth(S(80))
            self.coord_limit_spins.append(sp)
            grid.addWidget(sp, 0, i * 2 + 1)

        for i, (lbl, val) in enumerate(zip(labels_row1, defaults_row1)):
            l = QLabel(lbl + ":")
            l.setProperty("class", "aux-text")
            grid.addWidget(l, 1, i * 2)
            sp = QSpinBox(); sp.setRange(-1000, 1000); sp.setValue(val); sp.setFixedWidth(S(80))
            self.coord_limit_spins.append(sp)
            grid.addWidget(sp, 1, i * 2 + 1)

        layout.addLayout(grid)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.btn_read_cl = QPushButton(STRINGS[self.lang]["btn_read_coord_limits"])
        self.btn_read_cl.clicked.connect(self.read_coord_limits)
        btn_row.addWidget(self.btn_read_cl)

        self.btn_set_cl = QPushButton(STRINGS[self.lang]["btn_set_coord_limits"])
        self.btn_set_cl.clicked.connect(self.set_coord_limits)
        btn_row.addWidget(self.btn_set_cl)
        layout.addLayout(btn_row)

    def read_coord_limits(self):
        self.comm_manager.send_sys(CMD_GET_COORD_LIMITS)

    def set_coord_limits(self):
        vals = [sp.value() for sp in self.coord_limit_spins]
        # Pitch (index 6,7) 需要 ×10，设备端会 ÷10
        vals[6] = vals[6] * 10  # Pitch min
        vals[7] = vals[7] * 10  # Pitch max
        self.comm_manager.send_sys(CMD_SET_COORD_LIMITS, list(struct.pack('<hhhhhhhhhhhh', *vals)))

    def update_coord_limits(self, xmin, xmax, ymin, ymax, zmin, zmax, pmin, pmax, rmin, rmax, cmin, cmax):
        # Pitch 设备端存的是 ×10 的值，显示时 ÷10
        pmin = pmin // 10
        pmax = pmax // 10
        all_vals = [xmin, xmax, ymin, ymax, zmin, zmax, pmin, pmax, rmin, rmax, cmin, cmax]
        for sp, v in zip(self.coord_limit_spins, all_vals):
            sp.setValue(v)

    # ── 底盘参数设置 ──────────────────────────────────────────
    def _build_chassis_config(self):
        layout = QVBoxLayout(self.grp_chassis)
        grid = QGridLayout()
        grid.setSpacing(8)

        # Row 0: 底盘类型 | 轮径 | 轴距
        self.lbl_chassis_type = QLabel(STRINGS[self.lang].get("lbl_chassis_type", "底盘类型:"))
        self.lbl_chassis_type.setProperty("class", "aux-text")
        grid.addWidget(self.lbl_chassis_type, 0, 0)
        self.cb_chassis_type = QComboBox()
        self.cb_chassis_type.addItems([
            STRINGS[self.lang].get("chassis_none", "无底盘"),
            STRINGS[self.lang].get("chassis_mecanum", "麦克纳姆轮"),
            STRINGS[self.lang].get("chassis_tank", "履带/差速"),
        ])
        grid.addWidget(self.cb_chassis_type, 0, 1)

        self.lbl_wheel_dia = QLabel(STRINGS[self.lang].get("lbl_wheel_diameter", "轮径(mm):"))
        self.lbl_wheel_dia.setProperty("class", "aux-text")
        grid.addWidget(self.lbl_wheel_dia, 0, 2)
        self.spin_wheel_dia = QDoubleSpinBox()
        self.spin_wheel_dia.setRange(10.0, 500.0)
        self.spin_wheel_dia.setValue(25.0)
        self.spin_wheel_dia.setDecimals(1)
        grid.addWidget(self.spin_wheel_dia, 0, 3)

        self.lbl_wheel_base = QLabel(STRINGS[self.lang].get("lbl_wheel_base", "轴距(mm):"))
        self.lbl_wheel_base.setProperty("class", "aux-text")
        grid.addWidget(self.lbl_wheel_base, 0, 4)
        self.spin_wheel_base = QDoubleSpinBox()
        self.spin_wheel_base.setRange(10.0, 1000.0)
        self.spin_wheel_base.setValue(190.0)
        self.spin_wheel_base.setDecimals(1)
        grid.addWidget(self.spin_wheel_base, 0, 5)

        # Row 1: 轮距 | 最大速度 | (buttons)
        self.lbl_track_width = QLabel(STRINGS[self.lang].get("lbl_track_width", "轮距(mm):"))
        self.lbl_track_width.setProperty("class", "aux-text")
        grid.addWidget(self.lbl_track_width, 1, 0)
        self.spin_track_width = QDoubleSpinBox()
        self.spin_track_width.setRange(10.0, 1000.0)
        self.spin_track_width.setValue(95.0)
        self.spin_track_width.setDecimals(1)
        grid.addWidget(self.spin_track_width, 1, 1)

        self.lbl_max_speed = QLabel(STRINGS[self.lang].get("lbl_max_speed", "最大速度:"))
        self.lbl_max_speed.setProperty("class", "aux-text")
        grid.addWidget(self.lbl_max_speed, 1, 2)
        self.spin_max_speed = QSpinBox()
        self.spin_max_speed.setRange(1, 100)
        self.spin_max_speed.setValue(100)
        grid.addWidget(self.spin_max_speed, 1, 3)

        # Uniform height for all input widgets in this grid
        uniform_h = S(32)
        for w in [self.cb_chassis_type, self.spin_wheel_dia, self.spin_wheel_base,
                   self.spin_track_width, self.spin_max_speed]:
            w.setFixedHeight(uniform_h)

        # All input columns equal stretch
        for c in (1, 3, 5):
            grid.setColumnStretch(c, 1)

        layout.addLayout(grid)

        # 按钮行
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.btn_read_chassis = QPushButton(STRINGS[self.lang].get("btn_read_chassis", "读取底盘参数"))
        self.btn_read_chassis.clicked.connect(self.read_chassis_config)
        btn_row.addWidget(self.btn_read_chassis)

        self.btn_set_chassis = QPushButton(STRINGS[self.lang].get("btn_set_chassis", "写入底盘参数"))
        self.btn_set_chassis.clicked.connect(self.set_chassis_config)
        btn_row.addWidget(self.btn_set_chassis)
        layout.addLayout(btn_row)

    def read_chassis_config(self):
        self.comm_manager.send_sys(CMD_GET_CHASSIS_CONFIG)

    def set_chassis_config(self):
        try:
            chassis_type = self.cb_chassis_type.currentIndex()  # 0=无, 1=麦轮, 2=履带
            wheel_dia = self.spin_wheel_dia.value()
            wheel_base = self.spin_wheel_base.value()
            track_width = self.spin_track_width.value()
            max_speed = self.spin_max_speed.value()

            # 打包: type(1B) + wheel_dia(4B float) + wheel_base(4B float) + track_width(4B float)
            #        + motor_dir(4B) + max_speed(1B) = 18字节
            args = [chassis_type]
            args += list(struct.pack('<f', wheel_dia))
            args += list(struct.pack('<f', wheel_base))
            args += list(struct.pack('<f', track_width))
            args += [0, 0, 0, 0]  # motor_dir 默认全0（正常方向）
            args += [max_speed & 0xFF]

            self.comm_manager.send_sys(CMD_SET_CHASSIS_CONFIG, args)
        except Exception as e:
            print(f"Set Chassis Config Error: {e}")

    def update_chassis_config(self, chassis_type, wheel_dia, wheel_base, track_width, max_speed):
        """从从机回包更新UI"""
        if hasattr(self, 'cb_chassis_type'):
            self.cb_chassis_type.setCurrentIndex(min(chassis_type, 2))
        if hasattr(self, 'spin_wheel_dia'):
            self.spin_wheel_dia.setValue(wheel_dia)
        if hasattr(self, 'spin_wheel_base'):
            self.spin_wheel_base.setValue(wheel_base)
        if hasattr(self, 'spin_track_width'):
            self.spin_track_width.setValue(track_width)
        if hasattr(self, 'spin_max_speed'):
            self.spin_max_speed.setValue(max_speed)

    # ── 运动学参数设置 ──────────────────────────────────────
    def _build_kinematics_config(self):
        grid = QGridLayout(self.grp_kinematics)
        grid.setSpacing(4)
        self.kin_spins = {}
        kin_params = [
            ("linkage1",      110.45, "kin_linkage1"),
            ("linkage2",      225.00, "kin_linkage2"),
            ("linkage2_perp",  36.97, "kin_linkage2_perp"),
            ("linkage3",      145.00, "kin_linkage3"),
            ("linkage3_perp",   0.00, "kin_linkage3_perp"),
            ("linkage4",      130.23, "kin_linkage4"),
            ("fixed_offset",    0.00, "kin_fixed_offset"),
            ("base_radius",    50.00, "kin_base_radius"),
            ("base_high",      70.50, "kin_base_high"),
        ]
        self.kin_labels = {}
        for i, (key, default, label_key) in enumerate(kin_params):
            row, col = divmod(i, 5)
            lbl = QLabel(STRINGS[self.lang].get(label_key, key))
            lbl.setStyleSheet("color: #FFFFFF; font-size: 8pt;")
            self.kin_labels[key] = (lbl, label_key)
            sp = QDoubleSpinBox()
            sp.setRange(-500.0, 500.0)
            sp.setDecimals(2)
            sp.setSingleStep(0.01)
            sp.setValue(default)
            sp.setFixedWidth(100)
            sp.setStyleSheet("font-size: 10pt;")
            self.kin_spins[key] = sp
            h = QHBoxLayout()
            h.setSpacing(2)
            h.addWidget(lbl)
            h.addWidget(sp)
            grid.addLayout(h, row, col)

        self.btn_set_kin = QPushButton(STRINGS[self.lang].get("btn_set_kin", "写入参数"))
        self.btn_set_kin.setStyleSheet("background-color: #1976D2; color: white; font-weight: bold;")
        self.btn_set_kin.clicked.connect(self.send_kinematics_config)

        self.btn_read_kin = QPushButton(STRINGS[self.lang].get("btn_read_kin", "读取参数"))
        self.btn_read_kin.clicked.connect(lambda: self.comm_manager.send_sys(CMD_GET_KINEMATICS_PARAM))

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_set_kin)
        btn_layout.addWidget(self.btn_read_kin)
        btn_layout.addStretch()
        grid.addLayout(btn_layout, 2, 0, 1, 5)

    def send_kinematics_config(self):
        try:
            keys = ["linkage1", "linkage2", "linkage2_perp", "linkage3",
                    "linkage3_perp", "linkage4", "fixed_offset", "base_radius", "base_high"]
            args = []
            for k in keys:
                val = self.kin_spins[k].value()
                args += list(struct.pack('<f', val))
            self.comm_manager.send_sys(CMD_SET_KINEMATICS_PARAM, args)
            print(f"[Kinematics] Sent: { {k: self.kin_spins[k].value() for k in keys} }")
            self.btn_set_kin.setStyleSheet("background-color: #FA8F01; color: white; font-weight: bold;")
            self.btn_set_kin.setText(STRINGS[self.lang].get("msg_set_success", "设置成功"))
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(1500, self._reset_kin_btn)
        except Exception as e:
            print(f"Set Kinematics Error: {e}")

    def _reset_kin_btn(self):
        self.btn_set_kin.setStyleSheet("background-color: #1976D2; color: white; font-weight: bold;")
        self.btn_set_kin.setText(STRINGS[self.lang].get("btn_set_kin", "写入参数"))

    def update_kinematics_config(self, params):
        """从从机回包更新运动学参数UI, params是9个float的列表"""
        keys = ["linkage1", "linkage2", "linkage2_perp", "linkage3",
                "linkage3_perp", "linkage4", "fixed_offset", "base_radius", "base_high"]
        for k, v in zip(keys, params):
            if k in self.kin_spins:
                self.kin_spins[k].setValue(v)

