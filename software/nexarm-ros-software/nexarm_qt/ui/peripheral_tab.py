from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QPushButton,
    QLabel, QSpinBox, QComboBox, QGridLayout, QLineEdit, QScrollArea,
    QSlider, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
import struct
import os
from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S


class PeripheralTab(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.setup_ui()
        self.comm_manager.channel_scan_received.connect(self._show_channel_scan)

    # ── UI ──────────────────────────────────────────────────────

    def setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(10)
        layout.setContentsMargins(5, 5, 5, 5)

        # ══════════════════════════════════════════════════════════
        # 上半部分：横着 4 块分区
        # ══════════════════════════════════════════════════════════
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        # 统一 GroupBox 样式
        grp_style = """
            QGroupBox {
                border: 1px solid #4A4D5E;
                border-radius: 6px;
                margin-top: 12px;
                padding: 12px 8px 8px 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #FA8F01;
                font-weight: bold;
            }
        """

        # ── 1. 传送带 ──
        self.grp_conv = QGroupBox(STRINGS[self.lang].get("lbl_conveyor", "传送带"))
        conv_lay = QVBoxLayout(self.grp_conv)
        conv_lay.setSpacing(6)
        r1 = QHBoxLayout()
        self.lbl_conv_speed = QLabel(STRINGS[self.lang].get("lbl_speed", "速度:"))
        r1.addWidget(self.lbl_conv_speed)
        self.ent_belt = QSpinBox()
        self.ent_belt.setRange(-100, 100); self.ent_belt.setValue(50)
        r1.addWidget(self.ent_belt)
        conv_lay.addLayout(r1)
        r2 = QHBoxLayout()
        self.btn_conv_run = QPushButton(STRINGS[self.lang].get("btn_run", "运行"))
        self.btn_conv_run.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_CONVEYOR_SET, [self.ent_belt.value() & 0xFF]), self._flash(self.btn_conv_run)))
        r2.addWidget(self.btn_conv_run)
        self.btn_conv_stop = QPushButton(STRINGS[self.lang].get("btn_stop", "停止"))
        self.btn_conv_stop.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_CONVEYOR_SET, [0]), self._flash(self.btn_conv_stop)))
        r2.addWidget(self.btn_conv_stop)
        conv_lay.addLayout(r2)
        conv_lay.addStretch()
        self.grp_conv.setStyleSheet(grp_style)
        self.grp_conv.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_row.addWidget(self.grp_conv, 1)

        # ── 2. 滑轨(步进电机) ──
        self.grp_step = QGroupBox(STRINGS[self.lang].get("lbl_stepper", "滑轨"))
        step_lay = QVBoxLayout(self.grp_step)
        step_lay.setSpacing(6)
        s1 = QHBoxLayout()
        self.lbl_step_count = QLabel(STRINGS[self.lang].get("lbl_steps", "步数:"))
        s1.addWidget(self.lbl_step_count)
        self.ent_step = QSpinBox()
        self.ent_step.setRange(-100000, 100000); self.ent_step.setValue(2000)
        s1.addWidget(self.ent_step)
        step_lay.addLayout(s1)
        s2 = QHBoxLayout()
        self.btn_step_run = QPushButton(STRINGS[self.lang].get("btn_run", "运行"))
        self.btn_step_run.clicked.connect(lambda: (self.send_stepper(), self._flash(self.btn_step_run)))
        s2.addWidget(self.btn_step_run)
        self.btn_step_rst = QPushButton(STRINGS[self.lang].get("btn_reset", "复位"))
        self.btn_step_rst.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_STEPPER_RESET), self._flash(self.btn_step_rst)))
        s2.addWidget(self.btn_step_rst)
        step_lay.addLayout(s2)
        s3 = QHBoxLayout()
        self.lbl_div = QLabel(STRINGS[self.lang].get("lbl_div", "细分:"))
        s3.addWidget(self.lbl_div)
        self.cb_step_div = QComboBox()
        self.cb_step_div.addItems(["1", "2", "4", "8", "16", "32"]); self.cb_step_div.setCurrentIndex(2)
        s3.addWidget(self.cb_step_div)
        self.btn_step_div = QPushButton(STRINGS[self.lang].get("btn_set_div", "设置"))
        self.btn_step_div.clicked.connect(lambda: (self.send_stepper_div(), self._flash(self.btn_step_div)))
        s3.addWidget(self.btn_step_div)
        step_lay.addLayout(s3)
        step_lay.addStretch()
        self.grp_step.setStyleSheet(grp_style)
        self.grp_step.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_row.addWidget(self.grp_step, 1)

        # ── 3. ESP-NOW ──
        self.grp_esp = QGroupBox("ESP-NOW")
        esp_lay = QVBoxLayout(self.grp_esp)
        esp_lay.setSpacing(6)
        e1 = QHBoxLayout()
        self.btn_esp_on = QPushButton(STRINGS[self.lang].get("btn_esp_mode_on", "开启ESP-NOW"))
        self.btn_esp_on.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_ENABLE, [1]), self._flash(self.btn_esp_on)))
        e1.addWidget(self.btn_esp_on)
        self.btn_esp_off = QPushButton(STRINGS[self.lang].get("btn_esp_mode_off", "关闭(WiFi)"))
        self.btn_esp_off.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_ENABLE, [0]), self._flash(self.btn_esp_off)))
        e1.addWidget(self.btn_esp_off)
        esp_lay.addLayout(e1)
        e2 = QHBoxLayout()
        self.btn_sync_on = QPushButton(STRINGS[self.lang].get("btn_sync_on", "开启同步"))
        self.btn_sync_on.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_SYNC, [1]), self._flash(self.btn_sync_on)))
        e2.addWidget(self.btn_sync_on)
        self.btn_sync_off = QPushButton(STRINGS[self.lang].get("btn_sync_off", "关闭同步"))
        self.btn_sync_off.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_SYNC, [0]), self._flash(self.btn_sync_off)))
        e2.addWidget(self.btn_sync_off)
        esp_lay.addLayout(e2)
        e3 = QHBoxLayout()
        e3.addWidget(QLabel("MAC:"))
        self.ent_mac = QLineEdit("A4:C1:38:12:34:56")
        e3.addWidget(self.ent_mac)
        esp_lay.addLayout(e3)
        e4 = QHBoxLayout()
        self.btn_mac_pair = QPushButton(STRINGS[self.lang].get("btn_pair", "锁定(配对)"))
        self.btn_mac_pair.clicked.connect(lambda: (self.send_mac_pair(), self._flash(self.btn_mac_pair)))
        e4.addWidget(self.btn_mac_pair)
        self.btn_mac_scan = QPushButton(STRINGS[self.lang].get("btn_scan", "刷新(广播)"))
        self.btn_mac_scan.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_SCAN), self._flash(self.btn_mac_scan)))
        e4.addWidget(self.btn_mac_scan)
        esp_lay.addLayout(e4)
        e5 = QHBoxLayout()
        self.lbl_channel = QLabel(STRINGS[self.lang].get("lbl_channel", "信道:"))
        e5.addWidget(self.lbl_channel)
        self.spin_channel = QSpinBox(); self.spin_channel.setRange(0, 14); self.spin_channel.setValue(1)
        e5.addWidget(self.spin_channel)
        self.btn_set_ch = QPushButton(STRINGS[self.lang].get("btn_set_channel", "设置信道"))
        self.btn_set_ch.clicked.connect(lambda: (self.send_channel(), self._flash(self.btn_set_ch)))
        e5.addWidget(self.btn_set_ch)
        esp_lay.addLayout(e5)
        e6 = QHBoxLayout()
        self.lbl_sync_acc = QLabel(STRINGS[self.lang].get("lbl_sync_acc", "同步加速度:"))
        e6.addWidget(self.lbl_sync_acc)
        self.spin_sync_acc = QSpinBox(); self.spin_sync_acc.setRange(0, 254)
        e6.addWidget(self.spin_sync_acc)
        self.btn_set_sync_acc = QPushButton(STRINGS[self.lang].get("btn_set_acc", "设置"))
        self.btn_set_sync_acc.clicked.connect(lambda: (self.send_sync_acc(), self._flash(self.btn_set_sync_acc)))
        e6.addWidget(self.btn_set_sync_acc)
        esp_lay.addLayout(e6)
        self.btn_scan_ch = QPushButton(STRINGS[self.lang].get("btn_scan_channel", "扫描信道"))
        self.btn_scan_ch.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_ESPNOW_SCAN_CHANNEL), self._flash(self.btn_scan_ch)))
        esp_lay.addWidget(self.btn_scan_ch)
        esp_lay.addStretch()
        self.grp_esp.setStyleSheet(grp_style)
        self.grp_esp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_row.addWidget(self.grp_esp, 1)

        # ── 4. PS3 MAC ──
        self.grp_ps3 = QGroupBox("PS3 MAC")
        ps3_lay = QVBoxLayout(self.grp_ps3)
        ps3_lay.setSpacing(6)
        ps3_lay.addWidget(QLabel("PS3 MAC:"))
        self.ent_ps3_mac = QLineEdit("10:00:00:00:85:95")
        ps3_lay.addWidget(self.ent_ps3_mac)
        self.btn_set_ps3 = QPushButton(STRINGS[self.lang].get("btn_set_mac", "设置MAC"))
        self.btn_set_ps3.clicked.connect(lambda: (self.send_ps3_mac(), self._flash(self.btn_set_ps3)))
        ps3_lay.addWidget(self.btn_set_ps3)
        ps3_lay.addStretch()
        self.grp_ps3.setStyleSheet(grp_style)
        self.grp_ps3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_row.addWidget(self.grp_ps3, 1)

        layout.addLayout(top_row)

        # ══════════════════════════════════════════════════════════
        # 下半部分：麦轮底盘 + 履带底盘（圆形方向盘）
        # ══════════════════════════════════════════════════════════
        from nexarm_qt.ui.dpad_widget import DPad

        wheels_row = QHBoxLayout()
        wheels_row.setSpacing(20)

        # ── 左：麦克纳姆轮 (WSAD + QE) ──
        self.mec_grp = QGroupBox(STRINGS[self.lang].get("grp_mecanum", "麦克纳姆轮"))
        self.mec_grp.setStyleSheet(grp_style)
        mec_main = QVBoxLayout(self.mec_grp)
        mec_main.setContentsMargins(10, 20, 10, 10)

        self.dpad_mec = DPad(mode='6dir')
        self.dpad_mec.direction_pressed.connect(self._on_mec_pressed)
        self.dpad_mec.direction_released.connect(self._on_mec_released)
        self.dpad_mec.stop_clicked.connect(self._mec_stop)
        mec_main.addWidget(self.dpad_mec)
        wheels_row.addWidget(self.mec_grp, 1)

        # ── 右：履带模式 (方向键 ↑↓←→) ──
        self.tank_grp = QGroupBox(STRINGS[self.lang].get("grp_tank", "履带模式"))
        self.tank_grp.setStyleSheet(grp_style)
        tank_main = QVBoxLayout(self.tank_grp)
        tank_main.setContentsMargins(10, 20, 10, 10)

        self.dpad_tank = DPad(mode='4dir')
        self.dpad_tank.direction_pressed.connect(self._on_tank_pressed)
        self.dpad_tank.direction_released.connect(lambda _: self._tank_stop())
        self.dpad_tank.stop_clicked.connect(self._tank_stop)
        tank_main.addWidget(self.dpad_tank)
        wheels_row.addWidget(self.tank_grp, 1)

        layout.addLayout(wheels_row)

        # ── 单电机控制 (4个横向滑杆 + 同步) ──
        self.motor_grp = QGroupBox(STRINGS[self.lang].get("grp_single_motor", "单电机控制"))
        self.motor_grp.setStyleSheet(grp_style)
        motor_lay = QVBoxLayout(self.motor_grp)
        motor_lay.setSpacing(6)

        self.motor_labels = []
        self.motor_sliders = []
        for i in range(4):
            row = QHBoxLayout()
            lbl = QLabel(f"M{i+1}: 0")
            lbl.setFixedWidth(60)
            lbl.setStyleSheet("color: #ffffff; font-size: 9pt;")
            self.motor_labels.append(lbl)
            row.addWidget(lbl)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-100, 100); slider.setValue(0)
            slider.valueChanged.connect(lambda v, x=i: self._on_motor_slide(x, v))
            self.motor_sliders.append(slider)
            row.addWidget(slider)
            motor_lay.addLayout(row)

        # 同步发送行
        sync_row = QHBoxLayout()
        self.lbl_sync = QLabel(STRINGS[self.lang].get("lbl_sync_label", "同步:"))
        sync_row.addWidget(self.lbl_sync)
        self.multi_mot_vars = []
        for i in range(4):
            sync_row.addWidget(QLabel(f"M{i+1}"))
            sp = QSpinBox(); sp.setRange(-100, 100)
            self.multi_mot_vars.append(sp)
            sync_row.addWidget(sp)
        self.btn_multi = QPushButton(STRINGS[self.lang].get("btn_sync_send", "同步发送"))
        self.btn_multi.clicked.connect(lambda: (self.send_multi_motors(), self._flash(self.btn_multi)))
        sync_row.addWidget(self.btn_multi)
        self.btn_stop_all = QPushButton(STRINGS[self.lang].get("btn_stop_all", "全部停止"))
        self.btn_stop_all.clicked.connect(lambda: (self.comm_manager.send_sys(CMD_MOTOR_STOP), self._flash(self.btn_stop_all)))
        sync_row.addWidget(self.btn_stop_all)
        motor_lay.addLayout(sync_row)

        layout.addWidget(self.motor_grp)

        layout.addStretch()
        scroll.setWidget(container)
        outer.addWidget(scroll)

    # ── Actions ─────────────────────────────────────────────────

    def send_stepper(self):
        try:
            steps = self.ent_step.value()
            self.comm_manager.send_sys(CMD_STEPPER_RUN, list(struct.pack('<i', steps)))
        except Exception as e:
            print(f"Stepper Error: {e}")

    def send_stepper_div(self):
        """发送细分设置 — 需要映射为 I2C 编码值"""
        # ComboBox: ["1","2","4","8","16","32"]
        # I2C 编码: SUB_NONE=0x00, SUB_2=0x01, SUB_4=0x02, SUB_8=0x03, SUB_16=0x07
        div_map = {1: 0x00, 2: 0x01, 4: 0x02, 8: 0x03, 16: 0x07, 32: 0x07}
        try:
            div = int(self.cb_step_div.currentText())
            code = div_map.get(div, 0x00)
            print(f"[Stepper] div={div} -> code=0x{code:02X}")
            self.comm_manager.send_sys(CMD_STEPPER_SET_DIV, [code])
        except Exception as e:
            print(f"Stepper Div Error: {e}")

    def send_mac_pair(self):
        try:
            mac_str = self.ent_mac.text().strip()

            
            mac_bytes = [int(x, 16) for x in mac_str.split(":")]
            self.comm_manager.send_sys(CMD_ESPNOW_SET_MAC, mac_bytes)
        except Exception as e:
            print(f"MAC Pair Error: {e}")

    def send_channel(self):
        self.comm_manager.send_sys(CMD_ESPNOW_SET_CHANNEL, [self.spin_channel.value()])

    def send_sync_acc(self):
        self.comm_manager.send_sys(CMD_ESPNOW_SET_ACC, [self.spin_sync_acc.value()])

    def send_ps3_mac(self):
        try:
            mac_str = self.ent_ps3_mac.text().strip()
            mac_bytes = [int(x, 16) for x in mac_str.split(":")]
            self.comm_manager.send_sys(CMD_PS3_SET_MAC, mac_bytes)
        except Exception as e:
            print(f"PS3 MAC Error: {e}")

    def _on_mec_pressed(self, name):
        """麦轮方向盘按下 — 支持多键同时按下，合并速度向量"""
        self._send_mec_combined()

    def _send_mec_combined(self):
        """根据当前所有按下的方向，合并速度向量发送"""
        mapping = {
            'up':            (0, -50, 0),   # W 前进 = vy 负
            'down':          (0, 50, 0),    # S 后退 = vy 正
            'left':          (0, 0, 50),
            'right':         (0, 0, -50),
            'left_strafe':   (-50, 0, 0),
            'right_strafe':  (50, 0, 0),
        }
        vx, vy, vz = 0, 0, 0
        for name in self.dpad_mec._active_set:
            if name in mapping:
                dx, dy, dz = mapping[name]
                vx += dx; vy += dy; vz += dz
        vx = max(-100, min(100, vx))
        vy = max(-100, min(100, vy))
        vz = max(-100, min(100, vz))
        self.comm_manager.send_sys(CMD_MECANUM_RUN, [vx & 0xFF, vy & 0xFF, vz & 0xFF])

    def _on_mec_released(self, name):
        """麦轮方向盘释放 — 如果还有其他键按着就发合并向量，否则停止"""
        if self.dpad_mec._active_set:
            self._send_mec_combined()
        else:
            self._mec_stop()

    def _mec_stop(self):
        self.comm_manager.send_sys(CMD_MOTOR_STOP)

    def _on_tank_pressed(self, name):
        """履带方向盘按下"""
        mapping = {
            'up':    (50, 0),
            'down':  (-50, 0),
            'left':  (0, 50),
            'right': (0, -50),
        }
        if name in mapping:
            s, t = mapping[name]
            self.comm_manager.send_sys(CMD_TANK_RUN, [s & 0xFF, t & 0xFF])

    def _tank_stop(self):
        self.comm_manager.send_sys(CMD_MOTOR_STOP)

    def _on_motor_slide(self, idx, val):
        if idx < len(self.motor_labels):
            self.motor_labels[idx].setText(f"M{idx+1}: {val}")
        self.comm_manager.send_sys(CMD_SET_SINGLE_MOTOR, [idx + 1, val & 0xFF])

    def send_multi_motors(self):
        try:
            speeds = [sp.value() & 0xFF for sp in self.multi_mot_vars]
            self.comm_manager.send_sys(CMD_SET_MOTOR_SPEED, speeds)
        except Exception as e:
            print(f"Multi Motor Error: {e}")

    def _flash(self, btn):
        """按钮变黄显示'成功'，1.5秒后恢复"""
        if not hasattr(btn, '_original_text'):
            btn._original_text = btn.text()
        btn.setStyleSheet("background-color: #FA8F01; color: white; font-weight: bold;")
        btn.setText("成功")
        text = btn._original_text
        QTimer.singleShot(1500, lambda t=text, b=btn: (b.setStyleSheet(""), b.setText(t)))

    def _show_channel_scan(self, results):
        from PyQt5.QtWidgets import QDialog, QTableWidget, QTableWidgetItem, QHeaderView
        s = STRINGS[self.lang]
        dlg = QDialog(self)
        dlg.setWindowTitle(s.get("title_channel_scan", "ESP-NOW Channel Quality"))
        dlg.setMinimumSize(500, 400)
        dlg.setStyleSheet("background-color: #1E1F31; color: white;")
        lay = QVBoxLayout(dlg)

        hint = QLabel(s.get("lbl_channel_hint", ""))
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #B0BEC5; font-size: 9pt; padding: 4px;")
        lay.addWidget(hint)

        tbl = QTableWidget(13, 4)
        tbl.setHorizontalHeaderLabels([
            s.get("lbl_channel", "Channel"),
            s.get("lbl_ap_count", "APs"),
            s.get("lbl_rssi", "Signal(dBm)"),
            s.get("lbl_quality", "Quality"),
        ])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setStyleSheet("""
            QTableWidget { background: #2D2F3F; border: 1px solid #4A4D5E; gridline-color: #4A4D5E; }
            QHeaderView::section { background: #343645; color: #B0BEC5; font-weight: bold; border: none; padding: 6px; }
            QTableWidget::item { padding: 4px; }
        """)

        for i, (ap_count, rssi) in enumerate(results):
            ch_item = QTableWidgetItem(str(i + 1))
            ch_item.setTextAlignment(Qt.AlignCenter)
            tbl.setItem(i, 0, ch_item)

            ap_item = QTableWidgetItem(str(ap_count))
            ap_item.setTextAlignment(Qt.AlignCenter)
            tbl.setItem(i, 1, ap_item)

            rssi_item = QTableWidgetItem(str(rssi))
            rssi_item.setTextAlignment(Qt.AlignCenter)
            tbl.setItem(i, 2, rssi_item)

            if ap_count <= 2:
                quality, color = "★★★", "#4CAF50"
            elif ap_count <= 5:
                quality, color = "★★", "#FF9800"
            else:
                quality, color = "★", "#F44336"
            q_item = QTableWidgetItem(quality)
            q_item.setTextAlignment(Qt.AlignCenter)
            q_item.setForeground(QColor(color))
            tbl.setItem(i, 3, q_item)

        lay.addWidget(tbl)
        dlg.exec_()

    # ── Language ─────────────────────────────────────────────────

    def update_language(self, lang):
        self.lang = lang
        S = STRINGS[lang]
        self.grp_conv.setTitle(S.get("lbl_conveyor", "Conveyor"))
        self.grp_step.setTitle(S.get("lbl_stepper", "Stepper"))
        self.grp_ps3.setTitle("PS3 MAC")
        self.btn_conv_run.setText(S.get("btn_run", "Run"))
        self.btn_conv_stop.setText(S.get("btn_stop", "Stop"))
        self.btn_step_run.setText(S.get("btn_run", "Run"))
        self.btn_step_rst.setText(S.get("btn_reset", "Reset"))
        self.btn_esp_on.setText(S.get("btn_esp_mode_on", "Enable ESP-NOW"))
        self.btn_esp_off.setText(S.get("btn_esp_mode_off", "Disable(WiFi)"))
        self.btn_sync_on.setText(S.get("btn_sync_on", "Sync ON"))
        self.btn_sync_off.setText(S.get("btn_sync_off", "Sync OFF"))
        self.btn_step_div.setText(S.get("btn_set_div", "Set"))
        self.btn_set_ch.setText(S.get("btn_set_channel", "Set Channel"))
        self.btn_set_sync_acc.setText(S.get("btn_set_acc", "Set"))
        self.btn_scan_ch.setText(S.get("btn_log_channel", "Log Channel"))
        self.btn_mac_pair.setText(S.get("btn_pair", "Pair"))
        self.btn_mac_scan.setText(S.get("btn_scan", "Scan"))
        self.btn_set_ps3.setText(S.get("btn_set_mac", "Set MAC"))
        self.btn_multi.setText(S.get("btn_sync_send", "Sync Send"))
        self.btn_stop_all.setText(S.get("btn_stop_all", "Stop All"))
        self.lbl_conv_speed.setText(S.get("lbl_speed", "Speed:"))
        self.lbl_step_count.setText(S.get("lbl_steps", "Steps:"))
        self.lbl_div.setText(S.get("lbl_div", "Div:"))
        self.lbl_channel.setText(S.get("lbl_channel", "Channel:"))
        self.lbl_sync_acc.setText(S.get("lbl_sync_acc", "Sync Accel:"))
        self.lbl_sync.setText(S.get("lbl_sync_label", "Sync:"))
        self.mec_grp.setTitle(S.get("grp_mecanum", "Mecanum"))
        self.tank_grp.setTitle(S.get("grp_tank", "Tank"))
        self.motor_grp.setTitle(S.get("grp_single_motor", "Motor"))
        if hasattr(self, 'dpad_mec'):
            self.dpad_mec.update_language(lang)
        if hasattr(self, 'dpad_tank'):
            self.dpad_tank.update_language(lang)
