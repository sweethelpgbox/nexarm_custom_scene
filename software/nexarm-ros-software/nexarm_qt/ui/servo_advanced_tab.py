from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QPushButton,
    QLabel, QSpinBox, QGridLayout, QComboBox, QScrollArea
)
from PyQt5.QtCore import Qt
import struct
from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS


class ServoAdvancedTab(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        container = QWidget()
        self.grid = QGridLayout(container)
        self.grid.setHorizontalSpacing(6)
        self.grid.setVerticalSpacing(17)
        self.grid.setContentsMargins(17, 17, 17, 17)
        self.grid.setColumnStretch(0, 0)
        self.grid.setColumnStretch(1, 1)
        self.grid.setColumnStretch(2, 0)
        self.grid.setColumnStretch(3, 1)
        self.grid.setColumnStretch(4, 1)
        self.grid.setColumnStretch(5, 1)

        r = 0
        r = self._add_section(r, "grp_servo_cfg")
        r = self._build_servo_cfg(r)
        r = self._add_section(r, "grp_servo_adv")
        r = self._build_servo_adv(r)
        r = self._add_section(r, "grp_overload")
        r = self._build_overload(r)
        r = self._add_section(r, "grp_baud")
        r = self._build_baud(r)
        r = self._add_section(r, "grp_max_torque")
        r = self._build_max_torque(r)
        self.grid.setRowStretch(r, 1)
        scroll.setWidget(container)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    # ── Helpers ───────────────────────────────────────────────

    def _add_section(self, r, key):
        lbl = QLabel(STRINGS[self.lang].get(key, key))
        lbl.setStyleSheet("color: #FA8F01; font-size: 11pt; font-weight: bold; padding-top: 6px;")
        self.grid.addWidget(lbl, r, 0, 1, 6)
        if not hasattr(self, '_section_labels'):
            self._section_labels = []
        self._section_labels.append((lbl, key))
        return r + 1

    def _spin(self, lo, hi, val=0):
        sp = QSpinBox(); sp.setRange(lo, hi); sp.setValue(val)
        return sp

    def _btn(self, key, callback):
        b = QPushButton(STRINGS[self.lang].get(key, key))
        b.clicked.connect(callback)
        return b

    # ── Build sections ────────────────────────────────────────

    def _build_servo_cfg(self, r):
        g = self.grid
        self.lbl_old_id = QLabel(STRINGS[self.lang]["lbl_old_id"])
        g.addWidget(self.lbl_old_id, r, 0)
        self.spin_old_id = self._spin(1, 255)
        g.addWidget(self.spin_old_id, r, 1)
        self.lbl_new_id = QLabel(STRINGS[self.lang]["lbl_new_id"])
        g.addWidget(self.lbl_new_id, r, 2)
        self.spin_new_id = self._spin(1, 255)
        g.addWidget(self.spin_new_id, r, 3)
        self.btn_set_id = self._btn("btn_set_id", self.set_servo_id)
        g.addWidget(self.btn_set_id, r, 4, 1, 2)
        r += 1
        self.lbl_mode_id = QLabel(STRINGS[self.lang]["lbl_servo_id"])
        g.addWidget(self.lbl_mode_id, r, 0)
        self.spin_mode_id = self._spin(1, 255)
        g.addWidget(self.spin_mode_id, r, 1)
        self.lbl_mode = QLabel(STRINGS[self.lang]["lbl_mode"])
        g.addWidget(self.lbl_mode, r, 2)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(STRINGS[self.lang].get("servo_modes", ["0: Position", "1: Wheel", "2: PWM", "3: Stepper"]))
        g.addWidget(self.combo_mode, r, 3)
        self.btn_set_mode = self._btn("btn_set_mode", self.set_servo_mode)
        g.addWidget(self.btn_set_mode, r, 4, 1, 2)
        return r + 1

    def _build_servo_adv(self, r):
        g = self.grid
        self.lbl_adv_id = QLabel(STRINGS[self.lang]["lbl_servo_id"])
        g.addWidget(self.lbl_adv_id, r, 0)
        self.spin_adv_id = self._spin(1, 255)
        g.addWidget(self.spin_adv_id, r, 1)
        self.lbl_offset = QLabel(STRINGS[self.lang]["lbl_offset"])
        g.addWidget(self.lbl_offset, r, 2)
        self.spin_offset = self._spin(-2047, 2047)
        # 回车实时预览偏差
        self.spin_offset.editingFinished.connect(self._preview_offset)
        g.addWidget(self.spin_offset, r, 3)
        self.btn_set_offset = self._btn("btn_set_offset", self.set_pos_offset)
        g.addWidget(self.btn_set_offset, r, 4)
        self.btn_get_offset = self._btn("btn_get_offset", self.get_pos_offset)
        g.addWidget(self.btn_get_offset, r, 5)
        r += 1
        self.btn_cali_pos = QPushButton(STRINGS[self.lang].get("btn_cali_pos", "校准中位"))
        self.btn_cali_pos.clicked.connect(self.cali_pos)
        self.btn_cali_pos.setVisible(False)
        g.addWidget(self.btn_cali_pos, r, 2)
        self.btn_cali_pos_all = QPushButton(STRINGS[self.lang].get("btn_cali_pos_all", "全部校准中位"))
        self.btn_cali_pos_all.clicked.connect(self.cali_pos_all)
        self.btn_cali_pos_all.setVisible(False)
        g.addWidget(self.btn_cali_pos_all, r, 3)
        r += 1
        self.lbl_pid = QLabel(STRINGS[self.lang]["lbl_pid"])
        g.addWidget(self.lbl_pid, r, 0)
        self.spin_p = self._spin(0, 255)
        self.spin_i = self._spin(0, 255)
        self.spin_d = self._spin(0, 255)
        self.spin_minf = self._spin(0, 65535)
        pid_w = QWidget()
        pid_lay = QHBoxLayout(pid_w)
        pid_lay.setContentsMargins(0, 0, 0, 0)
        pid_lay.setSpacing(4)
        pid_lay.addWidget(QLabel("P:")); pid_lay.addWidget(self.spin_p)
        pid_lay.addWidget(QLabel("I:")); pid_lay.addWidget(self.spin_i)
        pid_lay.addWidget(QLabel("D:")); pid_lay.addWidget(self.spin_d)
        pid_lay.addWidget(QLabel("MinF:")); pid_lay.addWidget(self.spin_minf)
        g.addWidget(pid_w, r, 1, 1, 3)
        self.btn_set_pid = self._btn("btn_set_pid", self.set_pid_param)
        g.addWidget(self.btn_set_pid, r, 4)
        self.btn_get_pid = self._btn("btn_get_pid", self.get_pid_param)
        g.addWidget(self.btn_get_pid, r, 5)
        return r + 1

    def _build_overload(self, r):
        g = self.grid
        self.lbl_ol_id = QLabel(STRINGS[self.lang]["lbl_servo_id"])
        g.addWidget(self.lbl_ol_id, r, 0)
        self.spin_ol_id = self._spin(1, 255)
        g.addWidget(self.spin_ol_id, r, 1)
        self.lbl_ol_torque = QLabel(STRINGS[self.lang]["lbl_overload_torque"])
        g.addWidget(self.lbl_ol_torque, r, 2)
        self.spin_ol_torque = self._spin(0, 254, 20)
        g.addWidget(self.spin_ol_torque, r, 3)
        self.btn_read_ol = self._btn("btn_read_overload", self.read_overload)
        g.addWidget(self.btn_read_ol, r, 4)
        self.btn_set_ol = self._btn("btn_set_overload", self.set_overload)
        g.addWidget(self.btn_set_ol, r, 5)
        r += 1
        self.lbl_ol_time = QLabel(STRINGS[self.lang]["lbl_overload_time"])
        g.addWidget(self.lbl_ol_time, r, 0)
        self.spin_ol_time = self._spin(0, 254, 200)
        g.addWidget(self.spin_ol_time, r, 1)
        self.lbl_ol_thresh = QLabel(STRINGS[self.lang].get("lbl_overload_thresh", "扭矩阈值"))
        g.addWidget(self.lbl_ol_thresh, r, 2)
        self.spin_ol_thresh = self._spin(0, 254, 30)
        g.addWidget(self.spin_ol_thresh, r, 3)
        return r + 1

    def _build_baud(self, r):
        g = self.grid
        self.lbl_baud_id = QLabel(STRINGS[self.lang]["lbl_servo_id"])
        g.addWidget(self.lbl_baud_id, r, 0)
        self.spin_baud_id = self._spin(1, 255)
        g.addWidget(self.spin_baud_id, r, 1)
        self.lbl_baud = QLabel(STRINGS[self.lang]["lbl_baud"])
        g.addWidget(self.lbl_baud, r, 2)
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["0: 1M", "1: 500K", "2: 250K", "4: 115200", "5: 76800", "6: 57600", "7: 38400"])
        g.addWidget(self.combo_baud, r, 3)
        self.btn_read_baud = self._btn("btn_read_baud", self.read_baud)
        g.addWidget(self.btn_read_baud, r, 4)
        self.btn_set_baud = self._btn("btn_set_baud", self.set_baud)
        g.addWidget(self.btn_set_baud, r, 5)
        r += 1
        self.btn_set_baud_all = QPushButton(STRINGS[self.lang].get("btn_set_baud_all", "全部设置"))
        self.btn_set_baud_all.clicked.connect(self.set_baud_all)
        g.addWidget(self.btn_set_baud_all, r, 4, 1, 2)
        return r + 1

    def _build_max_torque(self, r):
        g = self.grid
        self.lbl_mt_id = QLabel(STRINGS[self.lang]["lbl_servo_id"])
        g.addWidget(self.lbl_mt_id, r, 0)
        self.spin_mt_id = self._spin(1, 255)
        g.addWidget(self.spin_mt_id, r, 1)
        self.lbl_mt_val = QLabel(STRINGS[self.lang]["lbl_max_torque"])
        g.addWidget(self.lbl_mt_val, r, 2)
        self.spin_mt_val = self._spin(0, 1000, 1000)
        g.addWidget(self.spin_mt_val, r, 3)
        self.btn_read_mt = self._btn("btn_read_max_torque", self.read_max_torque)
        g.addWidget(self.btn_read_mt, r, 4)
        self.btn_set_mt = self._btn("btn_set_max_torque", self.set_max_torque)
        g.addWidget(self.btn_set_mt, r, 5)
        return r + 1

    # ── Signal connections ─────────────────────────────────────

    def connect_signals(self):
        self.comm_manager.servo_offset_received.connect(self.update_servo_offset)
        self.comm_manager.servo_pid_received.connect(self.update_servo_pid)
        self.comm_manager.servo_overload_received.connect(self.update_overload)
        self.comm_manager.servo_baud_received.connect(self.update_baud)
        self.comm_manager.servo_max_torque_received.connect(self.update_max_torque)

    # ── Actions ────────────────────────────────────────────────

    def _flash_success(self, btn, original_key):
        """按钮变黄显示'设置成功'，1.5秒后恢复"""
        from PyQt5.QtCore import QTimer
        btn.setStyleSheet("background-color: #FA8F01; color: white; font-weight: bold;")
        btn.setText(STRINGS[self.lang].get("msg_set_success", "Set OK"))
        QTimer.singleShot(1500, lambda: self._reset_btn(btn, original_key))

    def _reset_btn(self, btn, key):
        btn.setStyleSheet("")
        btn.setText(STRINGS[self.lang].get(key, key))

    def set_servo_id(self):
        self.comm_manager.send_sys(CMD_SET_SERVO_ID, [self.spin_old_id.value(), self.spin_new_id.value()])
        self._flash_success(self.btn_set_id, "btn_set_id")

    def set_servo_mode(self):
        sid = self.spin_mode_id.value()
        mode = self.combo_mode.currentIndex()
        self.comm_manager.send_sys(CMD_SET_SERVO_MODE, [sid, mode])
        self._flash_success(self.btn_set_mode, "btn_set_mode")

    def set_pos_offset(self):
        sid = self.spin_adv_id.value()
        offset = max(-2047, min(2047, self.spin_offset.value()))
        self.spin_offset.setValue(offset)
        self.comm_manager.send_sys(CMD_SET_POS_OFFSET, list(struct.pack('<Bh', sid, offset)))
        self.comm_manager.send_sys(CMD_GET_POS_OFFSET, [sid])
        self._flash_success(self.btn_set_offset, "btn_set_offset")

    def _reset_offset_btn(self):
        pass  # handled by _flash_success now

    def _preview_offset(self):
        """回车实时预览偏差值"""
        sid = self.spin_adv_id.value()
        offset = max(-2047, min(2047, self.spin_offset.value()))
        self.spin_offset.setValue(offset)
        self.comm_manager.send_sys(CMD_SET_POS_OFFSET, list(struct.pack('<Bh', sid, offset)))

    def get_pos_offset(self):
        sid = self.spin_adv_id.value()
        self.comm_manager.send_sys(CMD_GET_POS_OFFSET, [sid])

    def cali_pos(self):
        sid = self.spin_adv_id.value()
        self.comm_manager.send_sys(CMD_SERVO_CALI_POS, [sid])
        self._flash_success(self.btn_cali_pos, "btn_cali_pos")

    def cali_pos_all(self):
        for sid in range(1, 7):
            self.comm_manager.send_sys(CMD_SERVO_CALI_POS, [sid])
            import time; time.sleep(0.05)
        self._flash_success(self.btn_cali_pos_all, "btn_cali_pos_all")

    def set_pid_param(self):
        sid = self.spin_adv_id.value()
        p, i, d, minf = self.spin_p.value(), self.spin_i.value(), self.spin_d.value(), self.spin_minf.value()
        self.comm_manager.send_sys(CMD_SET_PID_PARAM, list(struct.pack('<BBBBH', sid, p, i, d, minf)))
        self._flash_success(self.btn_set_pid, "btn_set_pid")

    def get_pid_param(self):
        self.comm_manager.send_sys(CMD_GET_PID_PARAM, [self.spin_adv_id.value()])

    def update_servo_offset(self, sid, offset):
        offset = max(-2047, min(2047, int(offset)))
        if self.spin_adv_id.value() == sid:
            self.spin_offset.setValue(offset)

    def update_servo_pid(self, sid, p, i, d, minf):
        if self.spin_adv_id.value() == sid:
            self.spin_p.setValue(p); self.spin_i.setValue(i)
            self.spin_d.setValue(d); self.spin_minf.setValue(minf)

    def read_overload(self):
        self.comm_manager.send_sys(CMD_READ_OVERLOAD, [self.spin_ol_id.value()])

    def set_overload(self):
        self.comm_manager.send_sys(CMD_SET_OVERLOAD, [
            self.spin_ol_id.value(), self.spin_ol_torque.value(),
            self.spin_ol_time.value(), self.spin_ol_thresh.value()
        ])
        self._flash_success(self.btn_set_ol, "btn_set_overload")

    def update_overload(self, sid, torque, t, thresh):
        if self.spin_ol_id.value() == sid:
            self.spin_ol_torque.setValue(torque)
            self.spin_ol_time.setValue(t)
            self.spin_ol_thresh.setValue(thresh)

    def read_baud(self):
        self.comm_manager.send_sys(CMD_READ_BAUD, [self.spin_baud_id.value()])

    def set_baud(self):
        text = self.combo_baud.currentText()
        baud_code = int(text.split(":")[0])
        self.comm_manager.send_sys(CMD_SET_BAUD, [self.spin_baud_id.value(), baud_code])
        self._flash_success(self.btn_set_baud, "btn_set_baud")

    def set_baud_all(self):
        text = self.combo_baud.currentText()
        baud_code = int(text.split(":")[0])
        self.comm_manager.send_sys(CMD_SET_BAUD, [0xFE, baud_code])
        self._flash_success(self.btn_set_baud_all, "btn_set_baud_all")

    def update_baud(self, sid, baud):
        if self.spin_baud_id.value() == sid:
            baud_map = {0: 0, 1: 1, 2: 2, 4: 3, 5: 4, 6: 5, 7: 6}
            idx = baud_map.get(baud, 0)
            self.combo_baud.setCurrentIndex(idx)

    def read_max_torque(self):
        self.comm_manager.send_sys(CMD_READ_MAX_TORQUE, [self.spin_mt_id.value()])

    def set_max_torque(self):
        sid = self.spin_mt_id.value()
        val = self.spin_mt_val.value()
        self.comm_manager.send_sys(CMD_SET_MAX_TORQUE, [sid, val & 0xFF, (val >> 8) & 0xFF])
        self._flash_success(self.btn_set_mt, "btn_set_max_torque")

    def update_max_torque(self, sid, torque):
        if self.spin_mt_id.value() == sid:
            self.spin_mt_val.setValue(torque)

    # ── Language ───────────────────────────────────────────────

    def update_language(self, lang):
        self.lang = lang
        s = STRINGS[lang]
        # Section titles
        if hasattr(self, '_section_labels'):
            for lbl, key in self._section_labels:
                lbl.setText(s.get(key, key))
        # Buttons
        self.btn_set_id.setText(s["btn_set_id"])
        self.btn_set_mode.setText(s["btn_set_mode"])
        self.btn_set_offset.setText(s["btn_set_offset"])
        self.btn_get_offset.setText(s["btn_get_offset"])
        self.btn_set_pid.setText(s["btn_set_pid"])
        self.btn_get_pid.setText(s["btn_get_pid"])
        self.btn_read_ol.setText(s["btn_read_overload"])
        self.btn_set_ol.setText(s["btn_set_overload"])
        self.btn_read_baud.setText(s["btn_read_baud"])
        self.btn_set_baud.setText(s["btn_set_baud"])
        self.btn_read_mt.setText(s["btn_read_max_torque"])
        self.btn_set_mt.setText(s["btn_set_max_torque"])
        # Labels
        self.lbl_old_id.setText(s["lbl_old_id"])
        self.lbl_new_id.setText(s["lbl_new_id"])
        self.lbl_mode_id.setText(s["lbl_servo_id"])
        self.lbl_mode.setText(s["lbl_mode"])
        self.lbl_adv_id.setText(s["lbl_servo_id"])
        self.lbl_offset.setText(s["lbl_offset"])
        self.lbl_pid.setText(s["lbl_pid"])
        self.lbl_ol_id.setText(s["lbl_servo_id"])
        self.lbl_ol_torque.setText(s["lbl_overload_torque"])
        self.lbl_ol_time.setText(s["lbl_overload_time"])
        self.lbl_ol_thresh.setText(s.get("lbl_overload_thresh", "Torque Threshold"))
        self.lbl_baud_id.setText(s["lbl_servo_id"])
        self.lbl_baud.setText(s["lbl_baud"])
        self.lbl_mt_id.setText(s["lbl_servo_id"])
        self.lbl_mt_val.setText(s["lbl_max_torque"])
        modes = s.get("servo_modes", ["0: Position", "1: Wheel", "2: PWM", "3: Stepper"])
        for i, m in enumerate(modes):
            if i < self.combo_mode.count():
                self.combo_mode.setItemText(i, m)
        self.btn_cali_pos.setText(s.get("btn_cali_pos", "Calibrate Center"))
        self.btn_cali_pos_all.setText(s.get("btn_cali_pos_all", "Calibrate All"))
        self.btn_set_baud_all.setText(s.get("btn_set_baud_all", "Set All"))
