import json
import os
import re
import struct
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QDoubleSpinBox, QGroupBox
)
from nexarm_qt.translations import STRINGS

class AICalibrationWidget(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.config_file = 'offset_config.json'
        self.log_buffer = ""
        
        # Default values
        self.cfg_x = 15.0
        self.cfg_y = 50.0
        self.cfg_z = 60.0
        
        self.load_local_config()
        self.setup_ui()
        self.connect_signals()

    def load_local_config(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    cfg = json.load(f)
                    self.cfg_x = cfg.get('x', 15.0)
                    self.cfg_y = cfg.get('y', 50.0)
                    self.cfg_z = cfg.get('z', 60.0)
            except Exception as e:
                print(f"Failed to load config: {e}")

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Group Box
        self.grp_box = QGroupBox(STRINGS[self.lang].get("grp_ai_calib", "Visual Offset Calibration"))
        grp_layout = QVBoxLayout(self.grp_box)
        
        # Deviation Display - 放在 grid 里和 XYZ 对齐
        from PyQt5.QtWidgets import QGridLayout
        grid = QGridLayout()
        grid.setVerticalSpacing(10)
        grid.setHorizontalSpacing(8)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 1)
        grid.setColumnStretch(4, 1)
        
        self.lbl_ex = QLabel('EX: --')
        self.lbl_ey = QLabel('EY: --')
        self.lbl_status = QLabel(STRINGS[self.lang].get("lbl_waiting", "Waiting..."))
        
        font_style = "font-size: 11pt; font-weight: bold; color: #FA8F01;"
        self.lbl_ex.setStyleSheet(font_style)
        self.lbl_ey.setStyleSheet(font_style)
        self.lbl_status.setStyleSheet("font-size: 11pt; font-weight: bold; color: #FA8F01;")
        
        self.lbl_calib_title = QLabel(STRINGS[self.lang].get("lbl_tag_calib", "标签校准"))
        self.lbl_calib_title.setStyleSheet("font-size: 10pt;")
        grid.addWidget(self.lbl_calib_title, 1, 0)
        grid.addWidget(self.lbl_ex, 0, 1)
        grid.addWidget(self.lbl_ey, 0, 2)
        grid.addWidget(self.lbl_status, 0, 3, 1, 2)
        
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-200, 200)
        self.spin_x.setValue(self.cfg_x)
        self.spin_x.setPrefix("X: ")
        
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-200, 200)
        self.spin_y.setValue(self.cfg_y)
        self.spin_y.setPrefix("Y: ")
        
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-200, 200)
        self.spin_z.setValue(self.cfg_z)
        self.spin_z.setPrefix("Z: ")
        
        self.btn_save = QPushButton(STRINGS[self.lang].get("btn_save_apply", "Save & Apply"))
        self.btn_save.clicked.connect(self.save_offsets)
        
        grid.addWidget(self.spin_x, 1, 1)
        grid.addWidget(self.spin_y, 1, 2)
        grid.addWidget(self.spin_z, 1, 3)
        grid.addWidget(self.btn_save, 1, 4)
        
        # Control Buttons - 放在 col1-4 和追踪按钮对齐
        active_style = "QPushButton { background-color: #FA8F01; color: #FFFFFF; }"

        self.btn_start = QPushButton(STRINGS[self.lang].get("btn_calib_start", "1. Start Align"))
        self.btn_start.clicked.connect(lambda: (self.send_cmd(0x01), self._calib_active(self.btn_start)))
        
        self.btn_grab = QPushButton(STRINGS[self.lang].get("btn_calib_grab", "2. Test Grab"))
        self.btn_grab.clicked.connect(lambda: (self.send_cmd(0x02), self._calib_active(self.btn_grab)))
        
        self.btn_reset = QPushButton(STRINGS[self.lang].get("btn_calib_reset", "3. Reset"))
        self.btn_reset.clicked.connect(lambda: (self.send_cmd(0x03), self._calib_inactive()))
        
        self.btn_stop = QPushButton(STRINGS[self.lang].get("btn_calib_stop", "4. Stop"))
        self.btn_stop.clicked.connect(lambda: (self.send_cmd(0x00), self._calib_inactive()))
        
        grid.addWidget(self.btn_start, 2, 1)
        grid.addWidget(self.btn_grab, 2, 2)
        grid.addWidget(self.btn_reset, 2, 3)
        grid.addWidget(self.btn_stop, 2, 4)
        
        grp_layout.addLayout(grid)
        layout.addWidget(self.grp_box)

    def update_language(self, lang):
        self.lang = lang
        self.grp_box.setTitle(STRINGS[lang].get("grp_ai_calib", "Visual Offset Calibration"))
        self.lbl_calib_title.setText(STRINGS[lang].get("lbl_tag_calib", "标签校准"))
        self.btn_save.setText(STRINGS[lang].get("btn_save_apply", "Save & Apply"))
        self.btn_start.setText(STRINGS[lang].get("btn_calib_start", "1. Start Align"))
        self.btn_grab.setText(STRINGS[lang].get("btn_calib_grab", "2. Test Grab"))
        self.btn_reset.setText(STRINGS[lang].get("btn_calib_reset", "3. Reset"))
        self.btn_stop.setText(STRINGS[lang].get("btn_calib_stop", "4. Stop"))
        self.lbl_status.setText(STRINGS[lang].get("lbl_waiting", "Waiting..."))

    def connect_signals(self):
        self.comm_manager.log_message_received.connect(self.process_log_chunk)

    def process_log_chunk(self, content):
        self.log_buffer += content
        while '\n' in self.log_buffer:
            line, self.log_buffer = self.log_buffer.split('\n', 1)
            self.process_line(line.strip())

    def process_line(self, line):
        try:
            if "[校准]" in line:
                # Format 1: Aligned Center (X偏:... Y偏:...)
                match_center = re.search(r"X偏:(-?\d+) Y偏:(-?\d+)", line)
                if match_center:
                    ex = int(match_center.group(1))
                    ey = int(match_center.group(2))
                    self.lbl_ex.setText(f"EX: {ex} px")
                    self.lbl_ey.setText(f"EY: {ey} px")
                    self.lbl_status.setText(STRINGS[self.lang].get("status_aligned", "Aligned"))
                    self.lbl_status.setStyleSheet("font-size: 11pt; font-weight: bold; color: #FA8F01;") # Orange
                else:
                    # Format 2: Adjusting (... pixels | ... pixels)
                    match_dev = re.search(r"\[校准\] (.*?) (\d+) 像素 \| (.*?) (\d+) 像素", line)
                    if match_dev:
                        x_dir, x_val, y_dir, y_val = match_dev.groups()
                        self.lbl_ex.setText(f"EX: {x_val} px")
                        self.lbl_ey.setText(f"EY: {y_val} px")
                        self.lbl_status.setText(STRINGS[self.lang].get("status_adjusting", "Adjusting..."))
                        self.lbl_status.setStyleSheet("font-size: 11pt; font-weight: bold; color: #FA8F01;") # Orange
        except Exception:
            pass

    def send_cmd(self, sub_cmd):
        # 0x31 (49) is the command ID for calibration control
        self.comm_manager.send_packet(0xFF, 0x31, [sub_cmd])

    def _calib_active(self, btn):
        active = "QPushButton { background-color: #FA8F01; color: #FFFFFF; }"
        btn.setStyleSheet(active)

    def _calib_inactive(self):
        self.btn_start.setStyleSheet("")
        self.btn_grab.setStyleSheet("")

    def save_offsets(self):
        self._calib_inactive()
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        
        # Send to ESP32 (CMD 45)
        data_bytes = struct.pack('<fff', x, y, z)
        data_list = list(data_bytes)
        
        self.comm_manager.send_packet(0xFF, 45, data_list + [0x00, 0x00])
        
        # Save local
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump({'x': x, 'y': y, 'z': z}, f)
            print("Saved config locally")
        except Exception as e:
            print(f"Failed to save local config: {e}")
