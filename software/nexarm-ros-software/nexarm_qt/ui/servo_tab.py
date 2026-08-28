from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QPushButton,
    QLabel, QSlider, QSpinBox, QTreeWidget, QTreeWidgetItem,
    QHeaderView, QFileDialog, QMessageBox, QFrame, QScrollArea, QGridLayout, QSplitter,
    QSizePolicy, QAbstractSpinBox
)
from PyQt5.QtCore import Qt, QSize, QTimer, pyqtSignal
import sys

class JumpSlider(QSlider):
    def mousePressEvent(self, ev):
        super().mousePressEvent(ev)
from PyQt5.QtGui import QPixmap, QFont, QColor, QBrush
import time
import json
import threading
from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S

class ResizableLabel(QLabel):
    def __init__(self, parent=None):

        
        super().__init__(parent)
        self.original_pixmap = None
        self.setMinimumSize(50, 50)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def setPixmap(self, pixmap):
        self.original_pixmap = pixmap
        self.update_scaled_pixmap()

    def update_scaled_pixmap(self):
        if self.original_pixmap and not self.original_pixmap.isNull():
            # Use self.contentsRect() to account for margins
            sz = self.contentsRect().size()
            if not sz.isEmpty():
                scaled = self.original_pixmap.scaled(sz, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                super().setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_scaled_pixmap()

class ServoControlWidget(QFrame):
    # 每个舵机ID的角度转换参数: (pos_min, pos_max, center, deg_range)
    # center=0°对应的pos值, deg_range=总角度范围
    SERVO_ANGLE_PARAMS = {
        1: (0, 4096, 2048, 360.0),
        2: (0, 4096, 2048, 360.0),
        3: (0, 4096, 2048, 360.0),
        4: (0, 4096, 2048, 360.0),
        5: (0, 4096, 2048, 240.0),
        6: (0, 4096, 2048, 240.0),
    }

    SERVO_RANGES = {
        1: (341, 3755),
        2: (1195, 2901),
        3: (1024, 3072),
        4: (683, 3413),
        5: (1024, 3072),
        6: (1024, 2731),
    }
        # 1: (341, 3755),
        # 2: (1195, 2901),
        # 3: (1024, 3072),
        # 4: (683, 3413),
        # 5: (1024, 3072),
        # 6: (1024, 2731),
    def __init__(self, id_val, parent=None):
        super().__init__(parent)
        self.id_val = id_val
        lo, hi = self.SERVO_RANGES.get(id_val, (0, 4096))
        params = self.SERVO_ANGLE_PARAMS.get(id_val, (0, 4096, 2048, 360.0))
        self._angle_center = params[2]
        self._angle_range = params[3]
        self._angle_pos_range = params[1] - params[0]
        self.setObjectName("ServoControlBox")
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        # DPI 缩放 — AA_EnableHighDpiScaling 已自动处理
        self._dpi_s = 1.0
        self.setFixedWidth(int(240 * self._dpi_s))
        # Styles moved to styles.py via object name ServoControlBox
        
        main_vbox = QVBoxLayout(self); main_vbox.setContentsMargins(0, 12, 12, 12); main_vbox.setSpacing(12)
        

        # Top row: ID and Torque button
        top = QHBoxLayout()
        self.lbl_id = QLabel(f"ID: {id_val}")
        self.lbl_id.setProperty("class", "title-text")
        self.lbl_id.setStyleSheet("QLabel { color: #FA8F01; font-size: 12pt; font-weight: bold; }")
        top.addWidget(self.lbl_id)
        top.addStretch()
        
        self.btn_torque = QPushButton("ON")
        self.btn_torque.setCheckable(True)
        self.btn_torque.setChecked(True)
        self.btn_torque.toggled.connect(lambda checked: self.btn_torque.setText("ON" if checked else "OFF"))
        self.btn_torque.setFixedSize(int(75 * self._dpi_s), int(30 * self._dpi_s))
        self.btn_torque.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: rgba(255, 255, 255, 0.8);
                border-radius: 12px;
                font-size: 9pt;
                font-weight: bold;
                border: 1px solid rgba(255, 255, 255, 0.4);
                font-family: "Microsoft YaHei", "PingFang SC", "Segoe UI", sans-serif;
            }
            QPushButton:checked {
                background-color: #FA8F01;
                color: #FFFFFF;
                border: none;
            }
        """)
        top.addWidget(self.btn_torque)
        main_vbox.addLayout(top)
        
        # Slider
        self.slider = JumpSlider(Qt.Horizontal)
        self.slider.setRange(lo, hi)
        self.slider.setValue((lo + hi) // 2)
        self.slider.setFixedHeight(int(20 * self._dpi_s))
        main_vbox.addWidget(self.slider)
        
        
        # Grid layout for parameters
        grid = QGridLayout()
        grid.setSpacing(8)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        
        # Row 0: Pos + Ang (aligned with Acc + Spd below)
        lbl_pos = QLabel("Pos:")
        lbl_pos.setProperty("class", "servo-label")
        lbl_pos.setFixedWidth(int(35 * self._dpi_s))
        lbl_pos.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(lbl_pos, 0, 0)
        self.spin_pos = QSpinBox()
        self.spin_pos.setProperty("class", "servo-spin")
        self.spin_pos.setRange(lo, hi)
        self.spin_pos.setValue((lo + hi) // 2)
        self.spin_pos.setButtonSymbols(QSpinBox.NoButtons)
        self.spin_pos.setAlignment(Qt.AlignCenter)
        self.spin_pos.setFixedHeight(int(28 * self._dpi_s))
        self.spin_pos.setMinimumWidth(int(65 * self._dpi_s))
        self.spin_pos.setKeyboardTracking(False)
        grid.addWidget(self.spin_pos, 0, 1)

        lbl_ang = QLabel("Ang:")
        lbl_ang.setProperty("class", "servo-label")
        lbl_ang.setFixedWidth(int(35 * self._dpi_s))
        lbl_ang.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(lbl_ang, 0, 2)
        self.lbl_angle_val = QLabel("0.0°")
        self.lbl_angle_val.setAlignment(Qt.AlignCenter)
        self.lbl_angle_val.setFixedHeight(int(28 * self._dpi_s))
        self.lbl_angle_val.setMinimumWidth(int(65 * self._dpi_s))
        self.lbl_angle_val.setStyleSheet(
            "QLabel { color: #4FC3F7; font-size: 10pt;"
            " background: rgba(255,255,255,0.05);"
            " border: 1px solid rgba(255,255,255,0.15);"
            " border-radius: 4px; }")
        grid.addWidget(self.lbl_angle_val, 0, 3)
        
        # Acc & Spd row
        lbl_acc = QLabel("Acc:")
        lbl_acc.setProperty("class", "servo-label")
        lbl_acc.setFixedWidth(int(35 * self._dpi_s))
        lbl_acc.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(lbl_acc, 1, 0)
        self.spin_acc = QSpinBox()
        self.spin_acc.setProperty("class", "servo-spin")
        self.spin_acc.setRange(0, 255)
        self.spin_acc.setValue(100)
        self.spin_acc.setAlignment(Qt.AlignCenter)
        self.spin_acc.setFixedHeight(int(24 * self._dpi_s))
        self.spin_acc.setMinimumWidth(int(65 * self._dpi_s))
        self.spin_acc.setKeyboardTracking(False)
        grid.addWidget(self.spin_acc, 1, 1)
        
        lbl_spd = QLabel("Spd:")
        lbl_spd.setProperty("class", "servo-label")
        lbl_spd.setFixedWidth(int(35 * self._dpi_s))
        lbl_spd.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(lbl_spd, 1, 2)
        self.spin_spd = QSpinBox()
        self.spin_spd.setProperty("class", "servo-spin")
        self.spin_spd.setRange(0, 3400)
        self.spin_spd.setValue(2000)
        self.spin_spd.setAlignment(Qt.AlignCenter)
        self.spin_spd.setFixedHeight(int(24 * self._dpi_s))
        self.spin_spd.setMinimumWidth(int(75 * self._dpi_s))
        self.spin_spd.setKeyboardTracking(False)
        self.spin_spd.setToolTip(STRINGS.get('zh', {}).get("tip_servo_spd", ""))
        grid.addWidget(self.spin_spd, 1, 3)
        
        main_vbox.addLayout(grid)

        self.spin_pos.valueChanged.connect(self._update_angle_display)
        self._update_angle_display(self.spin_pos.value())

    def pos_to_angle(self, pos):
        offset = pos - self._angle_center
        if self._angle_pos_range > 0:
            return offset / self._angle_pos_range * self._angle_range
        return 0.0

    def _update_angle_display(self, pos):
        ang = self.pos_to_angle(pos)
        self.lbl_angle_val.setText(f"{ang:+.1f}°")

class ServoTab(QWidget):
    online_run_finished = pyqtSignal()
    online_run_step_changed = pyqtSignal(int)

    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.action_data_list, self.is_updating_ui = [], False
        self.servo_vals, self.servo_acc, self.servo_spd = [2048]*6, [100]*6, [2000]*6
        self.sliders, self.spin_pos, self.spin_acc, self.spin_spd, self.servo_widgets = [], [], [], [], []
        self.last_user_input_time = [0] * 6
        self.user_is_dragging = [False] * 6
        self.pending_servo_send = [False] * 6
        self.last_servo_send_time = [0.0] * 6
        self.range_min = [4096] * 6
        self.range_max = [0] * 6
        self.manual_read_refresh_active = False
        self.manual_read_pending_ids = set()
        self.latest_actual_positions = [2048] * 6
        self.actual_positions_event = threading.Event()
        self.is_online_running = False
        self.suppress_tree_preview = False
        self._highlight_brush = QBrush(QColor("#FFFFFF"))
        self._highlight_bg_brush = QBrush(QColor(250, 143, 1, 110))
        self.servo_send_timer = QTimer(self)
        self.servo_send_timer.timeout.connect(self.flush_pending_servo_cmds)
        self.servo_send_timer.start(50)
        self.setStyleSheet("background-color: #1E1F31;")
        self.setup_ui()
        self.comm_manager.packet_received.connect(self.on_packet_received)
        self.online_run_finished.connect(self._show_online_run_finished)
        self.online_run_step_changed.connect(self._highlight_action_row)

    def on_packet_received(self, id_val, cmd, data):
        # 只在用户主动点击“读取位置”后，才接受真实舵机位置回写到界面。
        if 1 <= id_val <= 6 and len(data) == 2 and cmd == 0:
            import struct
            try:
                pos = struct.unpack('<h', data)[0]
                if pos < 0:
                    pos += 65536
                if 0 <= pos <= 4096:
                    idx = id_val - 1
                    self.latest_actual_positions[idx] = pos
                    self.actual_positions_event.set()
                    if self.manual_read_refresh_active:
                        self.is_updating_ui = True
                        self.spin_pos[idx].setValue(pos)
                        self.sliders[idx].setValue(pos)
                        self.servo_vals[idx] = pos
                        self.servo_widgets[idx]._update_angle_display(pos)
                        self.is_updating_ui = False
                if self.manual_read_refresh_active:
                    self.manual_read_pending_ids.discard(id_val)
                    if not self.manual_read_pending_ids:
                        self.manual_read_refresh_active = False
            except:
                pass
        elif (id_val == 0xFF or id_val == 0x5A) and cmd == CMD_GET_REAL_JOINT_ANGLES and len(data) >= 24:
            import struct
            positions = None
            try:
                positions = []
                for i in range(6):
                    base = i * 4
                    pos, _angle_x10 = struct.unpack('<hh', data[base:base + 4])
                    if pos < 0:
                        pos += 65536
                    if 0 <= pos <= 4096:
                        positions.append(pos)
                    else:
                        positions.append(self.latest_actual_positions[i])
                self.latest_actual_positions = positions
                self.actual_positions_event.set()

                if self.manual_read_refresh_active:
                    self.is_updating_ui = True
                    for i, pos in enumerate(positions):
                        self.spin_pos[i].setValue(pos)
                        self.sliders[i].setValue(pos)
                        self.servo_vals[i] = pos
                        self.servo_widgets[i]._update_angle_display(pos)
                    self.is_updating_ui = False
            except:
                pass
            if self.manual_read_refresh_active:
                self.manual_read_pending_ids.clear()
                self.manual_read_refresh_active = False

    def update_language(self, lang):
        self.lang = lang
        self.grp_ctrl.setTitle(STRINGS[lang]["grp_servo_ctrl"])
        self.btn_home.setText(STRINGS[lang]["btn_all_home"]); self.btn_all_off.setText(STRINGS[lang]["btn_all_off"]); self.btn_read.setText(STRINGS[lang]["btn_read_pos"])
        if hasattr(self, 'btn_cali'): self.btn_cali.setText(STRINGS[lang].get("btn_cali_pos", "Calibrate Center"))
        if hasattr(self, 'btn_all_on'): self.btn_all_on.setText(STRINGS[lang].get("btn_all_on", "全部上电"))
        self.grp_act.setTitle(STRINGS[lang]["grp_action_editor"])
        self.tree.setHeaderLabels([STRINGS[lang][k] for k in ["tree_idx", "tree_time"]] + ["S1", "S2", "S3", "S4", "S5", "S6"])
        self.lbl_frame_time.setText(STRINGS[lang]["lbl_frame_time"])
        self.lbl_frame_time.setToolTip(STRINGS[lang].get("tip_frame_time", ""))
        self.ent_delay.setToolTip(STRINGS[lang].get("tip_frame_time", ""))
        if hasattr(self, 'lbl_action_speed_note'):
            self.lbl_action_speed_note.setText(STRINGS[lang].get("lbl_action_speed_note", ""))
        for k in ["add", "update", "del", "clear", "up", "down"]:
            b = getattr(self, f"btn_{k}", None)
            if b:
                if k == 'del': b.setText(STRINGS[lang]["btn_delete"])
                elif k == 'up': b.setText(STRINGS[lang].get("btn_up", "上移"))
                elif k == 'down': b.setText(STRINGS[lang].get("btn_down", "下移"))
                else: b.setText(STRINGS[lang][f"btn_{k}"])
        self.btn_save.setText(STRINGS[lang]["btn_save"]); self.btn_load.setText(STRINGS[lang]["btn_load"])
        if hasattr(self, 'lbl_loop'): self.lbl_loop.setText(STRINGS[lang].get("lbl_loop", "Loop:"))
        if hasattr(self, 'btn_stop_online'): self.btn_stop_online.setText(STRINGS[lang].get("btn_stop", "Stop"))
        self.grp_run.setTitle(STRINGS[lang]["grp_run_download"]); self.btn_run_online.setText(STRINGS[lang]["btn_run_online"])
        self.lbl_group_id.setText(STRINGS[lang]["lbl_group_id"])
        for k in ["dl", "run_board", "erase", "stop"]:
            b = getattr(self, f"btn_{k}", None)
            if b: b.setText(STRINGS[lang][f"btn_{'download' if k=='dl' else k}"])
        for w in getattr(self, 'servo_widgets', []):
            if hasattr(w, 'spin_spd'):
                w.spin_spd.setToolTip(STRINGS[lang].get("tip_servo_spd", ""))
        self.calc_total_time()

    def setup_ui(self):
        main_layout = QVBoxLayout(self); main_layout.setContentsMargins(15, 15, 15, 15); main_layout.setSpacing(15)
        self.splitter = QSplitter(Qt.Horizontal); self.splitter.setStyleSheet("QSplitter::handle { background-color: transparent; width: 6px; } QSplitter::handle:hover { background-color: rgba(255, 255, 255, 0.05); }")
        self.left_pane = QWidget()
        self.left_pane_layout = QVBoxLayout(self.left_pane)
        self.left_pane_layout.setContentsMargins(0, 0, 0, 0)
        self.left_pane_layout.setSpacing(10)
        
        self.grp_ctrl = QGroupBox(STRINGS[self.lang]["grp_servo_ctrl"])
        left_layout = QVBoxLayout(self.grp_ctrl); left_layout.setContentsMargins(5, 5, 5, 5)
        c_lay = QHBoxLayout(); c_lay.setContentsMargins(0, 0, 0, 0)
        self.img_frame = QFrame()
        self.img_frame.setStyleSheet("background-color: #2D2F3F; border: 1px solid rgba(255,255,255,0.05); border-radius: 12px;")
        self.img_frame.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.img_frame.setMinimumWidth(0)
        img_lay = QVBoxLayout(self.img_frame); img_lay.setContentsMargins(5, 5, 5, 5)

        btn_row = QHBoxLayout(); btn_row.setContentsMargins(5, 5, 5, 0); btn_row.setSpacing(10)
        top_buttons = []
        self.btn_home = QPushButton(STRINGS[self.lang]["btn_all_home"]); self.btn_home.setObjectName("btn_home"); self.btn_home.clicked.connect(self.servo_home); top_buttons.append(self.btn_home)
        self.btn_all_on = QPushButton(STRINGS[self.lang].get("btn_all_on", "全部上电")); self.btn_all_on.setObjectName("btn_all_on"); self.btn_all_on.clicked.connect(self.servo_torque_on_all); top_buttons.append(self.btn_all_on)
        self.btn_all_off = QPushButton(STRINGS[self.lang]["btn_all_off"]); self.btn_all_off.setObjectName("btn_all_off"); self.btn_all_off.clicked.connect(self.servo_torque_off_all); top_buttons.append(self.btn_all_off)
        self.btn_read = QPushButton(STRINGS[self.lang]["btn_read_pos"]); self.btn_read.setObjectName("btn_read"); self.btn_read.clicked.connect(self.read_and_sync); top_buttons.append(self.btn_read)
        self.btn_cali = QPushButton(STRINGS[self.lang].get("btn_cali_pos", "设置中位")); self.btn_cali.setObjectName("btn_cali"); self.btn_cali.clicked.connect(self.servo_cali_all); top_buttons.append(self.btn_cali)
        btn_style = "QPushButton { background-color: rgba(255, 255, 255, 0.1); border: none; border-radius: 4px; color: #FFFFFF; padding: 0 12px; } QPushButton:hover { background-color: #FA8F01; color: #FFFFFF; }"
        for btn in top_buttons:
            btn.setStyleSheet(btn_style)
            btn.setMinimumSize(40, 38)
            btn.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
            btn_row.addWidget(btn)
        btn_row.addStretch(); img_lay.addLayout(btn_row)

        self.img_label = ResizableLabel()
        self.img_label.setAlignment(Qt.AlignCenter)
        import sys, os
        res_path = os.path.join(sys._MEIPASS, "0.842.png") if hasattr(sys, '_MEIPASS') else "0.842.png"
        pix = QPixmap(res_path)
        if not pix.isNull():
            self.img_label.setPixmap(pix)
        img_lay.addWidget(self.img_label)
        self.lbl_hint = QLabel(STRINGS[self.lang].get("hint_servo_id", "提示: 舵机ID从下往上依次为 1-6"))
        self.lbl_hint.setStyleSheet("color: #B0BEC5; font-size: 9pt; margin-top: 5px;")
        self.lbl_hint.setAlignment(Qt.AlignCenter)
        img_lay.addWidget(self.lbl_hint)
        id_scroll = QScrollArea(); id_scroll.setWidgetResizable(True); id_scroll.setStyleSheet("background: transparent; border: 1px solid #4A4D5E; border-radius: 12px;"); id_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded); id_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        id_cont = QWidget(); id_cont.setStyleSheet("background: transparent;"); id_grid = QGridLayout(id_cont); id_grid.setSpacing(15); id_grid.setContentsMargins(10, 0, 10, 0)
        for i in range(6):
            w = ServoControlWidget(i+1)
            w.slider.valueChanged.connect(lambda v, x=i: self.on_servo_slide(x, v))
            w.slider.sliderPressed.connect(lambda x=i: self.on_slider_pressed(x))
            w.slider.sliderReleased.connect(lambda x=i: self.on_slider_released(x))
            w.spin_pos.valueChanged.connect(lambda v, x=i: self.on_spin_pos_change(x, v)); w.spin_acc.valueChanged.connect(lambda v, x=i: self.update_params(x, acc=v)); w.spin_spd.valueChanged.connect(lambda v, x=i: self.update_params(x, spd=v)); w.btn_torque.clicked.connect(lambda checked, x=i+1: self.send_servo_torque(x, 1 if checked else 0))
            self.servo_widgets.append(w); self.sliders.append(w.slider); self.spin_pos.append(w.spin_pos); self.spin_acc.append(w.spin_acc); self.spin_spd.append(w.spin_spd); id_grid.addWidget(w, i // 2, i % 2)
        id_scroll.setWidget(id_cont)
        # 用 QSplitter 让图片和舵机ID框可以拖动调整大小
        c_splitter = QSplitter(Qt.Horizontal)
        c_splitter.setObjectName("imgSplitter")
        c_splitter.setStyleSheet("""
            QSplitter#imgSplitter::handle {
                background-color: #4A4D5E;
                width: 3px;
            }
            QSplitter#imgSplitter::handle:hover {
                background-color: #6A6D7E;
            }
        """)
        c_splitter.setHandleWidth(3)
        c_splitter.addWidget(self.img_frame)
        c_splitter.addWidget(id_scroll)
        c_splitter.setStretchFactor(0, 2)
        c_splitter.setStretchFactor(1, 3)
        # 让图片区域可以被压缩到很小
        self.img_frame.setMinimumWidth(0)
        c_splitter.setCollapsible(0, True)
        c_splitter.setCollapsible(1, False)
        self.c_splitter = c_splitter
        c_splitter.splitterMoved.connect(lambda pos, idx: None)
        c_lay.addWidget(c_splitter); left_layout.addLayout(c_lay)
        self.left_pane_layout.addWidget(self.grp_ctrl, 1)

        right_w = QWidget(); right_lay = QVBoxLayout(right_w); right_lay.setContentsMargins(0, 0, 0, 0); right_lay.setSpacing(15)
        self.grp_act = QGroupBox(STRINGS[self.lang]["grp_action_editor"]);
        act_lay = QVBoxLayout(self.grp_act); act_lay.setContentsMargins(5, 5, 5, 5); self.tree = QTreeWidget(); self.tree.setHeaderLabels([STRINGS[self.lang][k] for k in ["tree_idx", "tree_time"]] + ["S1", "S2", "S3", "S4", "S5", "S6"]);
        self.tree.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding);
        self.tree.setStyleSheet("""
            QTreeWidget {
                background-color: #2D2F3F;
                border: 1px solid #4A4D5E;
                border-radius: 0px;
                color: white;
                font-size: 10pt;
            }
            QTreeWidget::item {
                padding: 6px;
                border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            }
            QHeaderView::section {
                background-color: #343645;
                color: #B0BEC5;
                padding: 8px;
                border: none;
                font-weight: bold;
                font-size: 10pt;
            }
        """)
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for i in range(1, 8): self.tree.header().setSectionResizeMode(i, QHeaderView.Stretch)
        self.tree.itemSelectionChanged.connect(self.on_tree_select); act_lay.addWidget(self.tree, 1)
        e_row = QHBoxLayout(); self.lbl_frame_time = QLabel(STRINGS[self.lang]["lbl_frame_time"]); self.lbl_frame_time.setToolTip(STRINGS[self.lang].get("tip_frame_time", "")); e_row.addWidget(self.lbl_frame_time); self.ent_delay = QSpinBox(); self.ent_delay.setRange(10, 10000); self.ent_delay.setValue(1000); self.ent_delay.setToolTip(STRINGS[self.lang].get("tip_frame_time", "")); e_row.addWidget(self.ent_delay); act_lay.addLayout(e_row)
        self.lbl_action_speed_note = QLabel(STRINGS[self.lang].get("lbl_action_speed_note", ""))
        self.lbl_action_speed_note.setWordWrap(True)
        self.lbl_action_speed_note.setStyleSheet("QLabel { color: #B0BEC5; font-size: 9pt; padding: 2px 4px 6px 4px; }")
        act_lay.addWidget(self.lbl_action_speed_note)
        b_grid = QGridLayout(); methods = {"add": "act_add", "update": "act_update", "del": "act_del", "clear": "act_clear", "up": "act_up", "down": "act_down"}
        for i, (k, m) in enumerate(methods.items()):
            label = STRINGS[self.lang].get(f"btn_{k}", "")
            if k == 'del': label = STRINGS[self.lang]["btn_delete"]
            if k == 'up': label = STRINGS[self.lang].get("btn_up", "上移")
            if k == 'down': label = STRINGS[self.lang].get("btn_down", "下移")
            b = QPushButton(label); b.clicked.connect(getattr(self, m)); b.setMinimumHeight(38); setattr(self, f"btn_{k}", b); b_grid.addWidget(b, i//2, i%2)
        act_lay.addLayout(b_grid); f_row = QHBoxLayout(); self.btn_save = QPushButton(STRINGS[self.lang]["btn_save"]); self.btn_save.clicked.connect(self.act_save); f_row.addWidget(self.btn_save); self.btn_load = QPushButton(STRINGS[self.lang]["btn_load"]); self.btn_load.clicked.connect(self.act_load); f_row.addWidget(self.btn_load); act_lay.addLayout(f_row)
        self.lbl_total_time = QLabel(STRINGS[self.lang]["lbl_total_time"].format(0)); self.lbl_total_time.setStyleSheet("QLabel { font-weight: bold; color: #FA8F01; font-size: 11pt; }"); act_lay.addWidget(self.lbl_total_time); right_lay.addWidget(self.grp_act, 1)
        self.grp_run = QGroupBox(STRINGS[self.lang]["grp_run_download"])
        # Style handled by styles.py
        run_lay = QVBoxLayout(self.grp_run)
        self.btn_run_online = QPushButton(STRINGS[self.lang]["btn_run_online"])
        self.btn_run_online.setMinimumHeight(S(38))
        self.btn_run_online.clicked.connect(self.act_run_online)
        online_row = QGridLayout()
        online_row.addWidget(self.btn_run_online, 0, 0)
        self.sb_loop = QSpinBox()
        self.sb_loop.setRange(0, 9999)
        self.sb_loop.setValue(1)
        self.sb_loop.setToolTip("0=∞, 1=1x, 2=2x...")
        self.lbl_loop = QLabel(STRINGS[self.lang].get("lbl_loop", "循环:"))
        online_row.addWidget(self.lbl_loop, 0, 1)
        online_row.addWidget(self.sb_loop, 0, 2)
        self.btn_stop_online = QPushButton(STRINGS[self.lang].get("btn_stop", "停止"))
        self.btn_stop_online.setMinimumHeight(S(38))
        self.btn_stop_online.setStyleSheet("QPushButton { background-color: #FF8F00; color: white; }")
        self.btn_stop_online.clicked.connect(self._stop_online_loop)
        online_row.addWidget(self.btn_stop_online, 0, 3)
        online_row.setColumnStretch(0, 3)
        online_row.setColumnStretch(1, 0)
        online_row.setColumnStretch(2, 1)
        online_row.setColumnStretch(3, 2)
        run_lay.addLayout(online_row)
        d_grid = QGridLayout()
        self.lbl_group_id = QLabel(STRINGS[self.lang]["lbl_group_id"])
        d_grid.addWidget(self.lbl_group_id, 0, 0)
        self.sb_gid = QSpinBox()
        self.sb_gid.setRange(0, 255)
        d_grid.addWidget(self.sb_gid, 0, 1)
        for i, k in enumerate(["dl", "run_board", "erase", "stop"]):
            b = QPushButton(STRINGS[self.lang][f"btn_{'download' if k=='dl' else k}"])
            b.setMinimumHeight(S(38))
            setattr(self, f"btn_{k}", b)
            d_grid.addWidget(b, (i+2)//2, i%2)
        self.btn_stop.setStyleSheet("QPushButton { background-color: #FF8F00; color: white; }"); self.btn_run_board.clicked.connect(lambda: self.comm_manager.send_sys(CMD_ACTION_GROUP_RUN, [self.sb_gid.value()])); self.btn_erase.clicked.connect(lambda: self.comm_manager.send_sys(CMD_ACTION_GROUP_ERASE, [self.sb_gid.value()])); self.btn_stop.clicked.connect(self.act_stop); self.btn_dl.clicked.connect(self.act_dl); run_lay.addLayout(d_grid); right_lay.addWidget(self.grp_run, 0)
        self.splitter.addWidget(self.left_pane); self.splitter.addWidget(right_w); self.splitter.setStretchFactor(0, 3); self.splitter.setStretchFactor(1, 2)
        self.splitter.setSizes([996, 482])
        self.splitter.splitterMoved.connect(lambda pos, idx: None)
        main_layout.addWidget(self.splitter)
        QTimer.singleShot(100, lambda: self.c_splitter.setSizes([381, 535]))

    def on_slider_pressed(self, idx):
        self.user_is_dragging[idx] = True
        self.last_user_input_time[idx] = time.time()

    def on_slider_released(self, idx):
        self.user_is_dragging[idx] = False
        self.last_user_input_time[idx] = time.time()
        self.queue_servo_cmd(idx, immediate=True)

    def on_servo_slide(self, idx, val):
        if not self.is_updating_ui:
            w = self.servo_widgets[idx]
            if not w.btn_torque.isChecked():
                w.btn_torque.blockSignals(True)
                w.btn_torque.setChecked(True)
                w.btn_torque.setText("ON")
                w.btn_torque.blockSignals(False)
                self.toggle_torque(idx, True)
            self.last_user_input_time[idx] = time.time()
            self.spin_pos[idx].blockSignals(True)
            self.spin_pos[idx].setValue(val)
            self.spin_pos[idx].blockSignals(False)
            self.servo_vals[idx] = val
            self.servo_widgets[idx]._update_angle_display(val)
            self.queue_servo_cmd(idx)

    def on_spin_pos_change(self, idx, val):
        if not self.is_updating_ui:
            # 如果舵机是掉电状态，自动上电
            w = self.servo_widgets[idx]
            if not w.btn_torque.isChecked():
                w.btn_torque.blockSignals(True)
                w.btn_torque.setChecked(True)
                w.btn_torque.setText("ON")
                w.btn_torque.blockSignals(False)
                self.toggle_torque(idx, True)
            self.last_user_input_time[idx] = time.time()
            self.sliders[idx].blockSignals(True)
            self.sliders[idx].setValue(val)
            self.sliders[idx].blockSignals(False)
            self.servo_vals[idx] = val
            self.queue_servo_cmd(idx)

    def update_params(self, idx, acc=None, spd=None):
        if self.is_updating_ui:
            return
        if acc is not None:
            self.servo_acc[idx] = acc
        if spd is not None:
            self.servo_spd[idx] = spd
        self.queue_servo_cmd(idx, immediate=not self.user_is_dragging[idx])

    def queue_servo_cmd(self, idx, immediate=False):
        self.pending_servo_send[idx] = True
        if immediate:
            self.flush_pending_servo_cmds(force_idx=idx)

    def flush_pending_servo_cmds(self, force_idx=None):
        now = time.time()
        for idx in range(6):
            if force_idx is not None and idx != force_idx:
                continue
            if not self.pending_servo_send[idx]:
                continue
            if force_idx is None and (now - self.last_servo_send_time[idx]) < 0.05:
                continue
            self.pending_servo_send[idx] = False
            self.send_servo_cmd(idx + 1)
            self.last_servo_send_time[idx] = now

    def send_servo_cmd(self, id_val):
        idx = id_val - 1
        p, a, s = self.servo_vals[idx], self.servo_acc[idx], self.servo_spd[idx]
        args = [SERVO_REG_ACC, a & 0xFF, p & 0xFF, (p >> 8) & 0xFF, 0, 0, s & 0xFF, (s >> 8) & 0xFF]
        self.comm_manager.send_packet(id_val, SERVO_CMD_WRITE, args)
    def send_servo_torque(self, id_val, enable): self.comm_manager.send_packet(id_val, SERVO_CMD_WRITE, [SERVO_REG_TORQUE, 1 if enable else 0])
    def toggle_torque(self, idx, on):
        self.send_servo_torque(idx + 1, 1 if on else 0)
    def servo_torque_off_all(self):
        self.comm_manager.send_sys(CMD_SET_TORQUE, [0])
        for w in self.servo_widgets:
            w.btn_torque.blockSignals(True)
            w.btn_torque.setChecked(False)
            w.btn_torque.setText("OFF")
            w.btn_torque.blockSignals(False)
    def servo_torque_on_all(self):
        self.comm_manager.send_sys(CMD_SET_TORQUE, [1])
        for w in self.servo_widgets:
            w.btn_torque.blockSignals(True)
            w.btn_torque.setChecked(True)
            w.btn_torque.setText("ON")
            w.btn_torque.blockSignals(False)
    def servo_home(self):
        self.is_updating_ui = True
        for i in range(6):
            self.spin_pos[i].setValue(2048)
            self.sliders[i].setValue(2048)
            self.servo_vals[i] = 2048
            self.servo_widgets[i]._update_angle_display(2048)
            w = self.servo_widgets[i]
            w.btn_torque.blockSignals(True)
            w.btn_torque.setChecked(True)
            w.btn_torque.setText("ON")
            w.btn_torque.blockSignals(False)
        self.is_updating_ui = False

        for i in range(6):
            self.send_servo_cmd(i + 1)
            time.sleep(0.002)
        
    def read_and_sync(self):
        self.pending_servo_send = [False] * 6
        self.manual_read_refresh_active = True
        self.manual_read_pending_ids = {1, 2, 3, 4, 5, 6}
        # 只用真实关节反馈接口，避免两种读取方式冲突导致值不一致
        self.comm_manager.send_sys(CMD_GET_REAL_JOINT_ANGLES)

    def servo_cali_all(self):
        title = STRINGS[self.lang].get("btn_cali_pos", "Calibrate Center")
        msg = STRINGS[self.lang].get("msg_cali_confirm",
            "确定要将当前位置设为所有舵机的中位吗？\n此操作不可撤销！")
        ret = QMessageBox.question(self, title, msg,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        for i in range(1, 7):
            self.comm_manager.send_sys(CMD_SERVO_CALI_POS, [i])
            time.sleep(0.15)

    def _read_all_fallback(self):
        time.sleep(0.1) #
        for i in range(1, 7):
            self.comm_manager.send_packet(i, SERVO_CMD_READ, [SERVO_REG_PRESENT_POS, 2])
            time.sleep(0.05)
        time.sleep(0.1)
        if self.manual_read_refresh_active and not self.manual_read_pending_ids:
            self.manual_read_refresh_active = False

    def _get_actual_positions(self, timeout=0.35):
        self.actual_positions_event.clear()
        self.comm_manager.send_sys(CMD_GET_REAL_JOINT_ANGLES)
        if self.actual_positions_event.wait(timeout):
            return list(self.latest_actual_positions)
        return list(self.latest_actual_positions)

    def _send_frame_preview(self, frame, start_positions=None):
        if frame is None:
            return
        current_pos = list(start_positions) if start_positions is not None else self._get_actual_positions()
        t_ms = max(0, int(frame.get('time', 1000)))
        for i, s in enumerate(frame.get('servos', [])):
            p = int(s.get('pos', 2048))
            a = int(s.get('acc', 0))
            sp = self._calc_time_based_speed(current_pos[i], p, t_ms)
            args = [SERVO_REG_ACC, a & 0xFF, p & 0xFF, (p >> 8) & 0xFF, 0, 0, sp & 0xFF, (sp >> 8) & 0xFF]
            self.comm_manager.send_packet(i + 1, SERVO_CMD_WRITE, args)
            current_pos[i] = p
            time.sleep(0.002)
        self.latest_actual_positions = list(current_pos)

    def _highlight_action_row(self, row_idx):
        self.suppress_tree_preview = True
        try:
            for row in range(self.tree.topLevelItemCount()):
                item = self.tree.topLevelItem(row)
                if item is None:
                    continue
                for col in range(self.tree.columnCount()):
                    if row == row_idx:
                        item.setForeground(col, self._highlight_brush)
                        item.setBackground(col, self._highlight_bg_brush)
                    else:
                        item.setForeground(col, QBrush())
                        item.setBackground(col, QBrush())
            if 0 <= row_idx < self.tree.topLevelItemCount():
                self.tree.setCurrentItem(self.tree.topLevelItem(row_idx))
        finally:
            self.suppress_tree_preview = False

    def _build_action_frame_from_ui(self):
        return {
            'time': max(0, int(self.ent_delay.value())),
            'servos': [
                {
                    'pos': int(self.servo_vals[i]),
                    'acc': int(self.servo_acc[i]),
                }
                for i in range(6)
            ]
        }

    def _normalize_action_frame(self, frame, start_pos=None):
        t_ms = max(0, int(frame.get('time', 1000)))
        servos = frame.get('servos', [])
        normalized_servos = []

        for i in range(6):
            src = servos[i] if i < len(servos) else {}
            default_pos = start_pos[i] if start_pos is not None else 2048
            p = int(src.get('pos', default_pos))
            a = int(src.get('acc', 0))
            normalized_servos.append({'pos': p, 'acc': a})

        return {'time': t_ms, 'servos': normalized_servos}

    def _normalize_action_data_list(self):
        self.action_data_list = [self._normalize_action_frame(frame) for frame in self.action_data_list]

    def act_add(self):
        self.action_data_list.append(self._build_action_frame_from_ui())
        self._normalize_action_data_list()
        self.refresh_tree()
        self.calc_total_time()

    def act_update(self):
        it = self.tree.currentItem()
        if it:
            idx = int(it.text(0)) - 1
            self.action_data_list[idx] = self._build_action_frame_from_ui()
            self._normalize_action_data_list()
            self.refresh_tree()
            self.calc_total_time()
            self.tree.blockSignals(True)
            self.tree.setCurrentItem(self.tree.topLevelItem(idx))
            self.tree.blockSignals(False)

    def act_del(self):
        it = self.tree.currentItem()
        if it:
            idx = int(it.text(0)) - 1
            self.action_data_list.pop(idx)
            self._normalize_action_data_list()
            self.refresh_tree()
            self.calc_total_time()
            if self.action_data_list:
                self.tree.blockSignals(True)
                self.tree.setCurrentItem(self.tree.topLevelItem(min(idx, len(self.action_data_list) - 1)))
                self.tree.blockSignals(False)

    def act_clear(self):
        self.action_data_list = []
        self.refresh_tree()
        self.calc_total_time()

    def act_up(self):
        it = self.tree.currentItem()
        if it:
            idx = int(it.text(0)) - 1
            if idx > 0:
                self.action_data_list[idx], self.action_data_list[idx - 1] = self.action_data_list[idx - 1], self.action_data_list[idx]
                self._normalize_action_data_list()
                self.refresh_tree()
                self.tree.blockSignals(True)
                self.tree.setCurrentItem(self.tree.topLevelItem(idx - 1))
                self.tree.blockSignals(False)

    def act_down(self):
        it = self.tree.currentItem()
        if it:
            idx = int(it.text(0)) - 1
            if idx < len(self.action_data_list) - 1:
                self.action_data_list[idx], self.action_data_list[idx + 1] = self.action_data_list[idx + 1], self.action_data_list[idx]
                self._normalize_action_data_list()
                self.refresh_tree()
                self.tree.blockSignals(True)
                self.tree.setCurrentItem(self.tree.topLevelItem(idx + 1))
                self.tree.blockSignals(False)

    def refresh_tree(self):
        self.tree.blockSignals(True)
        self.tree.clear()
        for i, f in enumerate(self.action_data_list):
            it = QTreeWidgetItem()
            it.setText(0, str(i + 1))
            it.setText(1, str(f['time']))
            for j, s in enumerate(f['servos']):
                it.setText(2 + j, str(s['pos']))
            self.tree.addTopLevelItem(it)
        self.tree.blockSignals(False)

    def sync_current_servo_positions(self):
        """切换到动作组页面时，同步当前舵机位置到滑杆"""
        servos = getattr(self.comm_manager, 'last_servos', None)
        if not servos or len(servos) < 6:
            return
        self.is_updating_ui = True
        for i in range(min(6, len(servos))):
            val = servos[i]
            if i < len(self.sliders):
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(val)
                self.sliders[i].blockSignals(False)
            if i < len(self.spin_pos):
                self.spin_pos[i].blockSignals(True)
                self.spin_pos[i].setValue(val)
                self.spin_pos[i].blockSignals(False)
            self.servo_vals[i] = val
            if i < len(self.servo_widgets):
                self.servo_widgets[i]._update_angle_display(val)
        self.is_updating_ui = False
        # print(f"[Sync] servo positions: {servos[:6]}")

    def on_tree_select(self):
        if self.suppress_tree_preview:
            return
        it = self.tree.currentItem()
        if it:
            try:
                idx = int(it.text(0)) - 1
                if 0 <= idx < len(self.action_data_list):
                    self._normalize_action_data_list()
                    f = self.action_data_list[idx]
                    self.pending_servo_send = [False] * 6
                    self.is_updating_ui = True
                    self.ent_delay.setValue(f['time'])
                    for i, s in enumerate(f['servos']):
                        p, a = s.get('pos', 2048), s.get('acc', 100)
                        self.servo_vals[i], self.servo_acc[i] = p, a
                        self.sliders[i].setValue(p)
                        self.spin_pos[i].setValue(p)
                        self.spin_acc[i].setValue(a)
                        self.spin_spd[i].setValue(s.get('spd', 2000))
                        self.servo_widgets[i]._update_angle_display(p)
                    self.is_updating_ui = False
                    if not self.is_online_running:
                        threading.Thread(target=self._send_frame_preview, args=(f,), daemon=True).start()
            except (ValueError, IndexError):
                pass

    def calc_total_time(self):
        self.lbl_total_time.setText(STRINGS[self.lang]["lbl_total_time"].format(sum([f['time'] for f in self.action_data_list])))

    def _show_online_run_finished(self):
        self._highlight_action_row(-1)
        QMessageBox.information(self, "Run", STRINGS[self.lang]["msg_online_done"])

    def act_save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save Action Group", "", "NexArm Files (*.d6a)")
        if fn:
            try:
                self._normalize_action_data_list()
                with open(fn, 'w') as f:
                    json.dump(self.action_data_list, f)
            except Exception as e:
                QMessageBox.critical(self, STRINGS[self.lang]["msg_error"], str(e))

    def act_load(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Open Action Group", "", "NexArm Files (*.d6a)")
        if fn:
            try:
                with open(fn, 'r') as f:
                    d = json.load(f)
                self.action_data_list = []
                for r in d:
                    t, s = r.get('time', 1000), r.get('servos', [])
                    if not s and isinstance(r, list):
                        s = [{'pos': p, 'acc': 0} for p in r]
                    elif s and isinstance(s[0], int):
                        s = [{'pos': p, 'acc': 0} for p in s]
                    else:
                        s = [{'pos': item.get('pos', 2048), 'acc': item.get('acc', 0)} for item in s]
                    frame = self._normalize_action_frame({'time': t, 'servos': s})
                    self.action_data_list.append(frame)
                self.refresh_tree()
                self.calc_total_time()
            except Exception as e:
                QMessageBox.critical(self, STRINGS[self.lang]["msg_error"], str(e))

    def act_run_online(self):
        if self.action_data_list and not self.is_online_running:
            self._normalize_action_data_list()
            self._online_stop_flag = False
            self._online_loop_count = self.sb_loop.value()
            threading.Thread(target=self._run_thread, daemon=True).start()

    def _stop_online_loop(self):
        """停止在线循环执行"""
        self._online_stop_flag = True

    def _calc_time_based_speed(self, start_pos, target_pos, move_time_ms):
        distance = abs(int(target_pos) - int(start_pos))
        if move_time_ms > 0:
            calc_speed = int(distance * 1000.0 / float(move_time_ms) * 1.2)
        else:
            calc_speed = 3400
        if calc_speed < 10:
            calc_speed = 10
        if calc_speed > 3400:
            calc_speed = 3400
        return calc_speed

    def _run_thread(self):
        self.is_online_running = True
        loop_target = getattr(self, '_online_loop_count', 1)  # 0=无限, 1=1遍, 2=2遍...
        loop_done = 0
        try:
            while True:
                # 检查停止标志
                if getattr(self, '_online_stop_flag', False):
                    break
                current_pos = self._get_actual_positions()
                for step_idx, f in enumerate(self.action_data_list):
                    if getattr(self, '_online_stop_flag', False):
                        break
                    self.online_run_step_changed.emit(step_idx)
                    t_ms = max(0, int(f.get('time', 1000)))
                    for i, s in enumerate(f['servos']):
                        p = s.get('pos', 2048)
                        a = s.get('acc', 0)
                        sp = self._calc_time_based_speed(current_pos[i], p, t_ms)
                        args = [SERVO_REG_ACC, a & 0xFF, p & 0xFF, (p >> 8) & 0xFF, 0, 0, sp & 0xFF, (sp >> 8) & 0xFF]
                        self.comm_manager.send_packet(i + 1, SERVO_CMD_WRITE, args)
                        current_pos[i] = p
                        time.sleep(0.002)
                    # 等待动作执行完，期间检查停止标志
                    wait_end = time.time() + t_ms / 1000.0
                    while time.time() < wait_end:
                        if getattr(self, '_online_stop_flag', False):
                            break
                        time.sleep(0.05)
                self.latest_actual_positions = list(current_pos)
                loop_done += 1
                # 检查是否达到循环次数
                if loop_target > 0 and loop_done >= loop_target:
                    break
        finally:
            self.is_online_running = False
        self.online_run_finished.emit()

    def act_stop(self):
        """停止动作组运行"""
        print(f"[STOP] sending CMD_ACTION_GROUP_STOP")
        self.comm_manager.send_sys(CMD_ACTION_GROUP_STOP)

    def act_dl(self):
        if not self.action_data_list:
            return
        self._normalize_action_data_list()
        gid, tot = self.sb_gid.value(), len(self.action_data_list)
        current_pos = self._get_actual_positions()
        for i, f in enumerate(self.action_data_list):
            t = max(0, int(f.get('time', 1000)))
            args = [gid, tot, i + 1, 6, t & 0xFF, (t >> 8) & 0xFF]
            for j, s in enumerate(f['servos']):
                p = s.get('pos', 2048)
                a = s.get('acc', 0)
                sp = self._calc_time_based_speed(current_pos[j], p, t)
                args.extend([j + 1, p & 0xFF, (p >> 8) & 0xFF, a & 0xFF, sp & 0xFF, (sp >> 8) & 0xFF])
                current_pos[j] = p
            self.comm_manager.send_sys(CMD_ACTION_GROUP_DOWNLOAD, args)
            time.sleep(0.15)
        self.latest_actual_positions = list(current_pos)
        QMessageBox.information(self, "Download", STRINGS[self.lang]["msg_download_complete"].format(gid, tot))
        
