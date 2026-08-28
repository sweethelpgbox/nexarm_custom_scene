from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QPushButton, 
    QLabel, QDoubleSpinBox, QSpinBox, QGridLayout, QFrame, QSplitter
)
from PyQt5.QtCore import Qt, QSize, QRectF, pyqtSignal, QPoint, QTimer, QEvent
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen, QBrush, QFont, QRadialGradient, QPolygonF
from PyQt5 import uic
import struct
import os
import math
from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S

# Try to import 3D widget (graceful fallback if OpenGL not available)
try:
    from nexarm_qt.ui.arm_3d_widget import Arm3DWidget
    HAS_3D = True
except Exception:
    HAS_3D = False

class CoordinateSystemViz(QWidget):
    def __init__(self, parent=None, lang='zh'):
        super().__init__(parent)
        self.lang = lang
        self.setFixedSize(S(210), S(210))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPoint(self.width() // 2, self.height() // 2)
        length = 100
        font = QFont("Microsoft YaHei", 10, QFont.Bold)
        painter.setFont(font)
        painter.setPen(QPen(QColor("#ff0000"), 2))
        painter.drawLine(center.x(), center.y(), center.x(), center.y() - length)
        painter.drawLine(center.x(), center.y(), center.x(), center.y() + length)
        painter.drawText(center.x() + 10, center.y() - length + 10, "+z")
        painter.drawText(center.x() + 10, center.y() + length - 5, "-z")
        painter.setPen(QPen(QColor("#0000ff"), 2))
        painter.drawLine(center.x() - length - 30, center.y(), center.x() + length + 30, center.y())
        painter.drawText(center.x() + length + 10, center.y() - 5, "-Y")
        painter.drawText(center.x() - length - 35, center.y() + 20, "+Y")
        painter.setPen(QPen(QColor("#00aa00"), 2))
        dy = int(length * 0.8)
        painter.drawLine(center.x() - dy, center.y() + dy, center.x() + dy, center.y() - dy)
        painter.drawText(center.x() + dy + 5, center.y() - dy - 5, "+X")
        painter.drawText(center.x() - dy - 25, center.y() + dy + 25, "-X")
        painter.setBrush(QBrush(Qt.white))
        painter.setPen(QPen(Qt.black, 1))
        painter.drawEllipse(center, 4, 4)

class DirectionWheel(QWidget):
    clicked = pyqtSignal(str)
    def __init__(self, parent=None, lang='zh'):
        super().__init__(parent)
        self.lang = lang
        self.setFixedSize(S(210), S(210))
        self.hovered_seg = None
        self.setMouseTracking(True)
        self.labels = ["X+", "Y-", "X-", "Y+"]
        
        self.timer = QTimer(self)
        self.timer.setInterval(150)
        self.timer.timeout.connect(self._on_timeout)
        self.active_seg = None

    def _on_timeout(self):
        if self.active_seg is not None:
            self.clicked.emit(self.labels[self.active_seg])

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPoint(self.width() // 2, self.height() // 2)
        outer_radius = self.width() // 2 - 10
        inner_radius = 60 # Larger inner hole
        gap_angle = 12 # More spacing between segments
        seg_angle = 90 - gap_angle
        for i in range(4):
            angles = [90, 0, 270, 180]
            start_angle = angles[i] - (seg_angle / 2)
            rect = QRectF(center.x() - outer_radius, center.y() - outer_radius, outer_radius * 2, outer_radius * 2)
            is_hovered = (self.hovered_seg == i)
            base_color = QColor("#2A2C3A") # Match card/sidebar lighter grey
            color = QColor("#FA8F01") if is_hovered else base_color
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(color))
            from PyQt5.QtGui import QPainterPath
            path = QPainterPath()
            path.arcMoveTo(rect, start_angle)
            path.arcTo(rect, start_angle, seg_angle)
            inner_rect = QRectF(center.x() - inner_radius, center.y() - inner_radius, inner_radius * 2, inner_radius * 2)
            path.arcTo(inner_rect, start_angle + seg_angle, -seg_angle)
            path.closeSubpath()
            painter.drawPath(path)
            painter.setPen(QPen(Qt.white))
            font = QFont("Microsoft YaHei", 10, QFont.Bold)
            painter.setFont(font)
            mid_rad = math.radians(angles[i])
            dist = (outer_radius + inner_radius) / 2
            tx = center.x() + dist * math.cos(mid_rad); ty = center.y() - dist * math.sin(mid_rad)
            painter.drawText(QRectF(tx-30, ty-15, 60, 30), Qt.AlignCenter, self.labels[i])
        center_color = QColor("#2d2e35")
        painter.setBrush(QBrush(center_color))
        painter.setPen(QPen(Qt.white, 2))
        center_rect = QRectF(center.x() - 36, center.y() - 36, 72, 72)
        painter.drawEllipse(center_rect)
        painter.setPen(QPen(Qt.white))
        painter.setFont(QFont("Microsoft YaHei", 11, QFont.Bold))
        painter.drawText(center_rect, Qt.AlignCenter, STRINGS.get(self.lang, STRINGS['zh']).get("btn_reset_view", "复位"))

    def mouseMoveEvent(self, event):
        old = self.hovered_seg
        self.hovered_seg = self.get_segment_at(event.pos())
        if old != self.hovered_seg: self.update()

    def mousePressEvent(self, event):
        seg = self.get_segment_at(event.pos())
        if seg is not None:
            self.active_seg = seg
            self.clicked.emit(self.labels[seg])
            self.timer.start(150)
        else:
            dist = math.sqrt((event.pos().x() - self.width()//2)**2 + (event.pos().y() - self.height()//2)**2)
            if dist < 40:
                self.clicked.emit("RESET")

    def mouseReleaseEvent(self, event):
        self.timer.stop()
        self.active_seg = None

    def get_segment_at(self, pos):
        center = QPoint(self.width() // 2, self.height() // 2)
        diff = pos - center
        dist = math.sqrt(diff.x()**2 + diff.y()**2)
        if dist < 40 or dist > self.width() // 2: return None
        angle = math.degrees(math.atan2(-diff.y(), diff.x()))
        if angle < 0: angle += 360
        if 45 <= angle < 135: return 0
        if (angle >= 315) or (angle < 45): return 1
        if 225 <= angle < 315: return 2
        if 135 <= angle < 225: return 3
        return None

class CoordTab(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.inputs = {}
        
        self.ui_loaded = False
        self.setup_ui()
        
        self.comm_manager.coord_updated.connect(self.update_coord_display)

    def setup_ui(self):
        ui_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'ui_files', 'coord_tab.ui')
        if os.path.exists(ui_path):
            try:
                uic.loadUi(ui_path, self)
                self.ui_loaded = True
                self.setup_ui_from_file()
            except Exception as e:
                print(f"Error loading {ui_path}: {e}")
                self.setup_ui_manual()
        else:
            self.setup_ui_manual()

    def setup_ui_from_file(self):
        # Bind inputs mapping
        # Expected names: spin_x, spin_y, spin_z, spin_pitch, spin_roll, spin_time
        # sb_step, btn_z_plus, btn_z_minus, btn_send, btn_get, lbl_cur_coord
        # Custom widgets: wheel (DirectionWheel), axis_viz (CoordinateSystemViz)
        
        mapping = {
            "X": "spin_x", "Y": "spin_y", "Z": "spin_z", 
            "Pitch": "spin_pitch", "Roll": "spin_roll", "Time": "spin_time"
        }
        
        for k, name in mapping.items():
            if hasattr(self, name):
                self.inputs[k] = getattr(self, name)
                
        # Z Buttons
        if hasattr(self, 'btn_z_plus'):
            self.btn_z_plus.setAutoRepeat(True)
            self.btn_z_plus.setAutoRepeatDelay(400)
            self.btn_z_plus.setAutoRepeatInterval(200)
            self.btn_z_plus.clicked.connect(lambda: self.adjust_coord("Z", 1))
        if hasattr(self, 'btn_z_minus'):
            self.btn_z_minus.setAutoRepeat(True)
            self.btn_z_minus.setAutoRepeatDelay(400)
            self.btn_z_minus.setAutoRepeatInterval(200)
            self.btn_z_minus.clicked.connect(lambda: self.adjust_coord("Z", -1))
            
        # Pitch Buttons
        if hasattr(self, 'btn_pitch_plus'):
            self.btn_pitch_plus.setAutoRepeat(True)
            self.btn_pitch_plus.setAutoRepeatDelay(400)
            self.btn_pitch_plus.setAutoRepeatInterval(200)
            self.btn_pitch_plus.clicked.connect(lambda: self.adjust_coord("Pitch", 1))
        if hasattr(self, 'btn_pitch_minus'):
            self.btn_pitch_minus.setAutoRepeat(True)
            self.btn_pitch_minus.setAutoRepeatDelay(400)
            self.btn_pitch_minus.setAutoRepeatInterval(200)
            self.btn_pitch_minus.clicked.connect(lambda: self.adjust_coord("Pitch", -1))
            
        # Send/Get
        if hasattr(self, 'btn_send'): self.btn_send.clicked.connect(self.send_coordinate_move)
        if hasattr(self, 'btn_get'): self.btn_get.clicked.connect(lambda: self.comm_manager.send_sys(CMD_GET_CUR_COORDS))
        
        # Wheel
        if hasattr(self, 'wheel'):
            # If it's a promoted widget, it might already be connected if we promoted it correctly
            # Check if it has 'clicked' signal
            if hasattr(self.wheel, 'clicked'):
                try: self.wheel.clicked.disconnect() 
                except: pass
                self.wheel.clicked.connect(self.handle_wheel_click)

    def setup_ui_manual(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(6)

        # ── Top title bar: IK title (left) + 3D button (right) ──
        self.arm_3d = None
        self._3d_window = None

        title_layout = QHBoxLayout()
        title_layout.setContentsMargins(0, 5, 0, 5)
        ik_title = QLabel(STRINGS[self.lang]["grp_ik"])
        ik_title.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 11pt;")
        self.lbl_ik_title = ik_title
        title_layout.addWidget(ik_title)
        title_layout.addStretch()

        if HAS_3D:
            self.btn_3d_view = QPushButton(STRINGS[self.lang].get("btn_open_3d", "3D 视图"))
            self.btn_3d_view.setStyleSheet(
                "color: #FA8F01; border: 1px solid #FA8F01; padding: 8px 24px; "
                "font-size: 11pt; font-weight: bold; border-radius: 4px; "
                "background: transparent; min-height: 24px;")
            self.btn_3d_view.setCursor(Qt.PointingHandCursor)
            self.btn_3d_view.clicked.connect(self._open_3d_window)
            title_layout.addWidget(self.btn_3d_view)

        # ── Main content: left visual + right params ──
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.setStyleSheet("QSplitter::handle { background-color: transparent; width: 6px; }")

        # --- Left Side: Visual Control Area ---
        visual_container = QGroupBox()
        self.grp_ik = visual_container

        container_layout = QVBoxLayout(visual_container)
        container_layout.setContentsMargins(20, 10, 20, 5)
        container_layout.setSpacing(5)

        container_layout.addLayout(title_layout)

        grid = QGridLayout()
        grid.setHorizontalSpacing(30)
        grid.setVerticalSpacing(15)
        container_layout.addLayout(grid)

        # 1. Top: Robot Image + Axis Viz
        self.robot_img = QLabel()
        self.robot_img.setStyleSheet("background-color: transparent;")
        import sys
        res_path = os.path.join(sys._MEIPASS, "0.842_old.png") if hasattr(sys, '_MEIPASS') else "0.842_old.png"
        self._robot_pix = QPixmap(res_path)
        if not self._robot_pix.isNull():
            self.robot_img.setPixmap(self._robot_pix.scaled(400, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.robot_img.setAlignment(Qt.AlignCenter)
        self.robot_img.setMaximumSize(500, 500)
        self.robot_img.setScaledContents(False)
        grid.addWidget(self.robot_img, 0, 0, Qt.AlignBottom | Qt.AlignHCenter)

        self.axis_viz = QLabel()
        self.axis_viz.setStyleSheet("background-color: transparent;")
        import sys as _sys
        axes_path = os.path.join(_sys._MEIPASS, "coord_axes.png") if hasattr(_sys, '_MEIPASS') else os.path.join(os.path.dirname(os.path.dirname(__file__)), "coord_axes.png")
        self._axes_pix = QPixmap(axes_path)
        if not self._axes_pix.isNull():
            self.axis_viz.setPixmap(self._axes_pix.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.axis_viz.setMaximumSize(400, 400)
        self.axis_viz.setScaledContents(False)
        grid.addWidget(self.axis_viz, 0, 1, Qt.AlignCenter)

        # 3. Bottom Left: Direction Wheel
        self.wheel = DirectionWheel(lang=self.lang)
        self.wheel.clicked.connect(self.handle_wheel_click)
        grid.addWidget(self.wheel, 1, 0, Qt.AlignCenter)
        
        # 4. Bottom Right: Z/Pitch Buttons
        zp_ctrl_container = QWidget()
        zp_ctrl_layout = QHBoxLayout(zp_ctrl_container)
        zp_ctrl_layout.setSpacing(15)

        z_ctrl_layout = QVBoxLayout()
        z_ctrl_layout.setSpacing(15)
        
        self.btn_z_plus = QPushButton("Z+")
        self.btn_z_plus.setProperty("type", "dir")
        self.btn_z_plus.setAutoRepeat(True)
        self.btn_z_plus.setAutoRepeatDelay(400)
        self.btn_z_plus.setAutoRepeatInterval(200)
        self.btn_z_plus.clicked.connect(lambda: self.adjust_coord("Z", 1))
        
        self.btn_z_minus = QPushButton("Z-")
        self.btn_z_minus.setProperty("type", "dir")
        self.btn_z_minus.setAutoRepeat(True)
        self.btn_z_minus.setAutoRepeatDelay(400)
        self.btn_z_minus.setAutoRepeatInterval(200)
        self.btn_z_minus.clicked.connect(lambda: self.adjust_coord("Z", -1))
        
        z_ctrl_layout.addStretch()
        z_ctrl_layout.addWidget(self.btn_z_plus)
        z_ctrl_layout.addWidget(self.btn_z_minus)
        z_ctrl_layout.addStretch()

        pitch_ctrl_layout = QVBoxLayout()
        pitch_ctrl_layout.setSpacing(15)
        
        self.btn_pitch_plus = QPushButton("P+")
        self.btn_pitch_plus.setProperty("type", "dir")
        self.btn_pitch_plus.setAutoRepeat(True)
        self.btn_pitch_plus.setAutoRepeatDelay(400)
        self.btn_pitch_plus.setAutoRepeatInterval(200)
        self.btn_pitch_plus.clicked.connect(lambda: self.adjust_coord("Pitch", 1))
        
        self.btn_pitch_minus = QPushButton("P-")
        self.btn_pitch_minus.setProperty("type", "dir")
        self.btn_pitch_minus.setAutoRepeat(True)
        self.btn_pitch_minus.setAutoRepeatDelay(400)
        self.btn_pitch_minus.setAutoRepeatInterval(200)
        self.btn_pitch_minus.clicked.connect(lambda: self.adjust_coord("Pitch", -1))

        pitch_ctrl_layout.addStretch()
        pitch_ctrl_layout.addWidget(self.btn_pitch_plus)
        pitch_ctrl_layout.addWidget(self.btn_pitch_minus)
        pitch_ctrl_layout.addStretch()

        zp_ctrl_layout.addLayout(z_ctrl_layout)
        zp_ctrl_layout.addLayout(pitch_ctrl_layout)
        
        grid.addWidget(zp_ctrl_container, 1, 1, Qt.AlignCenter)
        
        # 5. Bottom: Step Box — 放在grid外面，确保始终可见
        step_box = QHBoxLayout()
        self.lbl_step = QLabel(STRINGS[self.lang].get("lbl_step_mm", "步进 (mm):"))
        self.lbl_step.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 13pt; margin-left: 20px;")
        step_box.addWidget(self.lbl_step)
        self.sb_step = QSpinBox()
        self.sb_step.setRange(1, 100); self.sb_step.setValue(10)
        self.sb_step.setFixedSize(150, 50)
        self.sb_step.setStyleSheet("font-size: 13pt; font-weight: bold;")
        step_box.addWidget(self.sb_step)
        
        self.lbl_speed_coord = QLabel(STRINGS[self.lang].get("lbl_speed_mms", "速度 (mm/s):"))
        self.lbl_speed_coord.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 13pt; margin-left: 20px;")
        step_box.addWidget(self.lbl_speed_coord)
        self.sb_speed = QSpinBox()
        self.sb_speed.setRange(10, 2000); self.sb_speed.setValue(20)
        self.sb_speed.setFixedSize(150, 50)
        self.sb_speed.setStyleSheet("font-size: 13pt; font-weight: bold;")
        step_box.addWidget(self.sb_speed)
        
        step_box.addStretch()
        container_layout.addLayout(step_box)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 2)  # images row
        grid.setRowStretch(1, 3)  # controls row

        main_splitter.addWidget(visual_container)
        
        # --- Right Side: Params and Status (scrollable) ---
        from PyQt5.QtWidgets import QScrollArea
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        right_widget = QWidget()
        right_panel = QVBoxLayout(right_widget)
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(4)

        grp_params = QGroupBox(STRINGS[self.lang]["grp_coordinates"])
        self.grp_params = grp_params
        params_layout = QGridLayout(grp_params)
        params_layout.setSpacing(3)
        params_layout.setContentsMargins(8, 20, 8, 8)
        items = ["X", "Y", "Z", "Pitch", "Roll", "Claw", "Time"]
        vals = [200, 0, 200, 0, 0, 0, 1000]
        ranges = {
            "X": (0, 550),
            "Y": (-550, 550),
            "Z": (100, 570),
            "Pitch": (-1000, 1000),
            "Roll": (-90, 90),
            "Claw": (-60, 30),
            "Time": (0, 1000000),
        }
        for idx, (k, v) in enumerate(zip(items, vals)):
            lbl = QLabel(k)
            lbl.setProperty("class", "aux-text")
            params_layout.addWidget(lbl, idx, 0)
            rng = ranges[k]
            if k in ["Pitch", "Roll", "Claw"]:
                sp = QDoubleSpinBox(); sp.setRange(rng[0], rng[1])
            elif k == "Time":
                sp = QSpinBox(); sp.setRange(rng[0], rng[1])
            else:
                sp = QSpinBox(); sp.setRange(rng[0], rng[1])
            sp.setValue(v); self.inputs[k] = sp
            sp.setKeyboardTracking(False)
            # 失焦时恢复原值，只有按回车或点发送才确认
            sp.installEventFilter(self)
            params_layout.addWidget(sp, idx, 1)
        self.btn_send = QPushButton(STRINGS[self.lang]["btn_send_coord"])
        self.btn_send.setObjectName("btn_send")
        self.btn_send.clicked.connect(self.send_coordinate_move)
        params_layout.addWidget(self.btn_send, len(items), 0, 1, 2)
        right_panel.addWidget(grp_params)
        
        self.grp_stat = QGroupBox(STRINGS[self.lang]["grp_realtime_stat"])
        self.grp_stat.setStyleSheet("QGroupBox { margin-top: 10px; padding-top: 12px; }")
        stat_layout = QVBoxLayout(self.grp_stat)
        stat_layout.setSpacing(2)
        stat_layout.setContentsMargins(8, 16, 8, 4)
        
        # Row 1: Buttons
        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        btn_row.setContentsMargins(0, 0, 0, 0)
        self.btn_get = QPushButton(STRINGS[self.lang]["btn_get_coord"])
        self.btn_get.setObjectName("btn_get")
        self.btn_get.clicked.connect(lambda: self.comm_manager.send_sys(CMD_GET_CUR_COORDS))
        btn_row.addWidget(self.btn_get)
        
        self.btn_home = QPushButton(STRINGS[self.lang].get("btn_home_all", "全部回中"))
        self.btn_home.clicked.connect(lambda: self.comm_manager.send_sys(CMD_ARM_RESET, [1000 & 0xFF, (1000 >> 8) & 0xFF]))
        btn_row.addWidget(self.btn_home)
        
        self.btn_read_servos = QPushButton(STRINGS[self.lang].get("btn_read_servos", "读取舵机"))
        self.btn_read_servos.clicked.connect(lambda: self.comm_manager.send_sys(CMD_READ_ALL_SERVOS))
        btn_row.addWidget(self.btn_read_servos)
        
        stat_layout.addLayout(btn_row)

        # Row 2: Coord Info — horizontal 2-column grid
        coord_grid = QGridLayout()
        coord_grid.setSpacing(4)
        self.coord_value_labels = {}
        coord_items = ["X", "Y", "Z", "Pitch", "Roll", "Claw"]
        defaults = [200, 0, 200, 0.0, 0, 0.0]
        for i, (name, val) in enumerate(zip(coord_items, defaults)):
            row = i // 2
            col = i % 2
            lbl = QLabel(f"{name}: {val}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-weight: bold; font-size: 12pt; padding: 2px;")
            coord_grid.addWidget(lbl, row, col)
            self.coord_value_labels[name] = lbl
        stat_layout.addLayout(coord_grid)

        # Row 3: Servo — horizontal 3-column grid
        self.lbl_servo_title = QLabel(STRINGS[self.lang].get("lbl_realtime_servo", "实时舵机:"))
        self.lbl_servo_title.setStyleSheet("font-weight: bold; font-size: 12pt; color: #FA8F01; margin-top: 6px;")
        self.lbl_servo_title.setAlignment(Qt.AlignCenter)
        stat_layout.addWidget(self.lbl_servo_title)

        servo_grid = QGridLayout()
        servo_grid.setSpacing(3)
        self.servo_labels = []
        for i in range(6):
            lbl = QLabel(f"ID{i+1}: --")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: rgba(255, 255, 255, 0.05); border-radius: 5px; padding: 4px;")
            servo_grid.addWidget(lbl, i // 3, i % 3)
            self.servo_labels.append(lbl)
            
        stat_layout.addLayout(servo_grid)
        
        right_panel.addWidget(self.grp_stat)
        right_panel.addStretch()
        
        scroll.setWidget(right_widget)
        main_splitter.addWidget(scroll)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 1)
        outer.addWidget(main_splitter, 1)

    def handle_wheel_click(self, cmd):
        if cmd == "RESET":
            # Reset all coordinates to defaults
            defaults = {"X": 200, "Y": 0, "Z": 200, "Pitch": 0, "Roll": 0}
            for k, v in defaults.items():
                if k in self.inputs:
                    self.inputs[k].setValue(v)
            self.send_coordinate_move()
        elif "X" in cmd: self.adjust_coord("X", 1 if "+" in cmd else -1)
        elif "Y" in cmd: self.adjust_coord("Y", 1 if "+" in cmd else -1)

    def adjust_coord(self, axis, direction):
        step = 10
        if hasattr(self, 'sb_step'):
            step = self.sb_step.value()
        speed = 200
        if hasattr(self, 'sb_speed'):
            speed = self.sb_speed.value()

        dx = dy = dz = dp = dr = dc = 0
        if axis == "X": dx = direction * step
        elif axis == "Y": dy = direction * step
        elif axis == "Z": dz = direction * step
        elif axis == "Pitch": dp = direction * step
        elif axis == "Roll": dr = direction * step
        elif axis == "Claw": dc = direction * step
        
        dur_ms = 200  # 固定 200ms，匹配连发间隔
        
        dp_val = int(dp * 10)
        args = list(struct.pack('<hhhhhhH', int(dx), int(dy), int(dz), dp_val, int(dr), int(dc), dur_ms))
        self.comm_manager.send_sys(CMD_ARM_MOVE_INC, args)

    def _open_3d_window(self):
        """Open 3D arm visualization in a separate fullscreen window."""
        if self._3d_window is not None:
            # Window exists but hidden — just show it
            self._3d_window.showMaximized()
            self._3d_window.activateWindow()
            self._3d_window.raise_()
            return

        try:
            from nexarm_qt.ui.arm_3d_window import Arm3DWindow
            self._3d_window = Arm3DWindow(self.comm_manager, self.lang)
            self._3d_window.showMaximized()
        except Exception as e:
            print(f"Failed to open 3D window: {e}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            print(f"Failed to open 3D window: {e}")
            import traceback
            traceback.print_exc()

    def eventFilter(self, obj, event):
        """焦点进入时记录值，焦点离开时保留用户修改"""
        if event.type() == QEvent.FocusIn:
            if isinstance(obj, (QSpinBox, QDoubleSpinBox)):
                obj._saved_value = obj.value()
        return super().eventFilter(obj, event)

    def send_coordinate_move(self):
        try:
            # 确认所有spinbox当前值
            for sp in self.inputs.values():
                sp._saved_value = sp.value()
            p = int(self.inputs["Pitch"].value() * 10) if "Pitch" in self.inputs else 0
            x = int(self.inputs["X"].value()) if "X" in self.inputs else 200
            y = int(self.inputs["Y"].value()) if "Y" in self.inputs else 0
            z = int(self.inputs["Z"].value()) if "Z" in self.inputs else 200
            r = int(self.inputs["Roll"].value()) if "Roll" in self.inputs else 0
            c = int(self.inputs["Claw"].value()) if "Claw" in self.inputs else 0
            t = int(self.inputs["Time"].value()) if "Time" in self.inputs else 1000
            
            args = list(struct.pack('<hhhhhhH', p, x, y, z, r, c, t))
            self.comm_manager.send_sys(CMD_COORDINATE_SET, args)
        except Exception as e: print(f"Error: {e}")

    def update_coord_display(self, x, y, z, p=0.0, r=0, c=0.0, servos=[]):
        if hasattr(self, 'coord_value_labels'):
            vals = {"X": x, "Y": y, "Z": z, "Pitch": f"{p:.1f}", "Roll": r, "Claw": f"{c:.1f}"}
            for name, val in vals.items():
                if name in self.coord_value_labels:
                    self.coord_value_labels[name].setText(f"{name}: {val}")

        if hasattr(self, 'servo_labels'):
            for i, lbl in enumerate(self.servo_labels):
                if i < len(servos):
                    lbl.setText(f"ID{i+1}: {servos[i]}")
                    lbl.setStyleSheet("background-color: rgba(250, 143, 1, 0.2); border-radius: 4px; padding: 3px; color: #FA8F01;")
                else:
                    lbl.setText(f"ID{i+1}: --")
                    lbl.setStyleSheet("background-color: rgba(255, 255, 255, 0.05); border-radius: 4px; padding: 3px; color: gray;")

    def update_language(self, lang):
        self.lang = lang
        if hasattr(self, 'lbl_ik_title'): self.lbl_ik_title.setText(STRINGS[lang]["grp_ik"])
        if hasattr(self, 'grp_params'): self.grp_params.setTitle(STRINGS[lang]["grp_coordinates"])
        if hasattr(self, 'btn_send'): self.btn_send.setText(STRINGS[lang]["btn_send_coord"])
        if hasattr(self, 'grp_stat'): self.grp_stat.setTitle(STRINGS[lang]["grp_realtime_stat"])
        if hasattr(self, 'btn_get'): self.btn_get.setText(STRINGS[lang]["btn_get_coord"])
        if hasattr(self, 'btn_home'): self.btn_home.setText(STRINGS[lang].get("btn_home_all", "全部回中"))
        if hasattr(self, 'btn_read_servos'): self.btn_read_servos.setText(STRINGS[lang].get("btn_read_servos", "读取舵机"))
        if hasattr(self, 'arm_3d') and self.arm_3d is not None:
            self.arm_3d.update_language(lang)
        if hasattr(self, 'btn_3d_view'):
            self.btn_3d_view.setText(STRINGS[lang].get("btn_open_3d", "3D 视图"))
        if hasattr(self, '_3d_window') and self._3d_window is not None:
            self._3d_window.update_language(lang)
        if hasattr(self, 'lbl_step'):
            self.lbl_step.setText(STRINGS[lang].get("lbl_step_mm", "Step (mm):"))
        if hasattr(self, 'lbl_speed_coord'):
            self.lbl_speed_coord.setText(STRINGS[lang].get("lbl_speed_mms", "Speed (mm/s):"))
        if hasattr(self, 'lbl_servo_title'):
            self.lbl_servo_title.setText(STRINGS[lang].get("lbl_realtime_servo", "Realtime Servo:"))
        if hasattr(self, 'wheel'):
            self.wheel.lang = lang
            self.wheel.update()
