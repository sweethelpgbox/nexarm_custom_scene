"""3D Arm Visualization — full STL, background loading with spinner."""

import os
import math
import struct
import struct as _struct
import threading
import numpy as np
import xml.etree.ElementTree as ET

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel,
    QPushButton, QGroupBox, QGridLayout, QStackedWidget,
    QDoubleSpinBox, QSpinBox, QFrame
)
from PyQt5.QtCore import Qt, QTimer, QEvent, pyqtSignal
from PyQt5.QtGui import QMatrix4x4, QPainter, QColor, QPen, QPixmap

import pyqtgraph.opengl as gl

from nexarm_qt.ui.urdf_parser import parse_urdf, joint_transform, make_transform, axis_angle_matrix
from nexarm_qt.constants import CMD_SET_SINGLE_MOTOR, CMD_COORDINATE_SET
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S

SCALE = 1000.0
SKIP_LINKS = {"camera_link"}

LINK_COLORS = [
    (0.55, 0.55, 0.60, 1.0),  # base
    (0.30, 0.60, 0.95, 1.0),  # J1
    (0.30, 0.80, 0.45, 1.0),  # J2
    (0.95, 0.60, 0.25, 1.0),  # J3
    (0.75, 0.35, 0.80, 1.0),  # J4
    (0.80, 0.60, 0.10, 1.0),  # link5 - 橙黄色
    (0.55, 0.10, 0.55, 1.0),  # link6 - 紫色关节
    (0.02, 0.02, 0.02, 1.0),  # gripper - 全黑
    (0.02, 0.02, 0.02, 1.0),
    (0.02, 0.02, 0.02, 1.0),
]


def load_stl_full(path):
    """Load ALL faces from binary STL using numpy bulk read."""
    with open(path, 'rb') as f:
        f.read(80)
        n = _struct.unpack('<I', f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8)
    if len(raw) < n * 50:
        n = len(raw) // 50
    raw = raw[:n * 50].reshape(n, 50)
    vb = raw[:, 12:48].copy()
    verts = (vb.view(np.float32).reshape(n, 3, 3) * SCALE).reshape(-1, 3)
    faces = np.arange(n * 3, dtype=np.int32).reshape(n, 3)
    return verts, faces


def np44_to_qmatrix(T):
    return QMatrix4x4(*T.flatten().tolist())


# ── Loading spinner widget ──────────────────────────────────

class SpinnerWidget(QWidget):
    """Rotating spinner shown while loading."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._angle = 0
        self._progress = ""
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._rotate)
        self.setMinimumSize(200, 200)

    def start(self):
        self._timer.start(30)

    def stop(self):
        self._timer.stop()

    def set_progress(self, text):
        self._progress = text

    def _rotate(self):
        self._angle = (self._angle + 8) % 360
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(30, 30, 40))

        cx, cy = self.width() // 2, self.height() // 2
        r = min(cx, cy) - 40
        if r < 20:
            r = 20

        # Draw spinning arc
        pen = QPen(QColor(250, 143, 1), 4)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.translate(cx, cy)
        p.rotate(self._angle)
        p.drawArc(-r, -r, r*2, r*2, 0, 270 * 16)

        # Text
        p.resetTransform()
        p.setPen(QColor(180, 180, 180))
        font = p.font()
        font.setPixelSize(14)
        p.setFont(font)
        text = self._progress if self._progress else "加载 3D 模型中..."
        p.drawText(self.rect(), Qt.AlignCenter, text)
        p.end()


# ── Main widget ─────────────────────────────────────────────

class Arm3DWidget(QWidget):
    """3D arm visualization with full STL, background loading."""

    _meshes_ready = pyqtSignal(list)  # signal: mesh data loaded
    _load_progress = pyqtSignal(str)  # signal: loading progress text

    JOINT_LABELS = ["J1 底座", "J2 肩部", "J3 肘部", "J4 腕部", "J5 旋转", "J6 爪子"]

    def __init__(self, comm_manager, lang='zh', parent=None):
        super().__init__(parent)
        self.comm_manager = comm_manager
        self.lang = lang
        # Initial servo values from real arm standing pose
        # ID1:2048 ID2:2088 ID3:1198 ID4:1238 ID5:2048 ID6:2048
        self._servo_home = [2048, 2088, 1198, 1238, 2048, 2048]
        self.joint_angles = [0.0] * 6
        self._servo_to_angles(self._servo_home)
        self._send_enabled = True
        self.mesh_items = {}

        # Parse URDF — support PyInstaller bundled path
        import sys
        if hasattr(sys, '_MEIPASS'):
            urdf_path = os.path.join(sys._MEIPASS, "ui", "nexarm.urdf")
        else:
            urdf_path = os.path.join(os.path.dirname(__file__), "nexarm.urdf")
        self.links_info, self.joints = parse_urdf(urdf_path)
        self.fixed_joints = self._parse_fixed_joints(urdf_path)

        self.link_order = ["base_link"] + [j["child"] for j in self.joints]
        for fj in self.fixed_joints:
            if fj["child"] not in self.link_order and fj["child"] not in SKIP_LINKS:
                self.link_order.append(fj["child"])

        self._build_ui()
        self._meshes_ready.connect(self._on_meshes_ready)
        self._load_progress.connect(self.spinner.set_progress)
        self._start_bg_load()

        self.comm_manager.coord_updated.connect(self._on_coord_updated)

    def _parse_fixed_joints(self, urdf_path):
        """Parse ALL non-revolute-chain joints (fixed + gripper revolute/prismatic)."""
        fjoints = []
        tree = ET.parse(urdf_path)
        # Get the revolute chain child names
        chain_children = {j["child"] for j in self.joints}
        chain_children.add("base_link")

        for je in tree.getroot().findall("joint"):
            child = je.find("child").get("link")
            # Skip joints already in the main revolute chain
            if child in chain_children:
                continue
            # Skip camera
            if child in SKIP_LINKS:
                continue
            jtype = je.get("type", "fixed")
            origin = je.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0,0,0]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0,0,0]
            axis_el = je.find("axis")
            axis = [float(v) for v in axis_el.get("xyz", "0 0 1").split()] if axis_el is not None else [0,0,1]
            fjoints.append({
                "name": je.get("name", ""),
                "type": jtype,
                "parent": je.find("parent").get("link"),
                "child": child,
                "origin_xyz": xyz, "origin_rpy": rpy,
                "axis": axis,
            })
        return fjoints

    # Servo parameters from firmware (Robot_Arm.c Knot_Pos2Deg):

    SERVO_CENTER = [2048, 2048, 1198, 1238, 2048, 2048]
    SERVO_RATIO  = [0.05859375, 0.087890625, 0.087890625, 0.087890625, 0.05859375, 0.05859375]
    SERVO_DIR    = [1, 1, -1, -1, -1, -1]  # J4 reversed in firmware

    # URDF axis vs firmware: J1 IK uses -atan2(y,x) so negate for URDF
    URDF_DIR = [1, -1, 1, -1, 1, 1]

    def _servo_to_angles(self, servo_vals):
        """Convert servo values to joint angles (radians) using firmware formula."""
        for i in range(min(6, len(servo_vals))):
            deg = (servo_vals[i] - self.SERVO_CENTER[i]) * self.SERVO_RATIO[i] * self.SERVO_DIR[i]
            deg *= self.URDF_DIR[i]
            self.joint_angles[i] = math.radians(deg)

    # ── UI ──────────────────────────────────────────────────────

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Stacked: spinner (0) / 3D view (1)
        self.stack = QStackedWidget()

        # Page 0: spinner
        self.spinner = SpinnerWidget()
        self.stack.addWidget(self.spinner)

        # Page 1: GL view
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setBackgroundColor(200, 200, 210)
        self.gl_widget.setCameraPosition(distance=500, elevation=25, azimuth=-50)

        g = gl.GLGridItem()
        g.setSize(600, 600)
        g.setSpacing(50, 50)
        g.setColor((120, 120, 130, 160))  # 深灰网格线
        self.gl_widget.addItem(g)

        ax = gl.GLAxisItem()
        ax.setSize(80, 80, 80)
        self.gl_widget.addItem(ax)

        self.stack.addWidget(self.gl_widget)
        self.stack.setCurrentIndex(0)  # show spinner first
        self.spinner.start()

        # ── Main split: 3D view (left) + control panel (right) ──
        hsplit = QHBoxLayout()
        hsplit.addWidget(self.stack, stretch=5)

        # ── Right control panel ──────────────────────────────────
        ctrl_panel = QWidget()
        ctrl_panel.setMinimumWidth(340)
        ctrl_panel.setMaximumWidth(420)
        ctrl_panel.setStyleSheet("""
            background: rgba(30,30,40,0.95); border-radius: 6px;
        """)
        cp = QVBoxLayout(ctrl_panel)
        cp.setContentsMargins(16, 12, 16, 12)
        cp.setSpacing(10)

        # Title
        import sys
        title_row = QHBoxLayout()
        arm_icon = QLabel()
        arm_icon.setStyleSheet("background: transparent; border: none;")
        icon_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "0.842_old.png")
        if hasattr(sys, '_MEIPASS'):
            icon_path = os.path.join(sys._MEIPASS, "0.842.png")
        pix = QPixmap(icon_path)
        if not pix.isNull():
            arm_icon.setPixmap(pix.scaled(22, 22, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        arm_icon.setFixedSize(S(24), S(24))
        title_row.addWidget(arm_icon)
        coord_title = QLabel("坐标控制 (IK)")
        coord_title.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 10pt; background: transparent; border: none;")
        title_row.addWidget(coord_title)
        title_row.addStretch()
        cp.addLayout(title_row)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #444; background: transparent;")
        cp.addWidget(sep)

        # Coordinate inputs — vertical list, bigger
        coord_style = """
            QSpinBox, QDoubleSpinBox {
                font-size: 10pt; padding: 6px 8px;
                background: #343645; color: white;
                border: 1px solid #4A4D5E; border-radius: 4px;
                selection-background-color: #FA8F01;
                min-height: 28pt;
            }
            QSpinBox:focus, QDoubleSpinBox:focus {
                border: 2px solid #FA8F01;
                background: #3A3C4D;
            }
            QLabel { background: transparent; border: none; }
        """
        coord_w = QWidget()
        coord_w.setStyleSheet(coord_style)
        cg = QGridLayout(coord_w)
        cg.setSpacing(6)
        cg.setContentsMargins(0, 0, 0, 0)

        self.coord_inputs = {}
        coord_fields = [
            ("X", 0, 550, 220, False),
            ("Y", -550, 550, 0, False),
            ("Z", 100, 570, 200, False),
            ("Pitch", -1000.0, 1000.0, 0.0, True),
            ("Roll", -90, 90, 0, False),
            ("Claw", -60.0, 30.0, 0.0, True),
            ("Time", 0, 1000000, 1000, False),
        ]

        for idx, (name, lo, hi, default, is_float) in enumerate(coord_fields):
            lbl = QLabel(name)
            lbl.setStyleSheet("color: #B0BEC5; font-size: 9pt; font-weight: 500;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setFixedWidth(S(48))
            cg.addWidget(lbl, idx, 0)

            if is_float:
                sb = QDoubleSpinBox()
                sb.setDecimals(1)
                sb.setRange(float(lo), float(hi))
                sb.setValue(float(default))
            else:
                sb = QSpinBox()
                sb.setRange(int(lo), int(hi))
                sb.setValue(int(default))
            sb.setReadOnly(False)
            sb.setKeyboardTracking(False)
            sb.setFocusPolicy(Qt.StrongFocus)
            sb._saved_value = sb.value()
            sb.installEventFilter(self)
            self.coord_inputs[name] = sb
            cg.addWidget(sb, idx, 1)

        cp.addWidget(coord_w)

        # Buttons — full width
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_send_coord = QPushButton("发送坐标")
        self.btn_send_coord.setStyleSheet(
            "background-color: #FF8F00; color: white; padding: 8px 16px; "
            "font-size: 9pt; font-weight: bold; border-radius: 4px; border: none; min-height: 24pt;")
        self.btn_send_coord.clicked.connect(self._send_coordinate)
        btn_row.addWidget(self.btn_send_coord)

        self.btn_home = QPushButton("回中")
        self.btn_home.setStyleSheet(
            "background-color: #455A64; color: white; padding: 8px 16px; "
            "font-size: 9pt; border-radius: 4px; border: none; min-height: 24pt;")
        self.btn_home.clicked.connect(self._home_all)
        btn_row.addWidget(self.btn_home)
        cp.addLayout(btn_row)

        # Separator
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet("color: #444; background: transparent;")
        cp.addWidget(sep2)

        # Servo values — clean table layout
        self.servo_title = QLabel(STRINGS[self.lang].get("lbl_realtime_servo", "实时舵机"))
        self.servo_title.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 9pt; background: transparent; border: none;")
        cp.addWidget(self.servo_title)

        sg = QGridLayout()
        sg.setSpacing(4)
        sg.setContentsMargins(0, 0, 0, 0)
        self.sliders = []
        self.slider_labels = []

        for i in range(6):
            row = i // 3
            col = i % 3
            name = self.JOINT_LABELS[i] if i < len(self.JOINT_LABELS) else f"J{i+1}"

            # Joint name
            lbl = QLabel(f"ID{i+1}")
            lbl.setStyleSheet("color: #78909C; font-size: 8pt; background: transparent; border: none;")
            lbl.setAlignment(Qt.AlignCenter)
            sg.addWidget(lbl, row * 2, col)

            # Value badge
            vl = QLabel(f"{self._servo_home[i]}")
            vl.setAlignment(Qt.AlignCenter)
            vl.setStyleSheet(
                "color: #FA8F01; font-size: 9pt; font-weight: bold; "
                "background: rgba(250,143,1,0.1); padding: 4px 8px; "
                "border-radius: 4px; border: none;")
            self.slider_labels.append(vl)
            sg.addWidget(vl, row * 2 + 1, col)

        sg.setColumnStretch(0, 1)
        sg.setColumnStretch(1, 1)
        sg.setColumnStretch(2, 1)
        cp.addLayout(sg)

        # Current coord display
        self.lbl_cur_coord = QLabel("")
        self.lbl_cur_coord.setStyleSheet("color: #78909C; font-size: 8pt; background: transparent; border: none;")
        self.lbl_cur_coord.setWordWrap(True)
        cp.addWidget(self.lbl_cur_coord)

        cp.addStretch()
        hsplit.addWidget(ctrl_panel, stretch=0)
        layout.addLayout(hsplit, stretch=1)

    # ── Background loading ──────────────────────────────────────

    def _start_bg_load(self):
        """Start loading STL files in background thread."""
        def _load():
            result = []
            ci = 0
            total = sum(1 for n in self.link_order if n not in SKIP_LINKS and self.links_info.get(n, {}).get("stl_path"))
            loaded = 0
            for name in self.link_order:
                if name in SKIP_LINKS:
                    ci += 1
                    continue
                info = self.links_info.get(name)
                if not info or not info["stl_path"]:
                    ci += 1
                    continue
                try:
                    self._load_progress.emit(f"加载中 {loaded+1}/{total}: {name}")
                    verts, faces = load_stl_full(info["stl_path"])
                    color = LINK_COLORS[ci % len(LINK_COLORS)]
                    result.append((name, verts, faces, color))
                    loaded += 1
                except Exception as e:
                    print(f"[3D] load {name}: {e}")
                ci += 1
            self._load_progress.emit(f"加载完成! {loaded} 个模型")
            self._meshes_ready.emit(result)

        t = threading.Thread(target=_load, daemon=True)
        t.start()

    def _on_meshes_ready(self, mesh_data_list):
        """Called on main thread — create GL meshes in batches to avoid freeze."""
        self.mesh_items = {}
        self._pending_meshes = mesh_data_list
        self._mesh_batch_idx = 0
        self._batch_timer = QTimer(self)
        self._batch_timer.timeout.connect(self._load_next_mesh_batch)
        self._batch_timer.start(10)

    def _load_next_mesh_batch(self):
        """Load one mesh per timer tick to keep UI responsive."""
        if self._mesh_batch_idx >= len(self._pending_meshes):
            self._batch_timer.stop()
            self._update_model()
            self.spinner.stop()
            self.stack.setCurrentIndex(1)
            print(f"[3D] Ready: {len(self.mesh_items)} meshes")
            return

        name, verts, faces, color = self._pending_meshes[self._mesh_batch_idx]
        try:
            md = gl.MeshData(vertexes=verts, faces=faces)
            item = gl.GLMeshItem(
                meshdata=md, smooth=True, color=color,
                shader='shaded', glOptions='opaque'
            )
            self.gl_widget.addItem(item)
            self.mesh_items[name] = item
            self.spinner.set_progress(f"渲染中 {self._mesh_batch_idx+1}/{len(self._pending_meshes)}: {name}")
        except Exception as e:
            print(f"[3D] mesh {name}: {e}")

        self._mesh_batch_idx += 1

    # ── Transform ───────────────────────────────────────────────

    def _compute_link_transforms(self):
        xf = {}
        xf["base_link"] = np.eye(4)
        T = np.eye(4)
        for i, jnt in enumerate(self.joints):
            scaled = dict(jnt)
            scaled["origin_xyz"] = [v * SCALE for v in jnt["origin_xyz"]]
            T = T @ joint_transform(scaled, self.joint_angles[i])
            xf[jnt["child"]] = T.copy()

        # Gripper: J6 angle drives gripper_base_joint (revolute) and jaw joints (prismatic)
        claw_angle = self.joint_angles[5]  # J6 in radians
        # Convert claw angle to jaw linear displacement (approximate)
        # Claw range ~-60..+30 deg, jaw travel ~0..0.038m
        # Positive claw angle = close, negative = open
        jaw_travel = -claw_angle * 0.02  # rough scale: radians to meters displacement

        # Non-main-chain joints — with revolute/prismatic support
        remaining = list(self.fixed_joints)
        for _ in range(5):  # max depth
            still_remaining = []
            for fj in remaining:
                parent_T = xf.get(fj["parent"])
                if parent_T is None:
                    still_remaining.append(fj)
                    continue
                T_origin = make_transform(
                    [v * SCALE for v in fj["origin_xyz"]],
                    fj["origin_rpy"]
                )
                jtype = fj.get("type", "fixed")
                if jtype == "revolute" and "gripper_base" in fj.get("name", ""):
                    # gripper_base_joint: driven by J6 claw angle
                    R = np.eye(4)
                    R[:3, :3] = axis_angle_matrix(fj["axis"], claw_angle)
                    xf[fj["child"]] = parent_T @ T_origin @ R
                elif jtype == "prismatic" and "jaw" in fj.get("name", ""):
                    # jaw joints: translate along axis by jaw_travel
                    # right jaw moves +Y, left jaw moves +Y (mirrored by origin)
                    T_slide = np.eye(4)
                    axis = np.array(fj["axis"], dtype=float)
                    disp = jaw_travel * SCALE  # convert to mm
                    if "left" in fj.get("name", ""):
                        disp = -disp  # left jaw moves opposite
                    T_slide[:3, 3] = axis * disp
                    xf[fj["child"]] = parent_T @ T_origin @ T_slide
                else:
                    # Fixed joint — no motion
                    xf[fj["child"]] = parent_T @ T_origin
            remaining = still_remaining
            if not remaining:
                break

        return xf

    def _update_model(self):
        if not self.mesh_items:
            return
        xf = self._compute_link_transforms()
        for name, item in self.mesh_items.items():
            T = xf.get(name, np.eye(4))
            item.setTransform(np44_to_qmatrix(T))

    # ── Interaction ─────────────────────────────────────────────

    def _slider_moved(self, idx, val_deg):
        """No longer used — sliders removed."""
        pass

    def _send_joint(self, idx, angle_deg):
        """No longer used — no joint sliders."""
        pass

    def _send_coordinate(self):
        """Send coordinate command using IK interface — same as coord_tab."""
        try:
            # 确认所有spinbox当前值（和2D一样）
            for sp in self.coord_inputs.values():
                sp._saved_value = sp.value()
            p = int(self.coord_inputs["Pitch"].value() * 10)
            x = int(self.coord_inputs["X"].value())
            y = int(self.coord_inputs["Y"].value())
            z = int(self.coord_inputs["Z"].value())
            r = int(self.coord_inputs["Roll"].value())
            c = int(self.coord_inputs["Claw"].value())
            t = int(self.coord_inputs["Time"].value())
            args = list(struct.pack('<hhhhhhH', p, x, y, z, r, c, t))
            self.comm_manager.send_sys(CMD_COORDINATE_SET, args)
        except Exception as e:
            print(f"[3D] send coord error: {e}")

    def eventFilter(self, obj, event):
        """焦点进入时记录值，焦点离开时保留用户修改"""
        if event.type() == QEvent.FocusIn:
            if isinstance(obj, (QSpinBox, QDoubleSpinBox)):
                obj._saved_value = obj.value()
        return super().eventFilter(obj, event)

    def _on_coord_updated(self, x, y, z, pitch, roll, claw, servo_angles):
        if not servo_angles:
            return

        self.lbl_cur_coord.setText(
            f"X:{x}  Y:{y}  Z:{z}  Pitch:{pitch:.1f}  Roll:{roll}  Claw:{claw:.1f}"
        )

        # 坐标输入框不自动更新，只在用户主动请求时更新
        # （通过 update_coord_inputs 方法手动调用）

        # Update servo angle display + 3D model
        self._servo_to_angles(servo_angles)
        n = min(6, len(servo_angles), len(self.slider_labels))
        for i in range(n):
            self.slider_labels[i].setText(f"ID{i+1}: {servo_angles[i]}")

        self._update_model()

    def update_coord_inputs(self, x, y, z, pitch, roll, claw):
        """手动请求时才更新坐标输入框"""
        for name, val in [("X", x), ("Y", y), ("Z", z), ("Roll", roll)]:
            sb = self.coord_inputs.get(name)
            if sb:
                sb.blockSignals(True)
                sb.setValue(int(val))
                sb._saved_value = int(val)
                sb.blockSignals(False)
        for name, val in [("Pitch", float(pitch)), ("Claw", float(claw))]:
            sb = self.coord_inputs.get(name)
            if sb:
                sb.blockSignals(True)
                sb.setValue(val)
                sb._saved_value = val
                sb.blockSignals(False)

    def _home_all(self):
        """Send home position coordinate."""
        self.coord_inputs["X"].setValue(200)
        self.coord_inputs["Y"].setValue(0)
        self.coord_inputs["Z"].setValue(200)
        self.coord_inputs["Pitch"].setValue(0.0)
        self.coord_inputs["Roll"].setValue(0)
        self.coord_inputs["Claw"].setValue(0.0)
        self.coord_inputs["Time"].setValue(1000)
        self._send_coordinate()

    def update_language(self, lang):
        self.lang = lang
        if hasattr(self, 'servo_title'):
            self.servo_title.setText(STRINGS[lang].get("lbl_realtime_servo", "Realtime Servo"))
