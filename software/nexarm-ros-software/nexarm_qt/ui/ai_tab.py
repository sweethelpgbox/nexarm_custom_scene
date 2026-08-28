from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QLabel, QPushButton, QHBoxLayout
from nexarm_qt.translations import STRINGS
from nexarm_qt.ui.ai_calibration_widget import AICalibrationWidget

class AITab(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Add the first feature: Calibration Widget
        self.calib_widget = AICalibrationWidget(self.comm_manager, self.lang)
        layout.addWidget(self.calib_widget)
        
        # Tracking Group
        self.grp_tracking = QGroupBox(STRINGS[self.lang].get('grp_tracking', 'Tracking'))
        self.init_tracking_ui()
        layout.addWidget(self.grp_tracking)

        # Grabbing Group
        self.grp_grabbing = QGroupBox(STRINGS[self.lang].get('grp_grabbing', 'Grabbing'))
        self.init_grabbing_ui()
        layout.addWidget(self.grp_grabbing)

        layout.addStretch()

    def _make_grid(self):
        """统一的 GridLayout 配置"""
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(8)
        layout.setColumnStretch(0, 3)  # 标签
        layout.setColumnStretch(1, 1)  # 按钮1
        layout.setColumnStretch(2, 1)  # 按钮2
        layout.setColumnStretch(3, 1)  # 按钮3
        layout.setColumnStretch(4, 1)  # 按钮4
        return layout

    def _pressed_style(self):
        """按下变橙色，松开恢复"""
        return "QPushButton:pressed { background-color: #FA8F01; color: #FFFFFF; }"

    def _set_active(self, btn_start, btn_stop):
        """开启按钮常亮橙色，停止按钮恢复"""
        active = "QPushButton { background-color: #FA8F01; color: #FFFFFF; }"
        normal = ""
        btn_start.setStyleSheet(active)
        btn_stop.setStyleSheet(normal)

    def _set_inactive(self, btn_start, btn_stop):
        btn_start.setStyleSheet("")
        btn_stop.setStyleSheet("")

    def _bind_pair(self, btn_start, btn_stop, cmd_start, cmd_stop):
        """绑定开启/停止按钮：开启常亮，停止恢复"""
        def on_start():
            self.comm_manager.send_packet(0xFF, cmd_start[0], cmd_start[1])
            self._set_active(btn_start, btn_stop)
        def on_stop():
            self.comm_manager.send_packet(0xFF, cmd_stop[0], cmd_stop[1])
            self._set_inactive(btn_start, btn_stop)
        btn_start.clicked.connect(on_start)
        btn_stop.clicked.connect(on_stop)

    def init_tracking_ui(self):
        layout = self._make_grid()
        
        # Face Tracking
        self.lbl_face = QLabel(STRINGS[self.lang].get('lbl_face_track', 'Face Tracking'))
        self.btn_face_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_face_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_face_start, self.btn_face_stop, (0x29, [0x01]), (0x29, [0x00]))
        layout.addWidget(self.lbl_face, 0, 0)
        layout.addWidget(self.btn_face_start, 0, 1)
        layout.addWidget(self.btn_face_stop, 0, 2)
        
        # Color Tracking
        self.lbl_color = QLabel(STRINGS[self.lang].get('lbl_color_track', 'Color Tracking'))
        self.btn_color_red = QPushButton(STRINGS[self.lang].get('color_red', 'Red'))
        self.btn_color_green = QPushButton(STRINGS[self.lang].get('color_green', 'Green'))
        self.btn_color_blue = QPushButton(STRINGS[self.lang].get('color_blue', 'Blue'))
        self.btn_color_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._color_btns = [self.btn_color_red, self.btn_color_green, self.btn_color_blue]
        active = "QPushButton { background-color: #FA8F01; color: #FFFFFF; }"
        def color_start(btn, cmd_args):
            def fn():
                self.comm_manager.send_packet(0xFF, 0x28, cmd_args)
                for b in self._color_btns:
                    b.setStyleSheet("")
                btn.setStyleSheet(active)
            return fn
        self.btn_color_red.clicked.connect(color_start(self.btn_color_red, [0x01, 0x01]))
        self.btn_color_green.clicked.connect(color_start(self.btn_color_green, [0x01, 0x02]))
        self.btn_color_blue.clicked.connect(color_start(self.btn_color_blue, [0x01, 0x03]))
        def color_stop():
            self.comm_manager.send_packet(0xFF, 0x28, [0x00, 0x00])
            for b in self._color_btns:
                b.setStyleSheet("")
        self.btn_color_stop.clicked.connect(color_stop)
        layout.addWidget(self.lbl_color, 1, 0)
        layout.addWidget(self.btn_color_red, 1, 1)
        layout.addWidget(self.btn_color_green, 1, 2)
        layout.addWidget(self.btn_color_blue, 1, 3)
        layout.addWidget(self.btn_color_stop, 1, 4)
        
        # AprilTag Tracking
        self.lbl_tag = QLabel(STRINGS[self.lang].get('lbl_tag_track', 'AprilTag Tracking'))
        self.btn_tag_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_tag_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_tag_start, self.btn_tag_stop, (0x2B, [0x01]), (0x2B, [0x00]))
        layout.addWidget(self.lbl_tag, 2, 0)
        layout.addWidget(self.btn_tag_start, 2, 1)
        layout.addWidget(self.btn_tag_stop, 2, 2)
        
        # Gesture Recognition
        self.lbl_gesture = QLabel(STRINGS[self.lang].get('lbl_gesture_track', '手势识别'))
        self.btn_gesture_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_gesture_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_gesture_start, self.btn_gesture_stop, (0x51, [0x01]), (0x51, [0x00]))
        layout.addWidget(self.lbl_gesture, 3, 0)
        layout.addWidget(self.btn_gesture_start, 3, 1)
        layout.addWidget(self.btn_gesture_stop, 3, 2)
        
        self.grp_tracking.setLayout(layout)

    def init_grabbing_ui(self):
        layout = self._make_grid()
        
        # AprilTag Grab
        self.lbl_tag_grab = QLabel(STRINGS[self.lang].get('lbl_tag_grab', 'AprilTag Grab'))
        self.btn_tag_grab_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_tag_grab_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_tag_grab_start, self.btn_tag_grab_stop, (0x2C, [0x01]), (0x2C, [0x00]))
        layout.addWidget(self.lbl_tag_grab, 0, 0)
        layout.addWidget(self.btn_tag_grab_start, 0, 1)
        layout.addWidget(self.btn_tag_grab_stop, 0, 2)
        
        # Waste Grab
        self.lbl_waste = QLabel(STRINGS[self.lang].get('lbl_waste_grab', 'Waste Grab'))
        self.btn_waste_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_waste_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_waste_start, self.btn_waste_stop, (0x30, [0x01]), (0x30, [0x00]))
        layout.addWidget(self.lbl_waste, 1, 0)
        layout.addWidget(self.btn_waste_start, 1, 1)
        layout.addWidget(self.btn_waste_stop, 1, 2)
        
        # Color Grab
        self.lbl_color_grab = QLabel(STRINGS[self.lang].get('lbl_color_grab', 'Color Grab'))
        self.btn_cgrab_red = QPushButton(STRINGS[self.lang].get('color_red', 'Red'))
        self.btn_cgrab_green = QPushButton(STRINGS[self.lang].get('color_green', 'Green'))
        self.btn_cgrab_blue = QPushButton(STRINGS[self.lang].get('color_blue', 'Blue'))
        self.btn_cgrab_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._cgrab_btns = [self.btn_cgrab_red, self.btn_cgrab_green, self.btn_cgrab_blue]
        active = "QPushButton { background-color: #FA8F01; color: #FFFFFF; }"
        def cgrab_start(btn, color_id):
            def fn():
                self.comm_manager.send_packet(0xFF, 0x2E, [0x01, color_id])
                for b in self._cgrab_btns:
                    b.setStyleSheet("")
                btn.setStyleSheet(active)
            return fn
        self.btn_cgrab_red.clicked.connect(cgrab_start(self.btn_cgrab_red, 0x01))
        self.btn_cgrab_green.clicked.connect(cgrab_start(self.btn_cgrab_green, 0x02))
        self.btn_cgrab_blue.clicked.connect(cgrab_start(self.btn_cgrab_blue, 0x03))
        def cgrab_stop():
            self.comm_manager.send_packet(0xFF, 0x2E, [0x00])
            for b in self._cgrab_btns:
                b.setStyleSheet("")
        self.btn_cgrab_stop.clicked.connect(cgrab_stop)
        layout.addWidget(self.lbl_color_grab, 2, 0)
        layout.addWidget(self.btn_cgrab_red, 2, 1)
        layout.addWidget(self.btn_cgrab_green, 2, 2)
        layout.addWidget(self.btn_cgrab_blue, 2, 3)
        layout.addWidget(self.btn_cgrab_stop, 2, 4)

        # LLM
        self.lbl_llm = QLabel(STRINGS[self.lang].get('lbl_llm', '大模型'))
        self.btn_llm_start = QPushButton(STRINGS[self.lang].get('btn_start_play', 'Start'))
        self.btn_llm_stop = QPushButton(STRINGS[self.lang].get('btn_stop_play', 'Stop'))
        self._bind_pair(self.btn_llm_start, self.btn_llm_stop, (0x2F, [0x01]), (0x2F, [0x00]))
        layout.addWidget(self.lbl_llm, 3, 0)
        layout.addWidget(self.btn_llm_start, 3, 1)
        layout.addWidget(self.btn_llm_stop, 3, 2)
        
        self.grp_grabbing.setLayout(layout)

    def update_language(self, lang):
        self.lang = lang
        if hasattr(self, 'calib_widget'):
            self.calib_widget.update_language(lang)
            
        s = STRINGS[lang]
        
        self.grp_tracking.setTitle(s.get('grp_tracking', 'Tracking'))
        self.lbl_face.setText(s.get('lbl_face_track', 'Face Tracking'))
        self.btn_face_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_face_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.lbl_color.setText(s.get('lbl_color_track', 'Color Tracking'))
        self.btn_color_red.setText(s.get('color_red', 'Red'))
        self.btn_color_green.setText(s.get('color_green', 'Green'))
        self.btn_color_blue.setText(s.get('color_blue', 'Blue'))
        self.btn_color_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.lbl_tag.setText(s.get('lbl_tag_track', 'AprilTag Tracking'))
        self.btn_tag_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_tag_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.lbl_gesture.setText(s.get('lbl_gesture_track', '手势识别'))
        self.btn_gesture_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_gesture_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.grp_grabbing.setTitle(s.get('grp_grabbing', 'Grabbing'))
        self.lbl_tag_grab.setText(s.get('lbl_tag_grab', 'AprilTag Grab'))
        self.btn_tag_grab_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_tag_grab_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.lbl_waste.setText(s.get('lbl_waste_grab', 'Waste Grab'))
        self.btn_waste_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_waste_stop.setText(s.get('btn_stop_play', 'Stop'))
        
        self.lbl_llm.setText(s.get('lbl_llm', '大模型'))
        self.btn_llm_start.setText(s.get('btn_start_play', 'Start'))
        self.btn_llm_stop.setText(s.get('btn_stop_play', 'Stop'))

        self.lbl_color_grab.setText(s.get('lbl_color_grab', 'Color Grab'))
        self.btn_cgrab_red.setText(s.get('color_red', 'Red'))
        self.btn_cgrab_green.setText(s.get('color_green', 'Green'))
        self.btn_cgrab_blue.setText(s.get('color_blue', 'Blue'))
        self.btn_cgrab_stop.setText(s.get('btn_stop_play', 'Stop'))

    def send_cmd(self, cmd, args):
        self.comm_manager.send_packet(0xFF, cmd, args)
