from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from nexarm_qt.constants import *
from nexarm_qt.translations import STRINGS


class TeachTab(QWidget):
    def __init__(self, comm_manager, lang="zh"):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        self._action_editing = False
        self._action_recording = False
        self._action_playing = False
        self._sync_editing = False
        self._sync_recording = False
        self._sync_playing = False
        self.setup_ui()
        self.comm_manager.sync_teach_status_received.connect(self._on_sync_status)
        self.comm_manager.action_edit_status_received.connect(self._on_action_status)

    def setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 16, 16, 16)
        outer.setSpacing(12)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(12)

        self.grp_action = QGroupBox(STRINGS[self.lang].get("grp_action_edit", "动作编辑"))
        self._build_action_edit(self.grp_action)
        layout.addWidget(self.grp_action)

        self.grp_sync = QGroupBox(STRINGS[self.lang].get("grp_sync_teach", "同步器示教"))
        self._build_sync_teach(self.grp_sync)
        layout.addWidget(self.grp_sync)

        layout.addStretch()
        scroll.setWidget(container)
        outer.addWidget(scroll)

    def _build_action_edit(self, group):
        grid = QGridLayout(group)
        grid.setSpacing(10)
        strings = STRINGS[self.lang]

        self.lbl_ae_status = self._status_label(strings.get("lbl_teach_idle", "空闲"))
        self.lbl_ae_frames = self._frames_label()
        self.btn_ae_enter = QPushButton(strings.get("btn_teach_enter", "进入编辑"))
        self.btn_ae_exit = QPushButton(strings.get("btn_teach_exit", "退出编辑"))
        self.btn_ae_rec = QPushButton(strings.get("btn_teach_rec_start", "开始录制"))
        self.btn_ae_rec_stop = QPushButton(strings.get("btn_teach_rec_stop", "停止录制"))
        self.btn_ae_play = QPushButton(strings.get("btn_teach_play", "播放动作"))
        self.btn_ae_play_stop = QPushButton(strings.get("btn_teach_play_stop", "停止播放"))
        self.btn_ae_clear = QPushButton(strings.get("btn_teach_clear", "清除动作"))
        self.btn_ae_query = QPushButton(strings.get("btn_teach_query", "查询状态"))

        for button, color in (
            (self.btn_ae_enter, "blue"),
            (self.btn_ae_exit, "gray"),
            (self.btn_ae_rec, "orange"),
            (self.btn_ae_rec_stop, "red"),
            (self.btn_ae_play, "green"),
            (self.btn_ae_play_stop, "red"),
            (self.btn_ae_clear, "red"),
            (self.btn_ae_query, "gray"),
        ):
            self._style_btn(button, color)

        grid.addWidget(self.lbl_ae_status, 0, 0, 1, 2)
        grid.addWidget(self.lbl_ae_frames, 1, 0, 1, 2)
        grid.addWidget(self.btn_ae_enter, 2, 0)
        grid.addWidget(self.btn_ae_exit, 2, 1)
        grid.addWidget(self.btn_ae_rec, 3, 0)
        grid.addWidget(self.btn_ae_rec_stop, 3, 1)
        grid.addWidget(self.btn_ae_play, 4, 0)
        grid.addWidget(self.btn_ae_play_stop, 4, 1)
        grid.addWidget(self.btn_ae_clear, 5, 0)
        grid.addWidget(self.btn_ae_query, 5, 1)

        self.btn_ae_enter.clicked.connect(self._ae_enter)
        self.btn_ae_exit.clicked.connect(self._ae_exit)
        self.btn_ae_rec.clicked.connect(self._ae_rec_start)
        self.btn_ae_rec_stop.clicked.connect(self._ae_rec_stop)
        self.btn_ae_play.clicked.connect(self._ae_play)
        self.btn_ae_play_stop.clicked.connect(self._ae_play_stop)
        self.btn_ae_clear.clicked.connect(self._ae_clear)
        self.btn_ae_query.clicked.connect(self._ae_query)
        self._update_ae_btns()

    def _build_sync_teach(self, group):
        grid = QGridLayout(group)
        grid.setSpacing(10)
        strings = STRINGS[self.lang]

        self.lbl_st_status = self._status_label(strings.get("lbl_teach_idle", "空闲"))
        self.lbl_st_frames = self._frames_label()
        self.btn_st_enter = QPushButton(strings.get("btn_sync_enter", "进入同步器示教"))
        self.btn_st_exit = QPushButton(strings.get("btn_sync_exit", "退出同步器示教"))
        self.btn_st_rec = QPushButton(strings.get("btn_teach_rec_start", "开始录制"))
        self.btn_st_rec_stop = QPushButton(strings.get("btn_teach_rec_stop", "停止录制"))
        self.btn_st_play = QPushButton(strings.get("btn_teach_play", "播放动作"))
        self.btn_st_play_stop = QPushButton(strings.get("btn_teach_play_stop", "停止播放"))
        self.btn_st_clear = QPushButton(strings.get("btn_sync_clear", "清空数据"))
        self.btn_st_query = QPushButton(strings.get("btn_teach_query", "查询状态"))

        for button, color in (
            (self.btn_st_enter, "blue"),
            (self.btn_st_exit, "gray"),
            (self.btn_st_rec, "orange"),
            (self.btn_st_rec_stop, "red"),
            (self.btn_st_play, "green"),
            (self.btn_st_play_stop, "red"),
            (self.btn_st_clear, "red"),
            (self.btn_st_query, "gray"),
        ):
            self._style_btn(button, color)

        grid.addWidget(self.lbl_st_status, 0, 0, 1, 2)
        grid.addWidget(self.lbl_st_frames, 1, 0, 1, 2)
        grid.addWidget(self.btn_st_enter, 2, 0)
        grid.addWidget(self.btn_st_exit, 2, 1)
        grid.addWidget(self.btn_st_rec, 3, 0)
        grid.addWidget(self.btn_st_rec_stop, 3, 1)
        grid.addWidget(self.btn_st_play, 4, 0)
        grid.addWidget(self.btn_st_play_stop, 4, 1)
        grid.addWidget(self.btn_st_clear, 5, 0)
        grid.addWidget(self.btn_st_query, 5, 1)

        self.btn_st_enter.clicked.connect(self._st_enter)
        self.btn_st_exit.clicked.connect(self._st_exit)
        self.btn_st_rec.clicked.connect(self._st_rec_start)
        self.btn_st_rec_stop.clicked.connect(self._st_rec_stop)
        self.btn_st_play.clicked.connect(self._st_play)
        self.btn_st_play_stop.clicked.connect(self._st_play_stop)
        self.btn_st_clear.clicked.connect(self._st_clear)
        self.btn_st_query.clicked.connect(self._st_query)
        self._update_st_btns()

    def _status_label(self, text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("color: #FA8F01; font-size: 11pt; font-weight: bold;")
        return label

    def _frames_label(self):
        label = QLabel(STRINGS[self.lang].get("lbl_frames", "帧数") + ": --")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("color: #B0BEC5; font-size: 10pt;")
        return label

    def _style_btn(self, button, color):
        colors = {
            "blue": ("#1976D2", "#1E88E5", "#1565C0"),
            "orange": ("#FA8F01", "#FB8C00", "#E65100"),
            "red": ("#D32F2F", "#E53935", "#B71C1C"),
            "green": ("#388E3C", "#43A047", "#2E7D32"),
            "gray": ("#546E7A", "#607D8B", "#37474F"),
        }
        bg, hover, pressed = colors.get(color, colors["gray"])
        button.setMinimumHeight(42)
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: white;
                font-weight: bold;
                font-size: 10pt;
                padding: 8px 14px;
                border-radius: 5px;
            }}
            QPushButton:hover {{ background-color: {hover}; }}
            QPushButton:pressed {{ background-color: {pressed}; }}
            QPushButton:disabled {{ background-color: #455A64; color: #78909C; }}
        """)

    def _ae_enter(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_ENTER)
        self._action_editing = True
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_editing", "编辑模式"))
        self._update_ae_btns()

    def _ae_exit(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_EXIT)
        self._action_editing = False
        self._action_recording = False
        self._action_playing = False
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_idle", "空闲"))
        self._update_ae_btns()

    def _ae_rec_start(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_START)
        self._action_recording = True
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_recording", "录制中..."))
        self._update_ae_btns()

    def _ae_rec_stop(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_STOP)
        self._action_recording = False
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_editing", "编辑模式"))
        self._update_ae_btns()

    def _ae_play(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_PLAY)
        self._action_playing = True
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_playing", "播放中..."))
        self._update_ae_btns()

    def _ae_play_stop(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_PLAY_STOP)
        self._action_playing = False
        self.lbl_ae_status.setText(STRINGS[self.lang].get("lbl_teach_editing", "编辑模式"))
        self._update_ae_btns()

    def _ae_clear(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_CLEAR)

    def _ae_query(self):
        self.comm_manager.send_sys(CMD_ACTION_EDIT_QUERY)

    def _st_enter(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_ENTER)
        self._sync_editing = True
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_sync_editing", "同步器示教模式"))
        self._update_st_btns()

    def _st_exit(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_EXIT)
        self._sync_editing = False
        self._sync_recording = False
        self._sync_playing = False
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_teach_idle", "空闲"))
        self._update_st_btns()

    def _st_rec_start(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_REC_START)
        self._sync_recording = True
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_teach_recording", "录制中..."))
        self._update_st_btns()

    def _st_rec_stop(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_REC_STOP)
        self._sync_recording = False
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_sync_editing", "同步器示教模式"))
        self._update_st_btns()

    def _st_play(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_PLAY)
        self._sync_playing = True
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_teach_playing", "播放中..."))
        self._update_st_btns()

    def _st_play_stop(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_PLAY_STOP)
        self._sync_playing = False
        self.lbl_st_status.setText(STRINGS[self.lang].get("lbl_sync_editing", "同步器示教模式"))
        self._update_st_btns()

    def _st_clear(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_CLEAR)
        self.lbl_st_frames.setText(STRINGS[self.lang].get("lbl_frames", "帧数") + ": 0")

    def _st_query(self):
        self.comm_manager.send_sys(CMD_SYNC_TEACH_QUERY)

    def _update_ae_btns(self):
        editing = self._action_editing
        recording = self._action_recording
        playing = self._action_playing
        self.btn_ae_enter.setEnabled(not editing)
        self.btn_ae_exit.setEnabled(editing and not recording)
        self.btn_ae_rec.setEnabled(editing and not recording and not playing)
        self.btn_ae_rec_stop.setEnabled(recording)
        self.btn_ae_play.setEnabled(editing and not recording and not playing)
        self.btn_ae_play_stop.setEnabled(playing)
        self.btn_ae_clear.setEnabled(editing and not recording and not playing)

    def _update_st_btns(self):
        editing = self._sync_editing
        recording = self._sync_recording
        playing = self._sync_playing
        self.btn_st_enter.setEnabled(not editing)
        self.btn_st_exit.setEnabled(editing and not recording)
        self.btn_st_rec.setEnabled(editing and not recording and not playing)
        self.btn_st_rec_stop.setEnabled(recording)
        self.btn_st_play.setEnabled(editing and not recording and not playing)
        self.btn_st_play_stop.setEnabled(playing)
        self.btn_st_clear.setEnabled(editing and not recording and not playing)

    def _on_action_status(self, mode, recording, playing, count):
        self._action_editing = bool(mode)
        self._action_recording = bool(recording)
        self._action_playing = bool(playing)
        self._update_ae_btns()

        strings = STRINGS[self.lang]
        if playing:
            self.lbl_ae_status.setText(strings.get("lbl_teach_playing", "播放中..."))
        elif recording:
            self.lbl_ae_status.setText(strings.get("lbl_teach_recording", "录制中..."))
        elif mode:
            self.lbl_ae_status.setText(strings.get("lbl_teach_editing", "编辑模式"))
        else:
            self.lbl_ae_status.setText(strings.get("lbl_teach_idle", "空闲"))
        self.lbl_ae_frames.setText(strings.get("lbl_frames", "帧数") + f": {int(count)}")

    def _on_sync_status(self, mode, recording, playing, count, overflow):
        self._sync_editing = bool(mode)
        self._sync_recording = bool(recording)
        self._sync_playing = bool(playing)
        self._update_st_btns()

        strings = STRINGS[self.lang]
        if overflow:
            self.lbl_st_status.setText(strings.get("lbl_sync_overflow", "溢出!"))
            self.lbl_st_status.setStyleSheet("color: #F44336; font-size: 11pt; font-weight: bold;")
        else:
            self.lbl_st_status.setStyleSheet("color: #FA8F01; font-size: 11pt; font-weight: bold;")
            if playing:
                self.lbl_st_status.setText(strings.get("lbl_teach_playing", "播放中..."))
            elif recording:
                self.lbl_st_status.setText(strings.get("lbl_teach_recording", "录制中..."))
            elif mode:
                self.lbl_st_status.setText(strings.get("lbl_sync_editing", "同步器示教模式"))
            else:
                self.lbl_st_status.setText(strings.get("lbl_teach_idle", "空闲"))
        self.lbl_st_frames.setText(strings.get("lbl_frames", "帧数") + f": {int(count)}")

    def update_language(self, lang):
        self.lang = lang
        strings = STRINGS[lang]
        self.grp_action.setTitle(strings.get("grp_action_edit", "Action Edit"))
        self.grp_sync.setTitle(strings.get("grp_sync_teach", "Sync Teach"))
        self.btn_ae_enter.setText(strings.get("btn_teach_enter", "Enter Edit"))
        self.btn_ae_exit.setText(strings.get("btn_teach_exit", "Exit Edit"))
        self.btn_ae_rec.setText(strings.get("btn_teach_rec_start", "Start Record"))
        self.btn_ae_rec_stop.setText(strings.get("btn_teach_rec_stop", "Stop Record"))
        self.btn_ae_play.setText(strings.get("btn_teach_play", "Play"))
        self.btn_ae_play_stop.setText(strings.get("btn_teach_play_stop", "Stop Play"))
        self.btn_ae_clear.setText(strings.get("btn_teach_clear", "Clear"))
        self.btn_ae_query.setText(strings.get("btn_teach_query", "Query Status"))
        self.btn_st_enter.setText(strings.get("btn_sync_enter", "Enter Sync Teach"))
        self.btn_st_exit.setText(strings.get("btn_sync_exit", "Exit Sync Teach"))
        self.btn_st_rec.setText(strings.get("btn_teach_rec_start", "Start Record"))
        self.btn_st_rec_stop.setText(strings.get("btn_teach_rec_stop", "Stop Record"))
        self.btn_st_play.setText(strings.get("btn_teach_play", "Play"))
        self.btn_st_play_stop.setText(strings.get("btn_teach_play_stop", "Stop Play"))
        self.btn_st_clear.setText(strings.get("btn_sync_clear", "Clear Data"))
        self.btn_st_query.setText(strings.get("btn_teach_query", "Query Status"))
        self._on_action_status(
            int(self._action_editing),
            int(self._action_recording),
            int(self._action_playing),
            self._current_frame_count(self.lbl_ae_frames),
        )
        self._on_sync_status(
            int(self._sync_editing),
            int(self._sync_recording),
            int(self._sync_playing),
            self._current_frame_count(self.lbl_st_frames),
            0,
        )

    @staticmethod
    def _current_frame_count(label):
        try:
            return int(label.text().split(":", 1)[1].strip())
        except Exception:
            return 0
