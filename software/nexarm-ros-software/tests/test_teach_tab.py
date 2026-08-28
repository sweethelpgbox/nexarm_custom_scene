import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication

from nexarm_qt.constants import (
    CMD_ACTION_EDIT_CLEAR,
    CMD_ACTION_EDIT_ENTER,
    CMD_ACTION_EDIT_EXIT,
    CMD_ACTION_EDIT_PLAY,
    CMD_ACTION_EDIT_PLAY_STOP,
    CMD_ACTION_EDIT_QUERY,
    CMD_ACTION_EDIT_START,
    CMD_ACTION_EDIT_STOP,
    CMD_SYNC_TEACH_CLEAR,
    CMD_SYNC_TEACH_ENTER,
    CMD_SYNC_TEACH_EXIT,
    CMD_SYNC_TEACH_PLAY,
    CMD_SYNC_TEACH_PLAY_STOP,
    CMD_SYNC_TEACH_QUERY,
    CMD_SYNC_TEACH_REC_START,
    CMD_SYNC_TEACH_REC_STOP,
)
from nexarm_qt.ui.teach_tab import TeachTab


_APP = None


class FakeCommManager(QObject):
    sync_teach_status_received = pyqtSignal(int, int, int, int, int)
    action_edit_status_received = pyqtSignal(int, int, int, int)

    def __init__(self):
        super().__init__()
        self.sent = []

    def send_sys(self, cmd, args=[]):
        self.sent.append((cmd, list(args)))


def app():
    global _APP
    instance = QApplication.instance()
    _APP = instance or _APP or QApplication([])
    return _APP


def test_teach_tab_buttons_send_expected_commands():
    app()
    comm = FakeCommManager()
    tab = TeachTab(comm, "zh")

    buttons = [
        (tab.btn_ae_enter, CMD_ACTION_EDIT_ENTER),
        (tab.btn_ae_rec, CMD_ACTION_EDIT_START),
        (tab.btn_ae_rec_stop, CMD_ACTION_EDIT_STOP),
        (tab.btn_ae_play, CMD_ACTION_EDIT_PLAY),
        (tab.btn_ae_play_stop, CMD_ACTION_EDIT_PLAY_STOP),
        (tab.btn_ae_clear, CMD_ACTION_EDIT_CLEAR),
        (tab.btn_ae_query, CMD_ACTION_EDIT_QUERY),
        (tab.btn_ae_exit, CMD_ACTION_EDIT_EXIT),
        (tab.btn_st_enter, CMD_SYNC_TEACH_ENTER),
        (tab.btn_st_rec, CMD_SYNC_TEACH_REC_START),
        (tab.btn_st_rec_stop, CMD_SYNC_TEACH_REC_STOP),
        (tab.btn_st_play, CMD_SYNC_TEACH_PLAY),
        (tab.btn_st_play_stop, CMD_SYNC_TEACH_PLAY_STOP),
        (tab.btn_st_clear, CMD_SYNC_TEACH_CLEAR),
        (tab.btn_st_query, CMD_SYNC_TEACH_QUERY),
        (tab.btn_st_exit, CMD_SYNC_TEACH_EXIT),
    ]

    for button, _cmd in buttons:
        button.setEnabled(True)
        button.click()

    assert [cmd for cmd, _args in comm.sent] == [cmd for _button, cmd in buttons]


def test_teach_tab_status_signals_update_labels_and_buttons():
    app()
    comm = FakeCommManager()
    tab = TeachTab(comm, "zh")

    comm.action_edit_status_received.emit(1, 1, 0, 258)
    assert tab.lbl_ae_status.text() == "录制中..."
    assert tab.lbl_ae_frames.text() == "帧数: 258"
    assert tab.btn_ae_rec_stop.isEnabled()
    assert not tab.btn_ae_exit.isEnabled()

    comm.sync_teach_status_received.emit(1, 0, 1, 515, 0)
    assert tab.lbl_st_status.text() == "播放中..."
    assert tab.lbl_st_frames.text() == "帧数: 515"
    assert tab.btn_st_play_stop.isEnabled()

    comm.sync_teach_status_received.emit(1, 0, 0, 516, 1)
    assert tab.lbl_st_status.text() == "溢出!"
