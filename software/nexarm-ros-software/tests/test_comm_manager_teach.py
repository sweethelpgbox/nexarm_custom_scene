from nexarm_qt.comm_manager import CommManager
from nexarm_qt.constants import (
    AT32_SYS_ID,
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


class DummyEmpty:
    pass


def test_teach_status_packets_emit_state_signals():
    manager = CommManager()
    action_states = []
    sync_states = []
    manager.action_edit_status_received.connect(lambda *state: action_states.append(state))
    manager.sync_teach_status_received.connect(lambda *state: sync_states.append(state))

    manager.handle_packet(AT32_SYS_ID, CMD_ACTION_EDIT_QUERY, bytes([1, 1, 0, 2, 1]))
    manager.handle_packet(AT32_SYS_ID, CMD_SYNC_TEACH_QUERY, bytes([1, 0, 1, 3, 2, 1]))

    assert action_states == [(1, 1, 0, 258)]
    assert sync_states == [(1, 0, 1, 515, 1)]


def test_teach_ros_commands_publish_expected_topics(monkeypatch):
    manager = CommManager()
    manager._ros_types = {"Empty": DummyEmpty}
    published = []
    monkeypatch.setattr(manager, "_ros_publish", lambda key, msg: published.append((key, type(msg))) or True)

    commands = [
        (CMD_ACTION_EDIT_ENTER, "action_edit_enter"),
        (CMD_ACTION_EDIT_EXIT, "action_edit_exit"),
        (CMD_ACTION_EDIT_START, "action_edit_start"),
        (CMD_ACTION_EDIT_STOP, "action_edit_stop"),
        (CMD_ACTION_EDIT_PLAY, "action_edit_play"),
        (CMD_ACTION_EDIT_PLAY_STOP, "action_edit_play_stop"),
        (CMD_ACTION_EDIT_CLEAR, "action_edit_clear"),
        (CMD_ACTION_EDIT_QUERY, "action_edit_query"),
        (CMD_SYNC_TEACH_ENTER, "sync_teach_enter"),
        (CMD_SYNC_TEACH_EXIT, "sync_teach_exit"),
        (CMD_SYNC_TEACH_REC_START, "sync_teach_rec_start"),
        (CMD_SYNC_TEACH_REC_STOP, "sync_teach_rec_stop"),
        (CMD_SYNC_TEACH_PLAY, "sync_teach_play"),
        (CMD_SYNC_TEACH_PLAY_STOP, "sync_teach_play_stop"),
        (CMD_SYNC_TEACH_CLEAR, "sync_teach_clear"),
        (CMD_SYNC_TEACH_QUERY, "sync_teach_query"),
    ]

    for cmd, _key in commands:
        manager._send_sys_ros(cmd, [])

    assert published == [(key, DummyEmpty) for _cmd, key in commands]
