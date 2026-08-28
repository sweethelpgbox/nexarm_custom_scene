import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget

from nexarm_qt.ui import main_window as main_window_module


_APP = None


class FakeCommManager(QObject):
    connection_status_changed = pyqtSignal(bool, str)
    ros_connect_success = True

    def __init__(self):
        super().__init__()
        self.is_connected = False
        self.connection_type = "serial"
        self.connect_ros_calls = 0
        self.connect_calls = []
        self.disconnect_calls = 0

    def get_available_ports(self):
        return ["/dev/ttyUSB0"]

    def connect_ros(self):
        self.connect_ros_calls += 1
        if not self.ros_connect_success:
            self.is_connected = False
            self.connection_type = "serial"
            self.connection_status_changed.emit(False, "ROS service not found")
            return
        self.is_connected = True
        self.connection_type = "ros"
        self.connection_status_changed.emit(True, "connected via ros")

    def connect(self, port_name, baudrate):
        self.connect_calls.append((port_name, baudrate))
        self.is_connected = True
        self.connection_type = "serial"
        self.connection_status_changed.emit(True, "connected")

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False
        self.connection_status_changed.emit(False, "Disconnected")

    def send_sys(self, *args, **kwargs):
        pass


class DummyTab(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.left_pane_layout = QVBoxLayout(self)

    def update_language(self, *args, **kwargs):
        pass

    def sync_current_servo_positions(self):
        pass


class DummyLogWidget(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.messages = []

    def update_language(self, *args, **kwargs):
        pass

    def append_text_log(self, message):
        self.messages.append(message)


class FakeCloseEvent:
    def __init__(self):
        self.accepted = False

    def accept(self):
        self.accepted = True


def app():
    global _APP
    instance = QApplication.instance()
    _APP = instance or _APP or QApplication([])
    return _APP


def make_window(monkeypatch, manager_cls=FakeCommManager):
    app()
    single_shots = []

    monkeypatch.setattr(main_window_module, "CommManager", manager_cls)
    monkeypatch.setattr(main_window_module, "ServoTab", DummyTab)
    monkeypatch.setattr(main_window_module, "CoordTab", DummyTab)
    monkeypatch.setattr(main_window_module, "PeripheralTab", DummyTab)
    monkeypatch.setattr(main_window_module, "SystemTab", DummyTab)
    monkeypatch.setattr(main_window_module, "TeachTab", DummyTab)
    monkeypatch.setattr(main_window_module, "LogWidget", DummyLogWidget)
    monkeypatch.setattr(
        main_window_module.QTimer,
        "singleShot",
        lambda delay_ms, callback: single_shots.append((delay_ms, callback)),
    )

    window = main_window_module.MainWindow()
    return window, single_shots


def test_main_window_auto_connects_via_ros_without_manual_serial_or_scene_controls(monkeypatch):
    window, single_shots = make_window(monkeypatch)

    assert not hasattr(window, "cb_scene")
    assert not hasattr(window, "btn_apply_scene")
    assert not hasattr(window, "btn_conn")
    assert not hasattr(window, "tab_servo_adv")
    assert not hasattr(window, "tab_environment")
    assert window.nav_list.count() == 5
    assert window.stack.count() == 5
    assert not hasattr(window, "tab_ai")
    assert hasattr(window, "tab_teach")
    labels = [window.nav_list.item(row).text() for row in range(window.nav_list.count())]
    assert "舵机高级设置" not in labels
    assert "环境设置" not in labels
    assert "Environment" not in labels
    assert "AI玩法控制" not in labels
    assert "AI Play Control" not in labels
    assert "示教编辑" in labels
    assert window.nav_list.spacing() <= 12
    assert 44 <= window.nav_list.item(0).sizeHint().height() <= 52
    assert window.nav_list.height() >= 300
    assert window.sidebar.layout().itemAt(2).widget() is window.sidebar_footer
    assert window.lbl_reception_owner.text() == "ROS接收权"
    assert window.lbl_status.text() == "未接管"
    assert "USB" not in window.lbl_reception_owner.text()

    assert single_shots
    delay_ms, callback = single_shots[0]
    assert delay_ms == 0

    callback()
    assert window.comm_manager.connect_ros_calls == 1
    assert window.comm_manager.connect_calls == []
    assert window.comm_manager.connection_type == "ros"
    assert window.lbl_status.text() == "Qt接管"
    assert not any("USB" in message for message in window.log_widget.messages)


def test_main_window_falls_back_to_serial_when_ros_bridge_is_unavailable(monkeypatch):
    class RosUnavailableCommManager(FakeCommManager):
        ros_connect_success = False

    window, single_shots = make_window(monkeypatch, RosUnavailableCommManager)
    single_shots[0][1]()

    assert window.comm_manager.connect_ros_calls == 1
    assert window.comm_manager.connect_calls == [("/dev/ttyUSB0", 1000000)]
    assert window.comm_manager.connection_type == "serial"


def test_main_window_disconnects_on_close(monkeypatch):
    window, single_shots = make_window(monkeypatch)
    single_shots[0][1]()

    event = FakeCloseEvent()
    window.closeEvent(event)

    assert event.accepted
    assert window.comm_manager.disconnect_calls == 1
