import subprocess

from nexarm_qt.comm_manager import CommManager


class FakeSerial:
    instances = []

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.closed = False
        self.writes = []
        FakeSerial.instances.append(self)

    @property
    def in_waiting(self):
        return 0

    def read(self, size=1):
        return b""

    def write(self, data):
        self.writes.append(bytes(data))

    def close(self):
        self.closed = True


def make_manager(monkeypatch, topic_info):
    calls = []

    def fake_check_output(cmd, *args, **kwargs):
        calls.append(("check_output", cmd))
        return topic_info

    def fake_run(cmd, *args, **kwargs):
        calls.append(("run", cmd))
        return subprocess.CompletedProcess(cmd, 0)

    FakeSerial.instances = []
    monkeypatch.setattr("nexarm_qt.comm_manager.serial.Serial", FakeSerial)
    monkeypatch.setattr("nexarm_qt.comm_manager.subprocess.check_output", fake_check_output)
    monkeypatch.setattr("nexarm_qt.comm_manager.subprocess.run", fake_run)
    monkeypatch.setattr(CommManager, "_read_all_offsets", lambda self: None)
    return CommManager(), calls


def published_values(calls):
    values = []
    for kind, cmd in calls:
        if kind != "run":
            continue
        joined = " ".join(str(part) for part in cmd)
        if "data: false" in joined:
            values.append(False)
        elif "data: true" in joined:
            values.append(True)
    return values


def test_serial_connect_pauses_and_disconnect_restores_ros_reception(monkeypatch):
    manager, calls = make_manager(monkeypatch, "Type: std_msgs/msg/Bool\nSubscription count: 1\n")

    manager.connect("/dev/ttyUSB0", 1000000)
    assert manager.is_connected
    assert published_values(calls) == [False]

    manager.disconnect()
    assert FakeSerial.instances[0].closed
    assert published_values(calls) == [False, True]


def test_serial_connect_does_not_publish_when_ros_reception_topic_has_no_subscription(monkeypatch):
    manager, calls = make_manager(monkeypatch, "Type: std_msgs/msg/Bool\nSubscription count: 0\n")

    manager.connect("/dev/ttyUSB0", 1000000)
    manager.disconnect()

    assert published_values(calls) == []
