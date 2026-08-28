from nexarm_qt.comm_manager import CommManager
from nexarm_qt.constants import (
    CMD_CONVEYOR_SET,
    SERVO_CMD_WRITE,
    SERVO_REG_ACC,
    SERVO_REG_TORQUE,
)


class DummyUInt8:
    def __init__(self):
        self.data = 0


class DummyInt8:
    def __init__(self):
        self.data = 0


class DummyServoPosition:
    def __init__(self):
        self.id = 0
        self.position = 0


class DummyServosPosition:
    def __init__(self):
        self.position = []


class DummyServoMode:
    def __init__(self):
        self.id = 0
        self.mode = 0


class DummyBusServoCtrlReq:
    def __init__(self):
        self.id = 0
        self.set_torque = False
        self.torque_enable = False
        self.set_position = False
        self.position = 0
        self.acc = 0
        self.speed = 0


def test_ros_servo_write_uses_async_topics_not_blocking_service(monkeypatch):
    manager = CommManager()
    manager._ros_types = {
        "UInt8": DummyUInt8,
        "ServoMode": DummyServoMode,
        "ServoPosition": DummyServoPosition,
        "ServosPosition": DummyServosPosition,
        "BusServoCtrlReq": DummyBusServoCtrlReq,
    }
    published = []
    service_calls = []
    monkeypatch.setattr(manager, "_ros_publish", lambda key, msg: published.append((key, msg)) or True)
    monkeypatch.setattr(manager, "_ros_call", lambda *args, **kwargs: service_calls.append((args, kwargs)))

    manager._send_packet_ros(2, SERVO_CMD_WRITE, [SERVO_REG_TORQUE, 0])
    manager._send_packet_ros(3, SERVO_CMD_WRITE, [SERVO_REG_ACC, 5, 0x34, 0x12, 0, 0, 0x78, 0x56])

    assert service_calls == []
    assert published[0][0] == "servo_torque"
    assert (published[0][1].id, published[0][1].mode) == (2, 0)
    assert published[1][0] == "bus_servo_set_position"
    assert len(published[1][1].position) == 1
    pos_msg = published[1][1].position[0]
    assert (pos_msg.id, pos_msg.position) == (3, 0x1234)


def test_ros_conveyor_uses_signed_int8_message(monkeypatch):
    manager = CommManager()
    manager._ros_types = {"Int8": DummyInt8}
    published = []
    monkeypatch.setattr(manager, "_ros_publish", lambda key, msg: published.append((key, msg)) or True)

    manager._send_sys_ros(CMD_CONVEYOR_SET, [0xCE])

    assert len(published) == 1
    assert published[0][0] == "conveyor_set"
    assert isinstance(published[0][1], DummyInt8)
    assert published[0][1].data == -50
