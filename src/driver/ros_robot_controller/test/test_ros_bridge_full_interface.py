import inspect
import sys
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_ros_robot_controller_registers_qt_bridge_subscriptions_and_services():
    from ros_robot_controller import ros_robot_controller_node

    source = inspect.getsource(ros_robot_controller_node.RosRobotController.__init__)

    expected_subscriptions = [
        "create_subscription(ServoId, '~/servo/set_id'",
        "create_subscription(ServoMode, '~/servo/set_mode'",
        "create_subscription(ServoOffset, '~/servo/set_offset'",
        "create_subscription(ServoPID, '~/servo/set_pid'",
        "create_subscription(ServoOverload, '~/servo/set_overload'",
        "create_subscription(ServoBaudRate, '~/servo/set_baud'",
        "create_subscription(ServoMaxTorque, '~/servo/set_max_torque'",
        "create_subscription(ServoAngleLimit, '~/servo/set_angle_limit'",
        "create_subscription(UInt8, '~/servo/calibrate'",
        "create_subscription(UInt8, '~/arm/set_torque'",
        "create_subscription(Empty, '~/arm/reset'",
        "create_subscription(SyncWriteServos, '~/servo/sync_write'",
        "create_subscription(UInt8, '~/arm/set_interp_mode'",
        "create_subscription(CoordLimits, '~/arm/set_coord_limits'",
        "create_subscription(ChassisConfig, '~/chassis/set_config'",
        "create_subscription(UInt8, '~/set_bt_mode'",
        "create_subscription(MacAddress, '~/set_ps3_mac'",
        "create_subscription(Empty, '~/factory_reset'",
        "create_subscription(UInt8, '~/set_lerobot_mode'",
    ]
    expected_services = [
        "create_service(GetFirmwareVersion, '~/get_firmware_version'",
        "create_service(GetServoOffset, '~/servo/get_offset'",
        "create_service(GetServoPID, '~/servo/get_pid'",
        "create_service(GetServoOverload, '~/servo/get_overload'",
        "create_service(GetServoBaud, '~/servo/get_baud'",
        "create_service(GetServoMaxTorque, '~/servo/get_max_torque'",
        "create_service(GetServoAngleLimit, '~/servo/get_angle_limit'",
        "create_service(GetCoordLimits, '~/arm/get_coord_limits'",
        "create_service(GetChassisConfig, '~/chassis/get_config'",
        "create_service(ScanWifiChannels, '~/scan_wifi_channels'",
    ]

    for subscription in expected_subscriptions:
        assert subscription in source
    for service in expected_services:
        assert service in source


def test_ros_robot_controller_qt_bridge_callbacks_forward_to_board():
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    class FakeBoard:
        def __init__(self):
            self.calls = []

        def set_servo_id(self, old_id, new_id):
            self.calls.append(("set_servo_id", old_id, new_id))

        def set_servo_mode(self, servo_id, mode):
            self.calls.append(("set_servo_mode", servo_id, mode))

        def set_pos_offset(self, servo_id, offset):
            self.calls.append(("set_pos_offset", servo_id, offset))

        def set_pid_param(self, servo_id, p, i, d, min_f):
            self.calls.append(("set_pid_param", servo_id, p, i, d, min_f))

        def servo_write_overload(self, servo_id, torque, time_val, thresh):
            self.calls.append(("servo_write_overload", servo_id, torque, time_val, thresh))

        def servo_write_baud(self, servo_id, baud):
            self.calls.append(("servo_write_baud", servo_id, baud))

        def servo_write_max_torque(self, servo_id, torque):
            self.calls.append(("servo_write_max_torque", servo_id, torque))

        def servo_write_angle_limit(self, servo_id, cw, ccw):
            self.calls.append(("servo_write_angle_limit", servo_id, cw, ccw))

        def servo_cali_pos(self, servo_id):
            self.calls.append(("servo_cali_pos", servo_id))

        def set_torque(self, enable):
            self.calls.append(("set_torque", enable))

        def arm_all_reset(self):
            self.calls.append(("arm_all_reset",))

        def sync_write_servos(self, positions, time_ms):
            self.calls.append(("sync_write_servos", list(positions), time_ms))

        def set_interp_mode(self, mode):
            self.calls.append(("set_interp_mode", mode))

        def set_coord_limits(self, *values):
            self.calls.append(("set_coord_limits",) + tuple(values))

        def set_chassis_config(self, chassis_type, wheel_dia, wheel_base, track_width, max_speed):
            self.calls.append(("set_chassis_config", chassis_type, wheel_dia, wheel_base, track_width, max_speed))

        def set_bt_mode(self, mode):
            self.calls.append(("set_bt_mode", mode))

        def set_ps3_mac(self, mac_bytes):
            self.calls.append(("set_ps3_mac", list(mac_bytes)))

        def factory_reset(self):
            self.calls.append(("factory_reset",))

        def set_lerobot_mode(self, mode):
            self.calls.append(("set_lerobot_mode", mode))

    controller = SimpleNamespace(board=FakeBoard())

    RosRobotController.servo_set_id_callback(controller, SimpleNamespace(old_id=1, new_id=2))
    RosRobotController.servo_set_mode_callback(controller, SimpleNamespace(id=3, mode=1))
    RosRobotController.servo_set_offset_callback(controller, SimpleNamespace(id=4, offset=-12))
    RosRobotController.servo_set_pid_callback(controller, SimpleNamespace(id=5, p=1, i=2, d=3, min_f=400))
    RosRobotController.servo_set_overload_callback(controller, SimpleNamespace(id=6, torque=7, time_val=8, thresh=9))
    RosRobotController.servo_set_baud_callback(controller, SimpleNamespace(id=1, baud=4))
    RosRobotController.servo_set_max_torque_callback(controller, SimpleNamespace(id=2, torque=900))
    RosRobotController.servo_set_angle_limit_callback(controller, SimpleNamespace(id=3, cw_limit=10, ccw_limit=200))
    RosRobotController.servo_calibrate_callback(controller, SimpleNamespace(data=4))
    RosRobotController.arm_set_torque_callback(controller, SimpleNamespace(data=1))
    RosRobotController.arm_reset_callback(controller, SimpleNamespace())
    RosRobotController.servo_sync_write_callback(controller, SimpleNamespace(positions=[100, 200], time_ms=300))
    RosRobotController.arm_set_interp_mode_callback(controller, SimpleNamespace(data=2))
    RosRobotController.arm_set_coord_limits_callback(
        controller,
        SimpleNamespace(
            xmin=-1, xmax=1, ymin=-2, ymax=2, zmin=-3, zmax=3,
            pmin=-4, pmax=4, rmin=-5, rmax=5, cmin=-6, cmax=6,
        ),
    )
    RosRobotController.chassis_set_config_callback(
        controller,
        SimpleNamespace(chassis_type=1, wheel_dia=2.5, wheel_base=3.5, track_width=4.5, max_speed=80),
    )
    RosRobotController.set_bt_mode_callback(controller, SimpleNamespace(data=1))
    RosRobotController.set_ps3_mac_callback(controller, SimpleNamespace(mac=[1, 2, 3, 4, 5, 6]))
    RosRobotController.factory_reset_callback(controller, SimpleNamespace())
    RosRobotController.set_lerobot_mode_callback(controller, SimpleNamespace(data=2))

    assert controller.board.calls == [
        ("set_servo_id", 1, 2),
        ("set_servo_mode", 3, 1),
        ("set_pos_offset", 4, -12),
        ("set_pid_param", 5, 1, 2, 3, 400),
        ("servo_write_overload", 6, 7, 8, 9),
        ("servo_write_baud", 1, 4),
        ("servo_write_max_torque", 2, 900),
        ("servo_write_angle_limit", 3, 10, 200),
        ("servo_cali_pos", 4),
        ("set_torque", 1),
        ("arm_all_reset",),
        ("sync_write_servos", [100, 200], 300),
        ("set_interp_mode", 2),
        ("set_coord_limits", -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6),
        ("set_chassis_config", 1, 2.5, 3.5, 4.5, 80),
        ("set_bt_mode", 1),
        ("set_ps3_mac", [1, 2, 3, 4, 5, 6]),
        ("factory_reset",),
        ("set_lerobot_mode", 2),
    ]


def test_ros_robot_controller_qt_bridge_read_services_return_board_values():
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    class FakeBoard:
        def get_firmware_version(self):
            return {"esp": "1.2.3", "at32": "4.5.6"}

        def get_pos_offset(self, servo_id):
            return {"id": servo_id, "offset": -10}

        def get_pid_param(self, servo_id):
            return {"id": servo_id, "p": 1, "i": 2, "d": 3, "min_f": 400}

        def servo_read_overload(self, servo_id):
            return {"id": servo_id, "torque": 1, "time": 2, "thresh": 3}

        def servo_read_baud(self, servo_id):
            return {"id": servo_id, "baud": 4}

        def servo_read_max_torque(self, servo_id):
            return {"id": servo_id, "torque": 900}

        def servo_read_angle_limit(self, servo_id):
            return {"id": servo_id, "cw": 10, "ccw": 200}

        def get_coord_limits(self):
            return {
                "xmin": -1, "xmax": 1, "ymin": -2, "ymax": 2, "zmin": -3, "zmax": 3,
                "pmin": -4, "pmax": 4, "rmin": -5, "rmax": 5, "cmin": -6, "cmax": 6,
            }

        def get_chassis_config(self):
            return {"type": 1, "wheel_dia": 2.5, "wheel_base": 3.5, "track_width": 4.5, "max_speed": 80}

        def scan_wifi_channels(self):
            return [{"ap_count": 1, "rssi": -40}, {"ap_count": 2, "rssi": -50}]

    controller = SimpleNamespace(board=FakeBoard())

    fw = RosRobotController.get_firmware_version_callback(controller, SimpleNamespace(), SimpleNamespace())
    offset = RosRobotController.get_servo_offset_callback(controller, SimpleNamespace(id=2), SimpleNamespace())
    pid = RosRobotController.get_servo_pid_callback(controller, SimpleNamespace(id=3), SimpleNamespace())
    overload = RosRobotController.get_servo_overload_callback(controller, SimpleNamespace(id=4), SimpleNamespace())
    baud = RosRobotController.get_servo_baud_callback(controller, SimpleNamespace(id=5), SimpleNamespace())
    max_torque = RosRobotController.get_servo_max_torque_callback(controller, SimpleNamespace(id=6), SimpleNamespace())
    angle = RosRobotController.get_servo_angle_limit_callback(controller, SimpleNamespace(id=1), SimpleNamespace())
    coord = RosRobotController.get_coord_limits_callback(controller, SimpleNamespace(), SimpleNamespace())
    chassis = RosRobotController.get_chassis_config_callback(controller, SimpleNamespace(), SimpleNamespace())
    scan = RosRobotController.scan_wifi_channels_callback(controller, SimpleNamespace(), SimpleNamespace())

    assert (fw.success, fw.esp_version, fw.at32_version) == (True, "1.2.3", "4.5.6")
    assert (offset.success, offset.id, offset.offset) == (True, 2, -10)
    assert (pid.success, pid.id, pid.p, pid.i, pid.d, pid.min_f) == (True, 3, 1, 2, 3, 400)
    assert (overload.success, overload.id, overload.torque, overload.time_val, overload.thresh) == (True, 4, 1, 2, 3)
    assert (baud.success, baud.id, baud.baud) == (True, 5, 4)
    assert (max_torque.success, max_torque.id, max_torque.torque) == (True, 6, 900)
    assert (angle.success, angle.id, angle.cw_limit, angle.ccw_limit) == (True, 1, 10, 200)
    assert (coord.success, coord.xmin, coord.cmax) == (True, -1, 6)
    assert (chassis.success, chassis.chassis_type, chassis.wheel_dia, chassis.max_speed) == (True, 1, 2.5, 80)
    assert (scan.success, list(scan.ap_counts), list(scan.rssi_values)) == (True, [1, 2], [-40, -50])
