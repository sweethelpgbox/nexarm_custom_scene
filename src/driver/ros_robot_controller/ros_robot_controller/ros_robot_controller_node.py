#!/usr/bin/env python3
import os
import time
import yaml
try:
    from app import scene_play_registry
except Exception:
    scene_play_registry = None
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import UInt16, Empty, UInt8, Int8, Bool, Float32MultiArray, UInt8MultiArray
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller.stepper_runtime import StepperRuntime
from ros_robot_controller.scene_kinematics import (
    scene4_startup_rail_config,
    scene4_target_and_restore_params,
    scene_home_command,
)
from ros_robot_controller_msgs.srv import (
    BusServoCtrl, GetArmCoords, GetArmIK, GetArmFK, GetArmFullState,
    GetKinematicsParam, SetKinematicsParam, SetStepperPosition, MoveStepper,
    GetFirmwareVersion, GetServoOffset, GetServoPID, GetServoOverload,
    GetServoBaud, GetServoMaxTorque, GetServoAngleLimit, GetCoordLimits,
    GetChassisConfig, ScanWifiChannels,
)
from ros_robot_controller_msgs.msg import (
    ButtonState, BuzzerState, OLEDState, MotorState, MotorsState, MecanumState, TankState,
    ConveyorState, StepperRun, ArmCoords, ArmMoveInc, ArmServoSingle, EspnowState, ArmFullState,
    ServosPosition, KinematicsParam, ServoId, ServoMode, ServoOffset, ServoPID,
    ServoOverload, ServoBaudRate, ServoMaxTorque, ServoAngleLimit, CoordLimits,
    ChassisConfig, SyncWriteServos, MacAddress,
)


APP_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
STEPPER_SCENE_CONFIG = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
SCENE4_PLAY_CONFIG = "/home/ubuntu/ros2_ws/src/app/config/plays/scene4_slide_rail_sorting.yaml"
DEFAULT_TYPERC_PATH = "/home/ubuntu/ros2_ws/.typerc"
SCENE4_ID = "scene_4"
SCENE5_ID = "scene_5"
STARTUP_KINEMATICS_READ_TIMEOUT = 0.2
SCENE4_PLAY_KEYS = (
    "rail",
    "scene4_pick",
    "scene4_place",
    "scene4_absolute_positions",
    "scene4_grid",
    "scene4_shelf",
    "kinematics",
)


class RosRobotController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.board = Board()
        self.board.enable_reception(True)
        self.running = True
        self.cb_group = ReentrantCallbackGroup()
        self.stepper_runtime = StepperRuntime(self.board)
        self.scene_config_path = os.environ.get("CALIBRATION_SCENE_CONFIG_PATH", self._default_scene_config_path())
        self.scene4_kinematics_applied = False
        self.scene4_restore_kinematics = None
        self.scene4_last_name = None
        self.scene4_home_pose_sent = False
        self.scene_home_pose_sent_key = None
        self.scene_home_ready_at = 0.0
        self.scene4_rail_startup_reset_started = False
        self.scene4_rail_startup_reset_done = False
        self.scene_kinematics_applied_key = None
        self.scene_runtime_prepare_lock = threading.RLock()
        self.ready_services_created = False
        self.status_thread_started = False

        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.arm_full_state_pub = self.create_publisher(ArmFullState, '~/arm/full_state', 1)
        self.arm_joint_pub = self.create_publisher(Float32MultiArray, '~/arm/joint_states', 1)
        self.action_edit_status_pub = self.create_publisher(UInt8MultiArray, '~/action_edit/status', 1)
        self.sync_teach_status_pub = self.create_publisher(UInt8MultiArray, '~/sync_teach/status', 1)
        self.last_arm_state = None

        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_callback, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_callback, 5)
        self.create_subscription(MotorState, '~/chassis/set_single_motor', self.set_single_motor_callback, 5)
        self.create_subscription(MotorsState, '~/chassis/set_motors', self.set_motors_callback, 5)
        self.create_subscription(Empty, '~/chassis/stop_motors', self.stop_motors_callback, 5)
        # self.create_subscription(Twist, '~/chassis/set_mecanum', self.set_mecanum_callback, 5)
        self.create_subscription(Twist, '~/cmd_vel', self.set_mecanum_callback, 5)
        self.create_subscription(TankState, '~/chassis/set_tank', self.set_tank_callback, 5)
        self.create_subscription(Int8, '~/conveyor/set', self.set_conveyor_callback, 5)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception_callback, 5)
        self.create_subscription(Empty, '~/stepper/reset', self.stepper_reset_callback, 5)
        self.create_subscription(UInt8, '~/stepper/set_div', self.stepper_div_callback, 5)
        self.create_subscription(StepperRun, '~/stepper/run', self.stepper_run_callback, 5)
        self.create_subscription(ArmCoords, '~/arm/set_coords', self.set_arm_coords_callback, 5)
        self.create_subscription(ArmMoveInc, '~/arm/move_inc', self.arm_move_inc_callback, 5)
        self.create_subscription(ArmServoSingle, '~/arm/servo_single', self.arm_servo_single_callback, 5)
        self.create_subscription(KinematicsParam, '~/arm/set_kinematics', self.set_kinematics_callback, 5)
        self.create_subscription(EspnowState, '~/espnow/set', self.espnow_set_callback, 5)
        self.create_subscription(UInt8, '~/arm/set_move_acc', self.set_move_acc_callback, 5)
        self.create_subscription(ServoId, '~/servo/set_id', self.servo_set_id_callback, 5)
        self.create_subscription(ServoMode, '~/servo/set_mode', self.servo_set_mode_callback, 5)
        self.create_subscription(ServoMode, '~/servo/torque', self.servo_torque_callback, 5)
        self.create_subscription(ServoOffset, '~/servo/set_offset', self.servo_set_offset_callback, 5)
        self.create_subscription(ServoPID, '~/servo/set_pid', self.servo_set_pid_callback, 5)
        self.create_subscription(ServoOverload, '~/servo/set_overload', self.servo_set_overload_callback, 5)
        self.create_subscription(ServoBaudRate, '~/servo/set_baud', self.servo_set_baud_callback, 5)
        self.create_subscription(ServoMaxTorque, '~/servo/set_max_torque', self.servo_set_max_torque_callback, 5)
        self.create_subscription(ServoAngleLimit, '~/servo/set_angle_limit', self.servo_set_angle_limit_callback, 5)
        self.create_subscription(UInt8, '~/servo/calibrate', self.servo_calibrate_callback, 5)
        self.create_subscription(UInt8, '~/arm/set_torque', self.arm_set_torque_callback, 5)
        self.create_subscription(Empty, '~/arm/reset', self.arm_reset_callback, 5)
        self.create_subscription(SyncWriteServos, '~/servo/sync_write', self.servo_sync_write_callback, 5)
        self.create_subscription(UInt8, '~/arm/set_interp_mode', self.arm_set_interp_mode_callback, 5)
        self.create_subscription(CoordLimits, '~/arm/set_coord_limits', self.arm_set_coord_limits_callback, 5)
        self.create_subscription(ChassisConfig, '~/chassis/set_config', self.chassis_set_config_callback, 5)
        self.create_subscription(UInt8, '~/set_bt_mode', self.set_bt_mode_callback, 5)
        self.create_subscription(MacAddress, '~/set_ps3_mac', self.set_ps3_mac_callback, 5)
        self.create_subscription(Empty, '~/factory_reset', self.factory_reset_callback, 5)
        self.create_subscription(UInt8, '~/set_lerobot_mode', self.set_lerobot_mode_callback, 5)
        self.create_subscription(UInt8, '~/set_pc_sync_teach', self.set_pc_sync_teach_callback, 5)
        self.create_subscription(UInt8, '~/action_group/run', self.action_group_run_callback, 5)
        self.create_subscription(Empty, '~/action_group/stop', self.action_group_stop_callback, 5)
        self.create_subscription(UInt8, '~/action_group/erase', self.action_group_erase_callback, 5)
        self.create_subscription(UInt8MultiArray, '~/action_group/download', self.action_group_download_callback, 5)
        self.create_subscription(Empty, '~/action_edit/enter', self.action_edit_enter_callback, 5)
        self.create_subscription(Empty, '~/action_edit/exit', self.action_edit_exit_callback, 5)
        self.create_subscription(Empty, '~/action_edit/start', self.action_edit_start_callback, 5)
        self.create_subscription(Empty, '~/action_edit/stop', self.action_edit_stop_callback, 5)
        self.create_subscription(Empty, '~/action_edit/play', self.action_edit_play_callback, 5)
        self.create_subscription(Empty, '~/action_edit/play_stop', self.action_edit_play_stop_callback, 5)
        self.create_subscription(Empty, '~/action_edit/clear', self.action_edit_clear_callback, 5)
        self.create_subscription(Empty, '~/action_edit/query', self.action_edit_query_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/enter', self.sync_teach_enter_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/exit', self.sync_teach_exit_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/rec_start', self.sync_teach_rec_start_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/rec_stop', self.sync_teach_rec_stop_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/play', self.sync_teach_play_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/play_stop', self.sync_teach_play_stop_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/clear', self.sync_teach_clear_callback, 5)
        self.create_subscription(Empty, '~/sync_teach/query', self.sync_teach_query_callback, 5)
        self.create_service(GetArmIK, '~/arm/get_ik', self.get_arm_ik_callback, callback_group=self.cb_group)
        self.create_service(GetArmFK, '~/arm/get_fk', self.get_arm_fk_callback, callback_group=self.cb_group)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)

        self.create_subscription(UInt8, '~/set_oled_icon', self.set_oled_icon_callback, 5)

        self.create_service(BusServoCtrl, '~/bus_servo/ctrl', self.bus_servo_ctrl_callback, callback_group=self.cb_group)
        self.create_service(GetArmCoords, '~/arm/get_coords', self.get_arm_coords_callback, callback_group=self.cb_group)
        self.create_service(GetArmFullState, '~/arm/get_full_state', self.get_arm_full_state_callback, callback_group=self.cb_group)
        self.create_service(GetKinematicsParam, '~/arm/get_kinematics', self.get_kinematics_callback, callback_group=self.cb_group)
        self.create_service(SetKinematicsParam, '~/arm/set_kinematics_param', self.set_kinematics_service_callback, callback_group=self.cb_group)
        self.create_service(GetFirmwareVersion, '~/get_firmware_version', self.get_firmware_version_callback, callback_group=self.cb_group)
        self.create_service(GetServoOffset, '~/servo/get_offset', self.get_servo_offset_callback, callback_group=self.cb_group)
        self.create_service(GetServoPID, '~/servo/get_pid', self.get_servo_pid_callback, callback_group=self.cb_group)
        self.create_service(GetServoOverload, '~/servo/get_overload', self.get_servo_overload_callback, callback_group=self.cb_group)
        self.create_service(GetServoBaud, '~/servo/get_baud', self.get_servo_baud_callback, callback_group=self.cb_group)
        self.create_service(GetServoMaxTorque, '~/servo/get_max_torque', self.get_servo_max_torque_callback, callback_group=self.cb_group)
        self.create_service(GetServoAngleLimit, '~/servo/get_angle_limit', self.get_servo_angle_limit_callback, callback_group=self.cb_group)
        self.create_service(GetCoordLimits, '~/arm/get_coord_limits', self.get_coord_limits_callback, callback_group=self.cb_group)
        self.create_service(GetChassisConfig, '~/chassis/get_config', self.get_chassis_config_callback, callback_group=self.cb_group)
        self.create_service(ScanWifiChannels, '~/scan_wifi_channels', self.scan_wifi_channels_callback, callback_group=self.cb_group)
        self.create_service(SetStepperPosition, '~/stepper/set_position', self.set_stepper_position_callback, callback_group=self.cb_group)
        self.create_service(Trigger, '~/stepper/reset_wait', self.stepper_reset_wait_callback, callback_group=self.cb_group)
        self.create_service(MoveStepper, '~/move_stepper', self.move_stepper_callback, callback_group=self.cb_group)
        self.create_service(Trigger, '~/scene_runtime/prepare', self.prepare_scene_runtime_callback, callback_group=self.cb_group)

        self._create_ready_services_once()
        threading.Thread(target=self._startup_scene_home, daemon=True).start()

    @staticmethod
    def _default_scene_config_path():
        if scene_play_registry is not None:
            return scene_play_registry.scene_config_path_for()
        scene_name = RosRobotController._active_scene_from_env()
        if scene_name == SCENE4_ID:
            return STEPPER_SCENE_CONFIG
        if RosRobotController._chassis_type() == "Slide_Rails" and scene_name != SCENE5_ID:
            return STEPPER_SCENE_CONFIG
        return APP_SCENE_CONFIG

    @staticmethod
    def _chassis_type():
        if scene_play_registry is not None:
            return scene_play_registry.chassis_type_from_env()
        return os.environ.get("CHASSIS_TYPE", "") or RosRobotController._typerc_export_values(
            ("CHASSIS_TYPE",)
        ).get("CHASSIS_TYPE", "")

    @staticmethod
    def _active_scene_from_env():
        scene_name = (
            os.environ.get("CALIBRATION_CURRENT_SCENE")
            or os.environ.get("CALIBRATION_DEFAULT_SCENE")
            or os.environ.get("SCENE")
        )
        if scene_name:
            return scene_name
        values = RosRobotController._typerc_export_values(
            ("CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", "SCENE")
        )
        for key in ("CALIBRATION_CURRENT_SCENE", "CALIBRATION_DEFAULT_SCENE", "SCENE"):
            if values.get(key):
                return values[key]
        return ""

    @staticmethod
    def _typerc_export_values(keys):
        try:
            with open(os.environ.get("CALIBRATION_TYPERC_PATH", DEFAULT_TYPERC_PATH), "r", encoding="utf-8") as f:
                lines = f.read().splitlines()
        except Exception:
            return {}

        values = {}
        keys = tuple(keys)
        for line in lines:
            line = line.strip()
            if not line.startswith("export ") or "=" not in line:
                continue
            key, value = line[len("export "):].split("=", 1)
            key = key.strip()
            if key not in keys:
                continue
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
                value = value[1:-1]
            values[key] = value
        return values

    @staticmethod
    def _merge_scene_play_config(scene_name, scene_cfg):
        if scene_play_registry is not None:
            return scene_play_registry.merge_play_into_scene(scene_name, scene_cfg)
        if str(scene_name) != SCENE4_ID:
            return scene_cfg
        try:
            with open(SCENE4_PLAY_CONFIG, "r", encoding="utf-8") as f:
                play = yaml.safe_load(f) or {}
        except Exception:
            return scene_cfg
        if not isinstance(play, dict):
            return scene_cfg
        merged = dict(scene_cfg) if isinstance(scene_cfg, dict) else {}
        for key in SCENE4_PLAY_KEYS:
            if key in play:
                merged[key] = play[key]
        return merged

    def init_finish_callback(self, request, response):
        response.success = True
        response.message = "init finish"
        return response

    def _create_ready_services_once(self):
        if self.ready_services_created:
            return
        namespace = self.get_namespace().strip('/')
        self.create_service(Trigger, '~/controller_manager/init_finish', self.init_finish_callback)
        self.create_service(Trigger, '~/kinematics/init_finish', self.init_finish_callback)
        if not namespace:
            self.create_service(Trigger, '/controller_manager/init_finish', self.init_finish_callback)
            self.create_service(Trigger, '/kinematics/init_finish', self.init_finish_callback)
        self.ready_services_created = True
        self.get_logger().info("controller init services ready")
        self._start_status_thread_once()

    def _start_status_thread_once(self):
        if self.status_thread_started:
            return
        self.status_thread_started = True
        threading.Thread(target=self.pub_callback, daemon=True).start()

    def _load_active_scene_config(self):
        try:
            with open(self.scene_config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return "scene_1", {}

        scenes = data.get("scenes") if isinstance(data, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            return "scene_1", {}

        scene_name = str(data.get("current_scene", "scene_1"))
        if scene_name not in scenes:
            scene_name = next(iter(scenes.keys()))
        scene_cfg = scenes.get(scene_name, {})
        if not isinstance(scene_cfg, dict):
            scene_cfg = {}
        return scene_name, self._merge_scene_play_config(scene_name, scene_cfg)

    def _load_scene4_scene_config(self):
        try:
            with open(self.scene_config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return "scene_1", {}

        scenes = data.get("scenes") if isinstance(data, dict) else None
        if not isinstance(scenes, dict):
            return str(data.get("current_scene", "scene_1")), {}

        scene_cfg = scenes.get(SCENE4_ID, {})
        if not isinstance(scene_cfg, dict):
            scene_cfg = {}
        return str(data.get("current_scene", "scene_1")), self._merge_scene_play_config(SCENE4_ID, scene_cfg)

    def _start_scene4_rail_startup_reset_from_config(self):
        active_scene_name, scene_cfg = self._load_scene4_scene_config()
        rail = scene4_startup_rail_config(
            active_scene_name,
            scene_cfg,
            self._chassis_type(),
        )
        self._start_scene4_rail_startup_reset_once(rail)

    def _start_scene4_rail_startup_reset_once(self, rail):
        if self.scene4_rail_startup_reset_started or rail is None:
            return

        self.scene4_rail_startup_reset_started = True
        threading.Thread(
            target=self._scene4_rail_startup_reset_worker,
            args=(rail,),
            daemon=True,
        ).start()

    def _scene4_rail_startup_reset_worker(self, rail):
        self.scene4_rail_startup_reset_done = self._reset_scene4_rail(rail)

    def _reset_scene4_rail(self, rail):
        try:
            self.get_logger().info(
                "scene_4 rail startup reset started: "
                f"subdivision={rail['subdivision']}, reset_wait_sec={rail['reset_wait_sec']:.1f}"
            )
            result = self.stepper_runtime.move(
                position=0,
                absolute=True,
                reset_first=True,
                set_subdivision=True,
                subdivision=rail["subdivision"],
                wait_for_motion=True,
                reset_wait_sec=rail["reset_wait_sec"],
                speed_steps_per_sec=rail["speed_steps_per_sec"],
            )
            if result.success:
                self.get_logger().info(f"scene_4 rail startup reset completed: {result.message}")
            else:
                self.get_logger().warn(f"scene_4 rail startup reset failed: {result.message}")
            return bool(result.success)
        except Exception as exc:
            self.get_logger().error(f"scene_4 rail startup reset exception: {exc}")
            return False

    def _apply_scene4_kinematics(self, scene_cfg):
        if self.scene4_kinematics_applied:
            return True

        params = self.board.get_kinematics_param(timeout=STARTUP_KINEMATICS_READ_TIMEOUT)
        if params is None or len(params) < 9:
            target = self._scene_kinematics_params(scene_cfg)
            if target is None:
                self.get_logger().warn("scene_4 kinematics apply skipped: cannot read current params")
                return False
            self.board.set_kinematics_param(target)
            self.scene4_restore_kinematics = None
            self.scene4_kinematics_applied = True
            self.get_logger().warn(
                "scene_4 kinematics applied from configured params without reading current params: "
                f"linkage1={target[0]:.2f}, base_high={target[8]:.2f}"
            )
            return True

        target, restore = scene4_target_and_restore_params(params, scene_cfg)

        if any(abs(float(a) - float(b)) > 0.01 for a, b in zip(params[:9], target)):
            self.board.set_kinematics_param(target)
        self.scene4_restore_kinematics = restore
        self.scene4_kinematics_applied = True
        self.get_logger().info(
            f"scene_4 kinematics applied: linkage1 {float(params[0]):.2f} -> {target[0]:.2f}, "
            f"base_high {float(params[8]):.2f} -> {target[8]:.2f}"
        )
        return True

    @staticmethod
    def _scene_kinematics_params(scene_cfg):
        cfg = scene_cfg.get("kinematics", {}) if isinstance(scene_cfg.get("kinematics"), dict) else {}
        raw_params = cfg.get("params")
        if isinstance(raw_params, (list, tuple)) and len(raw_params) >= 9:
            try:
                return [float(v) for v in raw_params[:9]]
            except Exception:
                return None

        keys = (
            "linkage1_mm",
            "linkage2_mm",
            "linkage2_perp_mm",
            "linkage3_mm",
            "linkage3_perp_mm",
            "linkage4_mm",
            "fixed_offset_mm",
            "base_radius_mm",
            "base_high_mm",
        )
        if not all(key in cfg for key in keys):
            return None
        try:
            return [float(cfg[key]) for key in keys]
        except Exception:
            return None

    def _apply_scene_kinematics_from_config(self, scene_name, scene_cfg):
        target = self._scene_kinematics_params(scene_cfg)
        if target is None:
            self.scene_kinematics_applied_key = None
            return True

        key = (str(scene_name), tuple(round(v, 3) for v in target))
        if self.scene_kinematics_applied_key == key:
            return True

        current = self.board.get_kinematics_param(timeout=STARTUP_KINEMATICS_READ_TIMEOUT)
        if current is None or len(current) < 9:
            self.get_logger().warn(f"{scene_name} kinematics apply skipped: cannot read current params")
            return False

        if any(abs(float(a) - float(b)) > 0.01 for a, b in zip(current[:9], target)):
            self.board.set_kinematics_param(target)
            self.get_logger().info(
                f"{scene_name} kinematics applied: linkage1 {float(current[0]):.2f} -> {target[0]:.2f}, "
                f"base_high {float(current[8]):.2f} -> {target[8]:.2f}"
            )
        else:
            self.get_logger().info(f"{scene_name} kinematics already match configured params")
        self.scene_kinematics_applied_key = key
        return True

    def _send_scene4_home_pose_once(self, scene_cfg):
        if self.scene4_home_pose_sent:
            return
        home = scene_home_command(scene_cfg, self._jetarm_roll_to_driver_roll)
        self.board.set_arm_coords(
            home["x"],
            home["y"],
            home["z"],
            home["pitch"],
            home["driver_roll"],
            home["claw"],
            home["time_ms"],
        )
        self.scene4_home_pose_sent = True
        self.scene_home_ready_at = time.time() + max(float(home["time_ms"]) / 1000.0, 0.0)
        self.get_logger().info(
            f"scene_4 home pose sent: x={home['x']:.1f}, y={home['y']:.1f}, z={home['z']:.1f}, "
            f"pitch={home['pitch']:.1f}, roll={home['roll']:.1f}, claw={home['claw']:.1f}"
        )

    def _send_scene_home_pose_once(self, scene_name, scene_cfg):
        home = scene_home_command(scene_cfg, self._jetarm_roll_to_driver_roll)
        key = (
            str(scene_name),
            round(home["x"], 3),
            round(home["y"], 3),
            round(home["z"], 3),
            round(home["pitch"], 3),
            round(home["roll"], 3),
            round(home["claw"], 3),
        )
        if self.scene_home_pose_sent_key == key:
            return
        self.board.set_arm_coords(
            home["x"],
            home["y"],
            home["z"],
            home["pitch"],
            home["driver_roll"],
            home["claw"],
            home["time_ms"],
        )
        self.scene_home_pose_sent_key = key
        self.scene_home_ready_at = time.time() + max(float(home["time_ms"]) / 1000.0, 0.0)
        self.get_logger().info(
            f"{scene_name} home pose sent: x={home['x']:.1f}, y={home['y']:.1f}, z={home['z']:.1f}, "
            f"pitch={home['pitch']:.1f}, roll={home['roll']:.1f}, claw={home['claw']:.1f}"
        )

    def restore_scene4_kinematics(self):
        if not self.scene4_kinematics_applied:
            return
        if self.scene4_restore_kinematics is not None:
            self.board.set_kinematics_param(self.scene4_restore_kinematics)
            self.get_logger().info(
                f"scene_4 kinematics restored: linkage1={float(self.scene4_restore_kinematics[0]):.2f}, "
                f"base_high={float(self.scene4_restore_kinematics[8]):.2f}"
            )
        self.scene4_kinematics_applied = False
        self.scene4_restore_kinematics = None
        self.scene4_home_pose_sent = False
        self.scene_home_pose_sent_key = None
        self.scene_home_ready_at = 0.0
        self.scene_kinematics_applied_key = None

    def _startup_scene_home(self):
        """节点启动后自动将机械臂发送到当前场景的 home 位置（读取 .typerc 的 CALIBRATION_CURRENT_SCENE）。"""
        time.sleep(0.5)
        self.scene_runtime_callback()

    def scene_runtime_callback(self):
        scene_name, scene_cfg = self._load_active_scene_config()
        scene_ready = False
        if scene_name == SCENE4_ID:
            self.scene_kinematics_applied_key = None
            rail = scene4_startup_rail_config(
                scene_name,
                scene_cfg,
                self._chassis_type(),
            )
            self._start_scene4_rail_startup_reset_once(rail)
            if self._apply_scene4_kinematics(scene_cfg):
                self._send_scene4_home_pose_once(scene_cfg)
                scene_ready = time.time() >= self.scene_home_ready_at
        else:
            self.restore_scene4_kinematics()
            if self._apply_scene_kinematics_from_config(scene_name, scene_cfg):
                self._send_scene_home_pose_once(scene_name, scene_cfg)
                scene_ready = time.time() >= self.scene_home_ready_at
            self.scene4_home_pose_sent = False
        if scene_ready:
            self._create_ready_services_once()
        self.scene4_last_name = scene_name

    def prepare_scene_runtime_callback(self, request, response):
        with self.scene_runtime_prepare_lock:
            scene_name, scene_cfg = self._load_active_scene_config()
            try:
                if scene_name == SCENE4_ID:
                    rail = scene4_startup_rail_config(
                        scene_name,
                        scene_cfg,
                        self._chassis_type(),
                    )
                    if rail is not None and not self.scene4_rail_startup_reset_done:
                        self.scene4_rail_startup_reset_started = True
                        if not self._reset_scene4_rail(rail):
                            response.success = False
                            response.message = "scene_4 rail reset failed"
                            return response
                        self.scene4_rail_startup_reset_done = True
                    response.success = bool(self._apply_scene4_kinematics(scene_cfg))
                else:
                    self.restore_scene4_kinematics()
                    response.success = bool(self._apply_scene_kinematics_from_config(scene_name, scene_cfg))

                response.message = (
                    f"{scene_name} runtime prepared"
                    if response.success else
                    f"{scene_name} runtime prepare failed"
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"{scene_name} runtime prepare exception: {exc}")
            return response

    @staticmethod
    def _jetarm_roll_to_driver_roll(angle_deg):
        return -float(angle_deg)

    @staticmethod
    def _driver_roll_to_jetarm_roll(angle_deg):
        return -float(angle_deg)

    def set_bus_servo_position(self, msg):
        for i in msg.position:
            # self.board.arm_move_servo_single(i.id, i.position, 100)
            self.board.bus_servo_set_position(i.id, position=i.position, acc=100, speed=600) 

    def _driver_joint_angles_to_public(self, joint_angles):
        public_angles = [float(v) for v in joint_angles]
        if len(public_angles) >= 5:
            public_angles[4] = self._driver_roll_to_jetarm_roll(public_angles[4])
        return public_angles

    def set_buzzer_callback(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def set_oled_callback(self, msg):
        self.board.set_oled_text(msg.index, msg.text)

    def set_single_motor_callback(self, msg):
        self.board.set_single_motor(msg.id, msg.speed)
        
    def get_arm_ik_callback(self, request, response):
        res = self.board.set_arm_coords(
            request.x, request.y, request.z, request.pitch,
            self._jetarm_roll_to_driver_roll(request.roll), request.claw, time_ms=0, calc_only=True
        )
        if res is not None:
            response.success = True
            response.servos = res['servos']
            self.get_logger().info(f"IK逆解计算成功, 舵机脉冲: {response.servos}")
        else:
            response.success = False
            self.get_logger().error("IK逆解计算失败或超时")
        return response

    def get_arm_fk_callback(self, request, response):
        res = self.board.get_fk_coords(
            request.j1, request.j2, request.j3, request.j4,
            self._jetarm_roll_to_driver_roll(request.roll), request.claw
        )
        if res is not None:
            response.success = True
            response.x = float(res['x'])
            response.y = float(res['y'])
            response.z = float(res['z'])
            response.pitch = float(res['pitch'])
            response.roll = self._driver_roll_to_jetarm_roll(res['roll'])
            response.claw = float(res['claw'])
            self.get_logger().info(f"FK正解计算成功, 坐标: X={res['x']}, Y={res['y']}, Z={res['z']}")
        else:
            response.success = False
            self.get_logger().error("FK正解计算失败或超时")
        return response
        
    def set_motors_callback(self, msg):
        self.board.set_motor_speed(msg.speed1, msg.speed2, msg.speed3, msg.speed4)

    def stop_motors_callback(self, msg):
        self.board.stop_all_motors()

    @staticmethod
    def _cmd_vel_to_chassis_speed(value):
        value = float(value)
        if -1.0 <= value <= 1.0:
            value *= 100.0
        return max(-100, min(100, int(round(value))))

    def set_mecanum_callback(self, msg):
        # self.board.set_mecanum(msg.vx, msg.vy, msg.vz)
        vx = self._cmd_vel_to_chassis_speed(msg.linear.x)
        vy = self._cmd_vel_to_chassis_speed(msg.linear.y)
        vz = self._cmd_vel_to_chassis_speed(msg.angular.z)
        self.board.set_mecanum(vx, vy, vz)

    def set_tank_callback(self, msg):
        self.board.set_tank(msg.speed, msg.turn)
    
    def set_oled_icon_callback(self, msg):
        self.board.set_oled_icon(msg.data)
        
    def set_conveyor_callback(self, msg):
        self.board.set_conveyor(msg.data)

    def enable_reception_callback(self, msg):
        self.board.enable_reception(bool(msg.data))
        self.get_logger().info(f"enable_reception={bool(msg.data)}")

    def stepper_reset_callback(self, msg):
        self.board.stepper_reset()
        self.stepper_runtime.current_position = None

    def stepper_div_callback(self, msg):
        self.board.stepper_set_div(msg.data)

    def stepper_run_callback(self, msg):
        self.board.stepper_run(msg.steps)
        if self.stepper_runtime.current_position is not None:
            self.stepper_runtime.current_position += int(msg.steps)

    def stepper_reset_wait_callback(self, request, response):
        result = self.stepper_runtime.reset()
        response.success = result.success
        response.message = result.message
        return response

    def set_stepper_position_callback(self, request, response):
        try:
            result = self.stepper_runtime.move(
                position=request.position,
                absolute=request.absolute,
                reset_first=request.reset_first,
                set_subdivision=request.set_subdivision,
                subdivision=request.subdivision,
                wait_for_motion=request.wait_for_motion,
                reset_wait_sec=request.reset_wait_sec,
                speed_steps_per_sec=request.speed_steps_per_sec,
            )
            response.success = result.success
            response.message = result.message
            response.current_position = int(result.current_position)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.current_position = -1
        return response

    def move_stepper_callback(self, request, response):
        try:
            if int(request.steps) == 0:
                result = self.stepper_runtime.reset()
            else:
                result = self.stepper_runtime.move(
                    position=int(request.steps),
                    absolute=False,
                    reset_first=False,
                    set_subdivision=True,
                    subdivision=0x07,
                    wait_for_motion=True,
                )
            response.success = result.success
            response.message = result.message
            response.current_position = int(result.current_position)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.current_position = -1
        return response

    def set_kinematics_callback(self, msg):
        params = [float(v) for v in list(msg.params)[:9]]
        if len(params) != 9:
            self.get_logger().warn("set_kinematics ignored: expected 9 floats")
            return
        self.board.set_kinematics_param(params)

    def get_kinematics_callback(self, request, response):
        params = self.board.get_kinematics_param()
        if params is None or len(params) < 9:
            response.success = False
            response.params = []
        else:
            response.success = True
            response.params = [float(v) for v in params[:9]]
        return response

    def set_kinematics_service_callback(self, request, response):
        params = [float(v) for v in list(request.params)[:9]]
        if len(params) != 9:
            response.success = False
            response.message = "expected 9 kinematics parameters"
            return response
        self.board.set_kinematics_param(params)
        response.success = True
        response.message = "kinematics params sent"
        return response

    # def set_arm_coords_callback(self, msg):
    #     self.board.set_arm_coords(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.time_ms)
    def set_arm_coords_callback(self, msg):
        self.board.set_arm_coords(
            msg.x,
            msg.y,
            msg.z,
            msg.pitch,
            self._jetarm_roll_to_driver_roll(msg.roll),
            msg.claw,
            msg.time_ms,
        )
    # def set_arm_coords_callback(self, msg):
    #     self.board.set_arm_coords(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.time_ms)
    def arm_move_inc_callback(self, msg):
        self.board.arm_move_inc(
            msg.dx,
            msg.dy,
            msg.dz,
            msg.dpitch,
            self._jetarm_roll_to_driver_roll(msg.droll),
            msg.dclaw,
            msg.time_ms,
        )
        
    # def arm_move_inc_callback(self, msg):
    #     self.board.arm_move_inc(msg.dx, msg.dy, msg.dz, msg.dpitch, msg.droll, msg.time_ms)

    def arm_servo_single_callback(self, msg):
        self.board.arm_move_servo_single(msg.id, msg.pos, msg.time_ms)

    def espnow_set_callback(self, msg):
        self.board.espnow_set_channel(msg.channel)
        time.sleep(0.05)
        self.board.espnow_set_global_acc(msg.global_acc)
        time.sleep(0.05)
        self.board.espnow_sync_ctrl(msg.sync_enable)
        
    def set_move_acc_callback(self, msg):
        # msg.data 的范围是 0~254
        self.board.set_move_acc(msg.data)
        self.get_logger().info(f"已下发底层插补加速度: {msg.data}")

    def servo_set_id_callback(self, msg):
        self.board.set_servo_id(int(msg.old_id), int(msg.new_id))

    def servo_set_mode_callback(self, msg):
        self.board.set_servo_mode(int(msg.id), int(msg.mode))

    def servo_torque_callback(self, msg):
        self.board.bus_servo_enable_torque(int(msg.id), bool(int(msg.mode)))

    def servo_set_offset_callback(self, msg):
        self.board.set_pos_offset(int(msg.id), int(msg.offset))

    def servo_set_pid_callback(self, msg):
        self.board.set_pid_param(int(msg.id), int(msg.p), int(msg.i), int(msg.d), int(msg.min_f))

    def servo_set_overload_callback(self, msg):
        self.board.servo_write_overload(int(msg.id), int(msg.torque), int(msg.time_val), int(msg.thresh))

    def servo_set_baud_callback(self, msg):
        self.board.servo_write_baud(int(msg.id), int(msg.baud))

    def servo_set_max_torque_callback(self, msg):
        self.board.servo_write_max_torque(int(msg.id), int(msg.torque))

    def servo_set_angle_limit_callback(self, msg):
        self.board.servo_write_angle_limit(int(msg.id), int(msg.cw_limit), int(msg.ccw_limit))

    def servo_calibrate_callback(self, msg):
        self.board.servo_cali_pos(int(msg.data))

    def arm_set_torque_callback(self, msg):
        self.board.set_torque(int(msg.data))

    def arm_reset_callback(self, msg):
        self.board.arm_all_reset()

    def servo_sync_write_callback(self, msg):
        self.board.sync_write_servos([int(v) for v in list(msg.positions)], int(msg.time_ms))

    def arm_set_interp_mode_callback(self, msg):
        self.board.set_interp_mode(int(msg.data))

    def arm_set_coord_limits_callback(self, msg):
        self.board.set_coord_limits(
            int(msg.xmin), int(msg.xmax), int(msg.ymin), int(msg.ymax),
            int(msg.zmin), int(msg.zmax), int(msg.pmin), int(msg.pmax),
            int(msg.rmin), int(msg.rmax), int(msg.cmin), int(msg.cmax),
        )

    def chassis_set_config_callback(self, msg):
        self.board.set_chassis_config(
            int(msg.chassis_type),
            float(msg.wheel_dia),
            float(msg.wheel_base),
            float(msg.track_width),
            int(msg.max_speed),
        )

    def set_bt_mode_callback(self, msg):
        self.board.set_bt_mode(int(msg.data))

    def set_ps3_mac_callback(self, msg):
        self.board.set_ps3_mac([int(v) & 0xFF for v in list(msg.mac)[:6]])

    def factory_reset_callback(self, msg):
        self.board.factory_reset()

    def set_lerobot_mode_callback(self, msg):
        self.board.set_lerobot_mode(int(msg.data))

    @staticmethod
    def _set_common_success(response, result):
        response.success = result is not None
        return result is not None

    def get_firmware_version_callback(self, request, response):
        result = self.board.get_firmware_version()
        response.success = result is not None
        if result is not None:
            response.esp_version = str(result.get("esp", ""))
            response.at32_version = str(result.get("at32", ""))
        return response

    def get_servo_offset_callback(self, request, response):
        result = self.board.get_pos_offset(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.offset = int(result.get("offset", 0))
        return response

    def get_servo_pid_callback(self, request, response):
        result = self.board.get_pid_param(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.p = int(result.get("p", 0))
            response.i = int(result.get("i", 0))
            response.d = int(result.get("d", 0))
            response.min_f = int(result.get("min_f", 0))
        return response

    def get_servo_overload_callback(self, request, response):
        result = self.board.servo_read_overload(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.torque = int(result.get("torque", 0))
            response.time_val = int(result.get("time", result.get("time_val", 0)))
            response.thresh = int(result.get("thresh", 0))
        return response

    def get_servo_baud_callback(self, request, response):
        result = self.board.servo_read_baud(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.baud = int(result.get("baud", 0))
        return response

    def get_servo_max_torque_callback(self, request, response):
        result = self.board.servo_read_max_torque(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.torque = int(result.get("torque", 0))
        return response

    def get_servo_angle_limit_callback(self, request, response):
        result = self.board.servo_read_angle_limit(int(request.id))
        response.success = result is not None
        if result is not None:
            response.id = int(result.get("id", request.id))
            response.cw_limit = int(result.get("cw", result.get("cw_limit", 0)))
            response.ccw_limit = int(result.get("ccw", result.get("ccw_limit", 0)))
        return response

    def get_coord_limits_callback(self, request, response):
        result = self.board.get_coord_limits()
        response.success = result is not None
        if result is not None:
            response.xmin = int(result.get("xmin", 0))
            response.xmax = int(result.get("xmax", 0))
            response.ymin = int(result.get("ymin", 0))
            response.ymax = int(result.get("ymax", 0))
            response.zmin = int(result.get("zmin", 0))
            response.zmax = int(result.get("zmax", 0))
            response.pmin = int(result.get("pmin", 0))
            response.pmax = int(result.get("pmax", 0))
            response.rmin = int(result.get("rmin", 0))
            response.rmax = int(result.get("rmax", 0))
            response.cmin = int(result.get("cmin", 0))
            response.cmax = int(result.get("cmax", 0))
        return response

    def get_chassis_config_callback(self, request, response):
        result = self.board.get_chassis_config()
        response.success = result is not None
        if result is not None:
            response.chassis_type = int(result.get("type", result.get("chassis_type", 0)))
            response.wheel_dia = float(result.get("wheel_dia", 0.0))
            response.wheel_base = float(result.get("wheel_base", 0.0))
            response.track_width = float(result.get("track_width", 0.0))
            response.max_speed = int(result.get("max_speed", 0))
        return response

    def scan_wifi_channels_callback(self, request, response):
        result = self.board.scan_wifi_channels()
        response.success = result is not None
        if result is not None:
            response.ap_counts = [int(item.get("ap_count", 0)) & 0xFF for item in result]
            response.rssi_values = [int(item.get("rssi", 0)) for item in result]
        return response

    def set_pc_sync_teach_callback(self, msg):
        self.board.set_pc_sync_teach(int(msg.data))

    def action_group_run_callback(self, msg):
        self.board.action_group_run(int(msg.data))

    def action_group_stop_callback(self, msg):
        self.board.action_group_stop()

    def action_group_erase_callback(self, msg):
        self.board.action_group_erase(int(msg.data))

    def action_group_download_callback(self, msg):
        self.board.action_group_download_raw(list(msg.data))

    def action_edit_enter_callback(self, msg):
        self.board.action_edit_enter()

    def action_edit_exit_callback(self, msg):
        self.board.action_edit_exit()

    def action_edit_start_callback(self, msg):
        self.board.action_edit_start()

    def action_edit_stop_callback(self, msg):
        self.board.action_edit_stop()

    def action_edit_play_callback(self, msg):
        self.board.action_edit_play()

    def action_edit_play_stop_callback(self, msg):
        self.board.action_edit_play_stop()

    def action_edit_clear_callback(self, msg):
        self.board.action_edit_clear()

    def action_edit_query_callback(self, msg):
        status = self.board.get_action_edit_status()
        if status is None:
            return
        count = int(status.get("count", 0)) & 0xFFFF
        self.action_edit_status_pub.publish(UInt8MultiArray(data=[
            int(status.get("mode", 0)) & 0xFF,
            int(status.get("recording", 0)) & 0xFF,
            int(status.get("playing", 0)) & 0xFF,
            count & 0xFF,
            (count >> 8) & 0xFF,
        ]))

    def sync_teach_enter_callback(self, msg):
        self.board.sync_teach_enter()

    def sync_teach_exit_callback(self, msg):
        self.board.sync_teach_exit()

    def sync_teach_rec_start_callback(self, msg):
        self.board.sync_teach_rec_start()

    def sync_teach_rec_stop_callback(self, msg):
        self.board.sync_teach_rec_stop()

    def sync_teach_play_callback(self, msg):
        self.board.sync_teach_play()

    def sync_teach_play_stop_callback(self, msg):
        self.board.sync_teach_play_stop()

    def sync_teach_clear_callback(self, msg):
        self.board.sync_teach_clear()

    def sync_teach_query_callback(self, msg):
        status = self.board.get_sync_teach_status()
        if status is None:
            return
        count = int(status.get("count", 0)) & 0xFFFF
        self.sync_teach_status_pub.publish(UInt8MultiArray(data=[
            int(status.get("mode", 0)) & 0xFF,
            int(status.get("recording", 0)) & 0xFF,
            int(status.get("playing", 0)) & 0xFF,
            count & 0xFF,
            (count >> 8) & 0xFF,
            int(status.get("overflow", 0)) & 0xFF,
        ]))
        
    def bus_servo_ctrl_callback(self, request, response):
        try:
            if request.set_torque:
                self.board.bus_servo_enable_torque(request.id, request.torque_enable)
                time.sleep(0.05)
            if request.set_mode:
                self.board.bus_servo_set_mode(request.id, request.mode)
                time.sleep(0.05)
            if request.set_position:
                self.board.bus_servo_set_position(request.id, request.position, request.acc, request.speed)

            response.current_position = self.board.bus_servo_read_position(request.id) or 0
            response.current_voltage = self.board.bus_servo_read_voltage(request.id) or 0
            response.current_temperature = self.board.bus_servo_read_temperature(request.id) or 0
            response.current_mode = self.board.bus_servo_read_mode(request.id) or 0
            response.success = True
        except Exception:
            response.success = False
        return response

    def get_arm_coords_callback(self, request, response):
        coords = self.board.get_arm_coords()
        if coords:
            response.success = True
            response.x = float(coords[0])
            response.y = float(coords[1])
            response.z = float(coords[2])
        else:
            response.success = False
        return response

    def _read_real_arm_state(self):
        tcp_pose = self.board.get_real_tcp_pose()
        if tcp_pose is None:
            return None

        joints_res = self.board.get_real_joint_angles()
        joint_angles = [0.0] * 6
        if joints_res and 'joints' in joints_res:
            for i, joint in enumerate(joints_res['joints'][:6]):
                joint_angles[i] = float(joint.get('angle', 0.0))

        servos = None
        full_state = self.board.get_full_state()
        if full_state is not None:
            servos = [int(v) for v in full_state.get('servos', [])[:6]]

        if servos is None or len(servos) < 6:
            if joints_res and 'joints' in joints_res:
                servos = [int(joint.get('pulse', 0)) for joint in joints_res['joints'][:6]]
            else:
                servos = [0] * 6

        if len(servos) < 6:
            servos = servos + [0] * (6 - len(servos))

        state = {
            'x': float(tcp_pose['x']),
            'y': float(tcp_pose['y']),
            'z': float(tcp_pose['z']),
            'yaw': float(tcp_pose['yaw']),
            'pitch': float(tcp_pose['pitch']),
            'roll': self._driver_roll_to_jetarm_roll(tcp_pose['roll']),
            'claw': float(tcp_pose['claw']),
            'servos': servos[:6],
            'joint_angles': self._driver_joint_angles_to_public(joint_angles[:6]),
        }
        self.last_arm_state = state
        return state

    def get_arm_full_state_callback(self, request, response):
        state = self._read_real_arm_state() or self.last_arm_state
        if state is None:
            response.success = False
            return response

        response.success = True
        response.x = float(state['x'])
        response.y = float(state['y'])
        response.z = float(state['z'])
        response.pitch = float(state['pitch'])
        response.roll = float(state['roll'])
        response.claw = float(state['claw'])
        response.yaw = float(state['yaw'])
        response.servos = [int(v) for v in state['servos']]
        response.joint_angles = [float(v) for v in state['joint_angles']]
        return response

    # def pub_callback(self):
    #     count = 0
    #     while self.running:
    #         if count % 20 == 0:
    #             battery_data = self.board.get_battery()
    #             if battery_data is not None:
    #                 self.battery_pub.publish(UInt16(data=battery_data))
            
    #         button_data = self.board.get_button()
    #         if button_data is not None:
    #             msg = ButtonState()
    #             msg.id = int(button_data[0])
    #             msg.state = int(button_data[1])
    #             self.button_pub.publish(msg)
            
    #         count += 1
    #         time.sleep(0.05)
            
    #     if rclpy.ok(): rclpy.shutdown()
    def pub_callback(self):
        count = 0
        self.get_logger().info("状态发布线程已启动")

        while self.running and rclpy.ok():
            try:
                state = self._read_real_arm_state() or self.last_arm_state

                if state:
                    full_msg = ArmFullState()
                    full_msg.x = float(state['x'])
                    full_msg.y = float(state['y'])
                    full_msg.z = float(state['z'])
                    full_msg.pitch = float(state['pitch'])
                    full_msg.roll = float(state['roll'])
                    full_msg.claw = float(state['claw'])
                    full_msg.yaw = float(state['yaw'])
                    full_msg.servos = [int(s) for s in state['servos']]
                    full_msg.joint_angles = [float(v) for v in state['joint_angles']]
                    self.arm_full_state_pub.publish(full_msg)

                    array_msg = Float32MultiArray()
                    array_msg.data = [
                        full_msg.x, full_msg.y, full_msg.z,
                        full_msg.yaw, full_msg.pitch, full_msg.roll, full_msg.claw,
                        *[float(v) for v in full_msg.joint_angles],
                        *[float(v) for v in full_msg.servos],
                    ]
                    self.arm_joint_pub.publish(array_msg)

                button_data = self.board.get_button()
                if button_data is not None:
                    btn_msg = ButtonState()
                    btn_msg.id = int(button_data[0])
                    btn_msg.state = int(button_data[1])
                    self.button_pub.publish(btn_msg)

                if count % 20 == 0:
                    battery_data = self.board.get_battery()
                    if battery_data is not None:
                        self.battery_pub.publish(UInt16(data=int(battery_data)))

                count += 1
                if count >= 1000:
                    count = 0

            except Exception as e:
                self.get_logger().error(f"pub_callback 发生异常: {str(e)}")

            time.sleep(0.1)

        self.get_logger().info("状态发布线程已停止")

def main():
    rclpy.init()
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.restore_scene4_kinematics()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
