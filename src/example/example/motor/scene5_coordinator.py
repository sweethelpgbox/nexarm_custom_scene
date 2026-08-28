#!/usr/bin/env python3
# coding: utf-8

import time
import yaml
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, SetBool
from interfaces.srv import SetInt64
from sdk.scene_context import load_scene_environment


CONFIG_PATH = '/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml'
PLAY_CONFIG_PATH = '/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml'


class Scene5Coordinator(Node):
    def __init__(self):
        super().__init__(
            'scene5_coordinator',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.scene_env = load_scene_environment()
        self.config_path = self.string_param('config_path', CONFIG_PATH)
        self.play_config_path = self.string_param('play_config_path', PLAY_CONFIG_PATH)
        self.config = self.load_config()
        play_config = self.load_yaml(self.play_config_path)
        dual = play_config.get(
            'scene5_dual_arm',
            self.config.get('scenes', {}).get('scene_5', {}).get('scene5_dual_arm', {}),
        )
        conveyor = dual.get('conveyor', {})
        self.conveyor_speed = int(conveyor.get('speed', 100))
        self.conveyor_stop_speed = int(conveyor.get('stop_speed', 0))
        self.conveyor_move_ms = int(conveyor.get('move_ms', 1200))
        self.conveyor_settle_ms = int(conveyor.get('settle_ms', 500))
        self.running = False
        self.conveyor_enabled = False
        self.worker = None
        self.lock = threading.RLock()
        default_service_prefix = self.scene_env.role_namespace if self.scene_env.is_scene5 else '/arm_a'
        self.service_prefix = self.string_param('service_prefix', default_service_prefix)
        self.arm_a_service_prefix = self.string_param('arm_a_service_prefix', '/arm_a')
        self.arm_b_service_prefix = self.string_param('arm_b_service_prefix', '/arm_b')

        default_conveyor_topic = (
            self.scene_env.controller_topic('conveyor/set')
            if self.scene_env.is_scene5 else '/arm_a/ros_robot_controller/conveyor/set'
        )
        default_topic = conveyor.get(
            'topic',
            default_conveyor_topic,
        )
        topic = self.string_param('conveyor_topic', default_topic)
        self.conveyor_pub = self.create_publisher(Int8, topic, 1)
        self.arm_a_load_client = self.create_client(
            Trigger,
            self.service_name(self.arm_a_service_prefix, 'scene5/arm_a/load_once'),
        )
        self.arm_a_start_client = self.create_client(
            Trigger,
            self.service_name(self.arm_a_service_prefix, 'scene5/arm_a/start'),
        )
        self.arm_a_stop_client = self.create_client(
            Trigger,
            self.service_name(self.arm_a_service_prefix, 'scene5/arm_a/stop'),
        )
        self.arm_a_home_client = self.create_client(
            Trigger,
            self.service_name(self.arm_a_service_prefix, 'scene5/arm_a/home'),
        )
        self.b_enter_client = self.create_client(
            Trigger,
            self.service_name(self.arm_b_service_prefix, 'scene5_waste_classification/enter'),
        )
        self.b_exit_client = self.create_client(
            Trigger,
            self.service_name(self.arm_b_service_prefix, 'scene5_waste_classification/exit'),
        )
        self.b_enable_client = self.create_client(
            SetBool,
            self.service_name(self.arm_b_service_prefix, 'scene5_waste_classification/enable_transport'),
        )

        self.create_service(Trigger, self.prefixed_service('scene5/start'), self.on_start)
        self.create_service(Trigger, self.prefixed_service('scene5/stop'), self.on_stop)
        self.create_service(Trigger, self.prefixed_service('scene5/arm_a/load_once_then_b'), self.on_load_once_then_b)
        self.create_service(Trigger, self.prefixed_service('scene5/conveyor/start'), self.on_conveyor_start)
        self.create_service(Trigger, self.prefixed_service('scene5/conveyor/stop'), self.on_conveyor_stop)
        self.create_service(SetInt64, self.prefixed_service('scene5/conveyor/set_speed'), self.on_conveyor_set_speed)
        self.get_logger().info(f'scene5 coordinator ready; conveyor topic={topic}')

    def string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None:
                return str(value)
        except Exception:
            pass
        return str(default)

    def load_config(self):
        with open(self.config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def load_yaml(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def service_name(self, prefix, suffix):
        prefix = str(prefix or '').strip().strip('/')
        suffix = str(suffix).strip().strip('/')
        return f'/{prefix}/{suffix}' if prefix else suffix

    def prefixed_service(self, suffix):
        return self.service_name(self.service_prefix, suffix)

    def wait_future(self, future, label, timeout_sec=30.0):
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise RuntimeError(f'{label} timed out')
            time.sleep(0.02)
        result = future.result()
        if result is None or not getattr(result, 'success', False):
            msg = getattr(result, 'message', 'no response') if result is not None else 'no response'
            raise RuntimeError(f'{label} failed: {msg}')
        return getattr(result, 'message', label)

    def call_trigger(self, client, label, timeout_sec=30.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{label} service unavailable')
        future = client.call_async(Trigger.Request())
        return self.wait_future(future, label, timeout_sec)

    def call_set_bool(self, client, value, label, timeout_sec=30.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{label} service unavailable')
        req = SetBool.Request()
        req.data = bool(value)
        future = client.call_async(req)
        return self.wait_future(future, label, timeout_sec)

    def missing_pipeline_services(self, timeout_sec=0.2):
        required = (
            (self.arm_a_start_client, 'scene5/arm_a/start'),
            (self.b_enter_client, 'scene5_waste_classification/enter'),
            (self.b_enable_client, 'scene5_waste_classification/enable_transport'),
        )
        missing = []
        for client, label in required:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                missing.append(label)
        return missing

    def set_conveyor(self, speed):
        msg = Int8()
        msg.data = max(-127, min(127, int(speed)))
        self.conveyor_pub.publish(msg)

    def set_conveyor_speed(self, speed):
        with self.lock:
            self.conveyor_speed = max(-127, min(127, int(speed)))
            enabled = self.conveyor_enabled or self.running
        if enabled:
            self.set_conveyor(self.conveyor_speed)

    def run_once(self):
        self.call_trigger(self.arm_a_load_client, 'arm_a load_once')
        with self.lock:
            if not self.running:
                return
            self.conveyor_enabled = True
        self.set_conveyor(self.conveyor_speed)
        time.sleep(max(0.0, self.conveyor_move_ms / 1000.0))
        self.set_conveyor(self.conveyor_stop_speed)
        with self.lock:
            self.conveyor_enabled = False
        time.sleep(max(0.0, self.conveyor_settle_ms / 1000.0))
        with self.lock:
            if not self.running:
                return
        self.call_trigger(self.b_enter_client, 'b enter')
        self.call_set_bool(self.b_enable_client, True, 'b enable_transport')

    def run_pipeline(self):
        try:
            self.get_logger().info('[scene5] starting pipeline: arm_a start ...')
            self.call_trigger(self.arm_a_start_client, 'arm_a start')
            self.get_logger().info('[scene5] arm_a started; b enter ...')
            self.call_trigger(self.b_enter_client, 'b enter')
            self.get_logger().info('[scene5] b entered; b enable_transport ...')
            self.call_set_bool(self.b_enable_client, True, 'b enable_transport')
            self.get_logger().info('[scene5] pipeline running')
            with self.lock:
                self.conveyor_enabled = True
                speed = self.conveyor_speed
            self.set_conveyor(speed)
            while rclpy.ok():
                with self.lock:
                    if not self.running:
                        break
                    speed = self.conveyor_speed
                    enabled = self.conveyor_enabled
                if enabled:
                    self.set_conveyor(speed)
                time.sleep(0.25)
        except Exception as ex:
            self.get_logger().error(f'[scene5] pipeline failed: {ex}')
            self.stop_all(call_b=True)
        finally:
            with self.lock:
                self.running = False

    def run_worker(self):
        try:
            self.run_once()
        except Exception as ex:
            self.get_logger().error(str(ex))
            self.stop_all(call_b=True)
        finally:
            with self.lock:
                self.running = False

    def start_worker(self, target=None):
        with self.lock:
            if self.running:
                return False
            self.running = True
            self.worker = threading.Thread(target=target or self.run_worker, daemon=True)
            self.worker.start()
        return True

    def stop_all(self, call_b=True):
        with self.lock:
            self.running = False
            self.conveyor_enabled = False
        self.set_conveyor(self.conveyor_stop_speed)
        if not call_b:
            return
        try:
            self.call_set_bool(self.b_enable_client, False, 'b disable_transport', timeout_sec=10.0)
        except Exception as ex:
            self.get_logger().warn(str(ex))
        try:
            self.call_trigger(self.b_exit_client, 'b exit', timeout_sec=10.0)
        except Exception as ex:
            self.get_logger().warn(str(ex))
        try:
            self.call_trigger(self.arm_a_stop_client, 'arm_a stop', timeout_sec=10.0)
        except Exception as ex:
            self.get_logger().warn(str(ex))

    def on_start(self, request, response):
        missing = self.missing_pipeline_services(timeout_sec=1.0)
        if missing:
            self.get_logger().warn(
                f'scene5 services not ready, retrying: {missing}'
            )
            time.sleep(2.0)
            missing = self.missing_pipeline_services(timeout_sec=2.0)
        if missing:
            response.success = False
            response.message = 'scene5 runtime service unavailable: ' + ', '.join(missing)
            self.get_logger().error(response.message)
            return response
        if self.start_worker(target=self.run_pipeline):
            response.success = True
            response.message = 'scene5 pipeline started'
        else:
            response.success = True
            response.message = 'scene5 pipeline already running'
        return response

    def on_stop(self, request, response):
        self.stop_all(call_b=True)
        response.success = True
        response.message = 'scene5 stopped'
        return response

    def on_load_once_then_b(self, request, response):
        if self.start_worker(target=self.run_worker):
            response.success = True
            response.message = 'scene5 one cycle started'
        else:
            response.success = False
            response.message = 'scene5 is already running'
        return response

    def on_conveyor_start(self, request, response):
        with self.lock:
            self.conveyor_enabled = True
        self.set_conveyor(self.conveyor_speed)
        response.success = True
        response.message = f'conveyor started speed={self.conveyor_speed}'
        return response

    def on_conveyor_stop(self, request, response):
        with self.lock:
            self.conveyor_enabled = False
        self.set_conveyor(self.conveyor_stop_speed)
        response.success = True
        response.message = 'conveyor stopped'
        return response

    def on_conveyor_set_speed(self, request, response):
        self.set_conveyor_speed(request.data)
        response.success = True
        response.message = f'conveyor speed set to {self.conveyor_speed}'
        return response


def main():
    rclpy.init()
    node = Scene5Coordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop_all(call_b=False)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
