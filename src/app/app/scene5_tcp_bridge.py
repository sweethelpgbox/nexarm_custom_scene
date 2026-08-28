#!/usr/bin/env python3
# encoding: utf-8
"""
Scene 5 ROS domain bridge.

The executable name is kept as ``scene5_tcp_bridge`` for launch compatibility.
A role runs one process with two ROS contexts and bridges:

Topics:
- B result image: B domain -> A domain
- B conveyor command: A domain -> B domain

Services (B domain -> A domain proxy):
- /arm_b/scene5_waste_classification/enter
- /arm_b/scene5_waste_classification/exit
- /arm_b/scene5_waste_classification/enable_transport
- /arm_b/scene5_waste_classification/set_slot_order
- /arm_b/scene5_waste_classification/set_place_targets
- /arm_b/scene5_waste_classification/set_fixed_pick
"""
from dataclasses import dataclass
import os
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, SetBool
from interfaces.srv import SetString, SetStringList


DEFAULT_A_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', 78))
DEFAULT_B_DOMAIN_ID = int(os.environ.get('SCENE5_B_DOMAIN_ID', DEFAULT_A_DOMAIN_ID + 1))

B_IMAGE_TOPIC = "/arm_b/waste_classification_motor_depth/result_image/compressed"
B_IMAGE_MSG_TYPE = CompressedImage
B_CONVEYOR_TOPIC = "/arm_b/ros_robot_controller/conveyor/set"

IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class BridgeConfig:
    arm_role: str
    a_domain_id: int
    b_domain_id: int
    run_domain_bridge: bool


def _domain_id(value, default):
    try:
        domain_id = int(str(value).strip())
    except Exception:
        return int(default)
    if domain_id < 0:
        return int(default)
    return domain_id


def normalize_role(role):
    role = str(role or "A").strip().upper()
    return role if role in ("A", "B") else "A"


def load_bridge_config(environ=None):
    environ = os.environ if environ is None else environ
    role = normalize_role(environ.get("SCENE5_ARM_ROLE", "A"))
    a_domain_id = _domain_id(
        environ.get(
            "SCENE5_BRIDGE_A_DOMAIN_ID",
            environ.get("ROS_DOMAIN_ID", DEFAULT_A_DOMAIN_ID),
        ),
        DEFAULT_A_DOMAIN_ID,
    )
    b_domain_id = _domain_id(
        environ.get(
            "SCENE5_BRIDGE_B_DOMAIN_ID",
            environ.get("SCENE5_B_DOMAIN_ID", DEFAULT_B_DOMAIN_ID),
        ),
        DEFAULT_B_DOMAIN_ID,
    )
    return BridgeConfig(
        arm_role=role,
        a_domain_id=a_domain_id,
        b_domain_id=b_domain_id,
        run_domain_bridge=(role == "A"),
    )


class Scene5DomainBridge:
    def __init__(self, config):
        self.config = config
        self.a_context = Context()
        self.b_context = Context()
        self.a_node = None
        self.b_node = None
        self.a_executor = None
        self.b_executor = None
        self.a_image_pub = None
        self.b_conveyor_pub = None
        self._executor_threads = []

    # B arm service names to be bridged from B domain to A domain
    B_SERVICE_PREFIX = '/arm_b/scene5_waste_classification'
    B_SERVICES = {
        'enter':             Trigger,
        'exit':              Trigger,
        'enable_transport':  SetBool,
        'set_slot_order':    SetStringList,
        'set_place_targets': SetString,
        'set_fixed_pick':    SetString,
    }

    def setup(self):
        if self.config.a_domain_id == self.config.b_domain_id:
            raise RuntimeError(
                "Scene5 bridge requires different ROS_DOMAIN_ID values, "
                f"got {self.config.a_domain_id}"
            )

        rclpy.init(context=self.a_context, domain_id=self.config.a_domain_id)
        rclpy.init(context=self.b_context, domain_id=self.config.b_domain_id)

        self.a_node = Node("scene5_domain_bridge_a", context=self.a_context)
        self.b_node = Node("scene5_domain_bridge_b", context=self.b_context)

        self.a_image_pub = self.a_node.create_publisher(
            B_IMAGE_MSG_TYPE,
            B_IMAGE_TOPIC,
            IMAGE_QOS,
        )
        self.b_node.create_subscription(
            B_IMAGE_MSG_TYPE,
            B_IMAGE_TOPIC,
            self._on_b_image,
            IMAGE_QOS,
        )

        self.b_conveyor_pub = self.b_node.create_publisher(
            Int8,
            B_CONVEYOR_TOPIC,
            1,
        )
        self.a_node.create_subscription(
            Int8,
            B_CONVEYOR_TOPIC,
            self._on_a_conveyor,
            1,
        )

        # Service bridge: proxy B-domain services into A domain
        self._b_srv_clients = {}
        for name, srv_type in self.B_SERVICES.items():
            full_name = f'{self.B_SERVICE_PREFIX}/{name}'
            client = self.b_node.create_client(srv_type, full_name)
            self._b_srv_clients[name] = (client, srv_type)
            self.a_node.create_service(
                srv_type,
                full_name,
                self._make_proxy(name),
            )
        self.a_node.get_logger().info(
            f'Service bridge ready: {list(self.B_SERVICES.keys())} '
            f'proxied from domain {self.config.b_domain_id} to {self.config.a_domain_id}'
        )

        self.a_executor = MultiThreadedExecutor(
            num_threads=4,
            context=self.a_context,
        )
        self.b_executor = MultiThreadedExecutor(
            num_threads=4,
            context=self.b_context,
        )
        self.a_executor.add_node(self.a_node)
        self.b_executor.add_node(self.b_node)

    def _make_proxy(self, name):
        """Return a service callback that forwards calls to B domain."""
        def _callback(request, response):
            return self._proxy_call(name, request, response)
        return _callback

    def _proxy_call(self, name, request, response, timeout_sec=10.0):
        """Forward a service call from A domain to B domain and return result."""
        client, srv_type = self._b_srv_clients[name]
        svc_name = f'{self.B_SERVICE_PREFIX}/{name}'
        if not client.wait_for_service(timeout_sec=3.0):
            self.a_node.get_logger().warn(
                f'[bridge] B service unavailable: {svc_name}'
            )
            if hasattr(response, 'success'):
                response.success = False
            if hasattr(response, 'message'):
                response.message = f'B service unavailable (bridge): {svc_name}'
            return response
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if future.done() and future.result() is not None:
            return future.result()
        self.a_node.get_logger().warn(f'[bridge] B service timeout: {svc_name}')
        if hasattr(response, 'success'):
            response.success = False
        if hasattr(response, 'message'):
            response.message = f'B service timeout (bridge): {svc_name}'
        return response

    def _on_b_image(self, msg):
        self.a_image_pub.publish(msg)

    def _on_a_conveyor(self, msg):
        bridged = Int8()
        bridged.data = int(msg.data)
        self.b_conveyor_pub.publish(bridged)

    def start(self):
        self.setup()
        self.a_node.get_logger().info(
            "Scene5 ROS topic bridge "
            f"A:{self.config.a_domain_id} <-image- "
            f"B:{self.config.b_domain_id}; conveyor A->B started"
        )
        for executor in (self.a_executor, self.b_executor):
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            self._executor_threads.append(thread)

    def shutdown(self):
        for executor in (self.a_executor, self.b_executor):
            if executor is not None:
                executor.shutdown()
        for node in (self.a_node, self.b_node):
            if node is not None:
                node.destroy_node()
        for context in (self.a_context, self.b_context):
            if context.ok():
                context.shutdown()

    def spin_forever(self):
        self.start()
        try:
            while self.a_context.ok() and self.b_context.ok():
                time.sleep(0.2)
        finally:
            self.shutdown()


def idle_on_b_role(config):
    print(
        "Scene5 ROS topic bridge runs on A only; "
        f"B role idle on ROS_DOMAIN_ID={config.b_domain_id}",
        flush=True,
    )
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        return


def main():
    config = load_bridge_config()
    if not config.run_domain_bridge:
        idle_on_b_role(config)
        return

    bridge = Scene5DomainBridge(config)
    try:
        bridge.spin_forever()
    except KeyboardInterrupt:
        bridge.shutdown()


if __name__ == "__main__":
    main()
