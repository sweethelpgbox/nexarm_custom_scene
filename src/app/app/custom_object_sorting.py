#!/usr/bin/env python3
# coding: utf-8
"""custom_object_sorting — scene_6 detection + pick-and-place behavior.

Detects a single YOLO object class, "strawberry shortcake ice cream bar"
(swap in a real trained model, see
``src/example/example/yolo_detect/models/``) and moves every instance it
sees to one fixed place target defined in
``src/app/config/plays/scene6_custom_object_sorting.yaml``.

Modeled on the working scene_1/scene_3 pattern in ``app/waste_classification.py``:
same camera calibration files (``src/app/config/transform.yaml``,
``calibration.yaml``), and the same ``example.yolo_detect.yolo_node``
detector (see ``custom_object_sorting.launch.py``), just given its own
node name/services (``DETECT_NODE_NAME``) so it doesn't collide with
waste_classification's always-on ``yolo`` instance. Reuses the shared
pick()/place() motion helpers in ``app.utils.pick_and_place``. Trimmed to
a single class and a single destination, so it skips the multi-target
coordinator/heartbeat machinery the bigger multi-class nodes need.

Pick height: measured live per-detection from the registered depth point
cloud (``_sample_real_height``), not a fixed guess. See that method's
docstring for the math and its safety fallback to ``OBJECT_HEIGHT_M`` --
CONFIRMED ON HARDWARE ONLY UP TO the classification/pick/place pipeline
working with a fixed height; the live-depth path below has not itself
been validated against real hardware and its first live runs should be
watched closely (logged heights compared to the object's actual height)
before being trusted.
"""

import os
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo, PointCloud2
from interfaces.msg import ObjectsInfo
from ros_robot_controller_msgs.msg import ArmCoords

from sdk import common
from app import calibrated_pose, scene4_runtime, scene_play_registry
from app.utils import pick_and_place

try:
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_POINT_CLOUD2 = True
except ImportError:
    pc2 = None
    HAS_POINT_CLOUD2 = False

SCENE_ID = 'scene_6'
DETECT_CLASS = 'strawberry shortcake ice cream bar'
# Must match custom_object_sorting.launch.py's yolo_node `name=` and
# start_service/stop_service -- the shared yolo_node executable defaults to
# node name 'yolo' and services '/yolo/start'|'/yolo/stop', which collide
# with waste_classification.launch.py's always-on yolo_node instance.
DETECT_NODE_NAME = 'strawberry_shortcake_detect'
DEPTH_CLOUD_TOPIC = 'depth_cam/depth_registered/points'
# Fallback used when live depth sampling is unavailable or looks invalid.
OBJECT_HEIGHT_M = 0.03
# Sanity bounds on the live-measured height -- protects against a bad
# depth reading (noise, a hole in the depth image, a units mistake)
# commanding a wildly wrong -- and potentially unsafe -- arm position.
# Widen only after confirming real measured heights land inside this
# range in the logs.
MIN_SAFE_HEIGHT_M = 0.0
MAX_SAFE_HEIGHT_M = 0.08
# Half-width (in pixels) of the window sampled around the detection's
# center pixel; the median of valid points in this window is used, to be
# robust to individual noisy/missing depth pixels.
DEPTH_SAMPLE_WINDOW = 2
PICK_PITCH_DEG = 80.0
PICK_GRIPPER_ANGLE = 500
PICK_GRIPPER_DEPTH_M = 0.02
DEFAULT_PLACE_TARGET = [0.15, 0.15, 0.02]
MIN_DETECT_SCORE = 0.5


class CustomObjectSortingNode(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.enter = False
        self.busy = False
        self.lock = threading.RLock()

        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.scene_config_path = scene4_runtime.scene_config_path()
        self.config_path = os.path.dirname(self.scene_config_path) + "/"
        self.play_config_path = self.get_string_param('play_config_path', '')

        self.intrinsic = None
        self.distortion = None
        # Raw (unshifted) extrinsic tvec/rmat from calibration -- the
        # per-detection code shifts this dynamically using the live
        # measured object height (see _shifted_extristric), rather than
        # baking in a single fixed-height shift once at entry time.
        self.raw_tvec = None
        self.raw_rmat = None
        self.white_area_center = None
        self.calibration_ready = False

        self.depth_cloud = None
        self.depth_cloud_lock = threading.Lock()
        if not HAS_POINT_CLOUD2:
            self.get_logger().warn(
                'sensor_msgs_py.point_cloud2 not importable -- live depth '
                'sampling disabled, will always fall back to OBJECT_HEIGHT_M')

        self.place_target = self._load_place_target()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.start_yolo_client = self.create_client(
            Trigger, f'/{DETECT_NODE_NAME}/start', callback_group=self.timer_cb_group)
        self.stop_yolo_client = self.create_client(
            Trigger, f'/{DETECT_NODE_NAME}/stop', callback_group=self.timer_cb_group)
        self.controller_init_client = self.create_client(
            Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)

        self.camera_info_sub = None
        self.object_sub = None
        self.depth_cloud_sub = None

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_string_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is not None and value != '':
                return str(value)
        except Exception:
            pass
        return str(default)

    def send_request(self, client, msg, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'service {client.srv_name} unavailable')
            return None
        future = client.call_async(msg)
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        return future.result()

    # -- lifecycle -----------------------------------------------------

    def _load_place_target(self):
        try:
            cfg = scene_play_registry.load_play_config(SCENE_ID, self.play_config_path or None)
            target = (cfg.get('place_targets') or {}).get(DETECT_CLASS)
            if target and len(target) == 3:
                return [float(v) for v in target]
        except Exception as e:
            self.get_logger().warn(f'failed to load {SCENE_ID} place config, using default: {e}')
        return list(DEFAULT_PLACE_TARGET)

    def init_process(self):
        self.timer.cancel()
        self.get_logger().info('waiting for arm controller...')
        self.controller_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.05)
        home = pick_and_place.load_scene_home_pose()
        pick_and_place.publish_arm(
            self.arm_pub, home['x'], home['y'], home['z'],
            home['pitch'], home['roll'], home.get('claw', 0.0),
            int(home.get('time_ms', 2000)))
        time.sleep(max(1.0, home.get('time_ms', 2000) / 1000.0))
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        if bool(self.get_parameter('start').value):
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
        self.get_logger().info('\033[1;32m%s\033[0m' % f'{SCENE_ID} (custom_object_sorting) ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def enter_srv_callback(self, request, response):
        if not self.enter:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, 'depth_cam/rgb/camera_info', self.camera_info_callback, 1)
            self.object_sub = self.create_subscription(
                ObjectsInfo, f'/{DETECT_NODE_NAME}/object_detect', self.object_callback, 1)
            if HAS_POINT_CLOUD2:
                self.depth_cloud_sub = self.create_subscription(
                    PointCloud2, DEPTH_CLOUD_TOPIC, self.depth_cloud_callback, 1)
            self.enter = True
            threading.Thread(target=self.get_roi, daemon=True).start()
        self.send_request(self.start_yolo_client, Trigger.Request())
        response.success = True
        response.message = 'entered scene_6'
        return response

    def exit_srv_callback(self, request, response):
        self.send_request(self.stop_yolo_client, Trigger.Request())
        if self.object_sub is not None:
            self.destroy_subscription(self.object_sub)
            self.object_sub = None
        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None
        if self.depth_cloud_sub is not None:
            self.destroy_subscription(self.depth_cloud_sub)
            self.depth_cloud_sub = None
        self.enter = False
        response.success = True
        response.message = 'exited scene_6'
        return response

    # -- calibration -----------------------------------------------------

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def depth_cloud_callback(self, msg):
        with self.depth_cloud_lock:
            self.depth_cloud = msg

    def get_roi(self):
        config = common.get_yaml_data(os.path.join(self.config_path, self.config_file)) or {}
        extristric = np.array(config.get('extristric', []))
        white_area_center = np.array(config.get('white_area_pose_world', []))
        if extristric.size == 0 or white_area_center.size == 0:
            self.get_logger().error(
                f'{self.config_path}{self.config_file} is missing extristric/white_area_pose_world; '
                'run the calibration GUI (software/calibration) before using scene_6.')
            return
        self.white_area_center = white_area_center
        while self.intrinsic is None or self.distortion is None:
            time.sleep(0.1)
        self.raw_tvec = np.array(extristric[:1]).reshape((3, 1))
        self.raw_rmat = np.array(extristric[1:])
        self.calibration_ready = True

    def _shifted_extristric(self, height):
        """Same trick get_roi() used to do once with a fixed height --
        shift the calibrated plane's tvec by `height` along its own Z
        axis (see sdk.common.extristric_plane_shift) so pixel_to_world's
        plane-intersection math targets a plane `height` above the
        calibrated table, rather than the table itself. Done fresh per
        detection now, using whatever height _sample_real_height (or the
        OBJECT_HEIGHT_M fallback) provides for that specific object."""
        return common.extristric_plane_shift(self.raw_tvec, self.raw_rmat, height)

    def _sample_real_height(self, pixel_center):
        """Best-effort real object height above the calibrated table
        plane, measured from the registered depth point cloud rather than
        assumed. Returns None (caller falls back to OBJECT_HEIGHT_M) if
        depth data is unavailable, the cloud isn't organized, no valid
        (non-NaN) points are found near the pixel, or the computed height
        falls outside [MIN_SAFE_HEIGHT_M, MAX_SAFE_HEIGHT_M].

        Math: back-project the sampled camera-frame 3D point through the
        *raw* (unshifted) calibration extrinsic the same way
        sdk.common.pixels_to_world back-projects a pixel ray, except
        using the point's real measured depth instead of solving for an
        assumed plane intersection -- world_point = invR @ (cam_point -
        tvec); world_point's Z component is the real height above the
        calibrated table.

        This has not been validated against real hardware -- the logged
        cam_point/height on each pick should be sanity-checked against
        the object's real height before trusting it.
        """
        if not HAS_POINT_CLOUD2 or not self.calibration_ready:
            return None
        with self.depth_cloud_lock:
            cloud = self.depth_cloud
        if cloud is None or cloud.height <= 1:
            return None
        try:
            u = max(0, min(cloud.width - 1, int(round(pixel_center[0]))))
            v = max(0, min(cloud.height - 1, int(round(pixel_center[1]))))
            uvs = [
                (uu, vv)
                for dv in range(-DEPTH_SAMPLE_WINDOW, DEPTH_SAMPLE_WINDOW + 1)
                for du in range(-DEPTH_SAMPLE_WINDOW, DEPTH_SAMPLE_WINDOW + 1)
                for uu, vv in [(u + du, v + dv)]
                if 0 <= uu < cloud.width and 0 <= vv < cloud.height
            ]
            points = list(pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True, uvs=uvs))
            if not points:
                return None
            cam_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
            cam_point = np.median(cam_points, axis=0).reshape(3, 1)

            invR = np.linalg.inv(self.raw_rmat)
            world_point = invR @ (cam_point - self.raw_tvec)
            real_height = float(world_point[2][0])

            self.get_logger().info(
                f'depth sample at ({u},{v}) from {len(points)} pts: '
                f'cam_point={cam_point.reshape(-1).round(4).tolist()}, '
                f'computed height={real_height:.4f} m')

            if not (MIN_SAFE_HEIGHT_M <= real_height <= MAX_SAFE_HEIGHT_M):
                self.get_logger().warn(
                    f'computed height {real_height:.4f} m outside safe range '
                    f'[{MIN_SAFE_HEIGHT_M}, {MAX_SAFE_HEIGHT_M}] -- falling back '
                    f'to OBJECT_HEIGHT_M={OBJECT_HEIGHT_M}')
                return None
            return real_height
        except Exception as e:
            self.get_logger().warn(f'depth sampling failed, falling back to OBJECT_HEIGHT_M: {e}')
            return None

    def get_object_world_position(self, pixel, extristric, height):
        calibration = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
        return calibrated_pose.pixel_to_calibrated_world(
            pixel, self.intrinsic, extristric, self.white_area_center, calibration, height=height)

    @staticmethod
    def _position_yaw(position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw += 180
        elif position[0] < 0 and position[1] > 0:
            yaw -= 180
        return yaw

    # -- detection -> pick/place -----------------------------------------------------

    def object_callback(self, msg):
        if self.busy or not self.calibration_ready:
            return
        best = None
        for obj in msg.objects:
            if obj.class_name != DETECT_CLASS or obj.score < MIN_DETECT_SCORE:
                continue
            if best is None or obj.score > best.score:
                best = obj
        if best is None:
            return
        x1, y1, x2, y2 = best.box
        center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        with self.lock:
            if self.busy:
                return
            self.busy = True
        threading.Thread(target=self._pick_and_place, args=(center,), daemon=True).start()

    def _pick_and_place(self, pixel_center):
        try:
            self.send_request(self.stop_yolo_client, Trigger.Request())

            real_height = self._sample_real_height(pixel_center)
            height = real_height if real_height is not None else OBJECT_HEIGHT_M
            extristric = self._shifted_extristric(height)

            position, _ = self.get_object_world_position(pixel_center, extristric, height)
            position = np.asarray(position, dtype=np.float64).tolist()
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file)) or {}
            position = calibrated_pose.apply_axis_calibration(position, calibration, 'kinematics').tolist()
            yaw = self._position_yaw(position)

            self.get_logger().info(
                f'picking {DETECT_CLASS}: height={height:.4f} m '
                f'({"measured" if real_height is not None else "OBJECT_HEIGHT_M fallback"}), '
                f'position={position}')

            picked = pick_and_place.pick(
                position, PICK_PITCH_DEG, yaw, PICK_GRIPPER_ANGLE, PICK_GRIPPER_DEPTH_M, self.arm_pub)
            if picked:
                place_yaw = self._position_yaw(self.place_target)
                pick_and_place.place(
                    self.place_target, PICK_PITCH_DEG, place_yaw, PICK_GRIPPER_ANGLE, self.arm_pub)
                self.get_logger().info(f'placed {DETECT_CLASS} at {self.place_target}')
            else:
                self.get_logger().warn(f'pick failed for {DETECT_CLASS} at {position}')
        except Exception as e:
            self.get_logger().error(f'pick/place error: {e}')
        finally:
            with self.lock:
                self.busy = False
            if self.enter:
                self.send_request(self.start_yolo_client, Trigger.Request())


def main():
    rclpy.init()
    node = CustomObjectSortingNode('custom_object_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
