#!/usr/bin/env python3
# encoding: utf-8
# 前视颜色追踪抓取（新底层 ArmCoords + ArmFullState 实时反馈）
import os
import cv2
import math
import time
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from app.utils import utils
from std_srvs.srv import Trigger
from interfaces.srv import SetString
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from example.scene_pose import load_scene_home_pose
from ros_robot_controller_msgs.srv import GetArmFullState
from rclpy.callback_groups import ReentrantCallbackGroup
from app import calibrated_pose


def decode_color_image(ros_image):
    height = int(ros_image.height)
    width = int(ros_image.width)
    encoding = str(getattr(ros_image, 'encoding', '') or '').lower()
    step = int(getattr(ros_image, 'step', 0) or 0)
    if step > 0 and width > 0:
        channels = max(1, step // width)
    else:
        channels = 4 if ('rgba' in encoding or 'bgra' in encoding) else 3

    shape = (height, width) if channels == 1 else (height, width, channels)
    raw_image = np.ndarray(shape=shape, dtype=np.uint8, buffer=ros_image.data)
    raw_image = np.copy(raw_image)

    if channels == 1:
        rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_GRAY2RGB)
    elif channels == 4:
        if encoding == 'rgba8':
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_RGBA2RGB)
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGRA2RGB)
    else:
        if encoding == 'rgb8':
            rgb_image = raw_image
        else:
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)

    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    return rgb_image, bgr_image


class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color

    def proc(self, source_rgb_image, result_bgr_image, color_ranges):
        h, w = source_rgb_image.shape[:2]
        color = color_ranges['color_range_list'][self.target_color]

        img = cv2.resize(source_rgb_image, (int(w / 2), int(h / 2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB)
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max']))

        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c)) < 50:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c)
            if min_c is None or center_x < min_c[1]:
                min_c = (c, center_x)

        if min_c is None:
            return result_bgr_image, None, 0.0

        (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0])
        center_x *= 2.0
        center_y *= 2.0
        radius *= 2.0

        circle_color = common.range_rgb[self.target_color] if self.target_color in common.range_rgb else (0x55, 0x55, 0x55)
        circle_color = (int(circle_color[2]), int(circle_color[1]), int(circle_color[0]))
        cv2.circle(result_bgr_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)
        return result_bgr_image, (float(center_x), float(center_y)), float(radius)


class TrackAndGrabNode(Node):
    INIT_HOME = load_scene_home_pose()
    INIT_X = INIT_HOME['x']
    INIT_Y = INIT_HOME['y']
    INIT_Z = INIT_HOME['z']
    INIT_PITCH = INIT_HOME['pitch']
    INIT_ROLL = INIT_HOME['roll']
    INIT_CLAW = INIT_HOME['claw']
    GRAB_CLAW = -25.0
    OPEN_CLAW = -82.5
    STABLE_DISTANCE_M = 0.004
    STABLE_FRAMES = 24
    MIN_TRACK_SECONDS = 1.5

    PLACE_X = 100.0
    PLACE_Y = -150.0
    PLACE_Z = 80.0
    PLACE_LIFT_Z = 150.0
    PLACE_PITCH = -60.0

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.running = True
        self.start = False
        self.moving = False
        self.tracker = None
        self.target_color = None
        self.display = self.get_bool_param('display', True)
        self.stable_count = 0
        self.last_world_position = None
        self.last_pick_stamp = time.time()
        self.start_stamp = time.time() + 1.0
        self.first_seen_stamp = None
        self.endpoint = None
        self.current_pose = None
        self.known_pose = {
            'x': self.INIT_X,
            'y': self.INIT_Y,
            'z': self.INIT_Z,
            'pitch': self.INIT_PITCH,
            'roll': self.INIT_ROLL,
            'claw': self.INIT_CLAW,
            'yaw': 0.0,
        }
        self.config_path = '/home/ubuntu/ros2_ws/src/app/config/'
        self.calibration_file = 'calibration.yaml'
        self.transform_file = 'transform.yaml'
        self.camera_info_path = '/home/ubuntu/ros2_ws/src/peripherals/config/camera_info.yaml'
        self.hand2cam_tf_matrix = None
        self.plane = None
        self.depth_offset = (0.0, 0.0, 0.0)
        self.depth_scale = (1.0, 1.0, 1.0)
        self.kinematics_offset = (0.0, 0.0, 0.0)
        self.kinematics_scale = (1.0, 1.0, 1.0)
        self.image_queue = queue.Queue(maxsize=2)

        signal.signal(signal.SIGINT, self.shutdown)

        self.data = common.get_yaml_data('/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml')
        self.lab_data = self.data['/**']['ros__parameters']

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback)

        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.2)
        sync.registerCallback(self.multi_callback)

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return bool(value)
        except Exception:
            return default

    def should_display(self):
        return self.display

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum=None, frame=None):
        self.running = False

    def arm_state_callback(self, msg):
        self.current_pose = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'pitch': float(msg.pitch),
            'roll': float(msg.roll),
            'claw': float(msg.claw),
            'yaw': float(msg.yaw),
        }

    def request_real_pose_snapshot(self, timeout_sec=0.3):
        if not self.arm_state_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
            return None
        future = self.arm_state_client.call_async(GetArmFullState.Request())
        end_time = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end_time:
            if future.done():
                try:
                    result = future.result()
                except Exception:
                    return None
                if result is None or not result.success:
                    return None
                return {
                    'x': float(result.x),
                    'y': float(result.y),
                    'z': float(result.z),
                    'pitch': float(result.pitch),
                    'roll': float(result.roll),
                    'claw': float(result.claw),
                    'yaw': float(result.yaw),
                }
            time.sleep(0.01)
        return None

    def get_pose_snapshot(self):
        return self.current_pose or self.request_real_pose_snapshot() or self.known_pose

    def wait_for_motion_ready(self):
        self.get_logger().info('等待底层控制初始化...')
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            self.get_logger().info('等待机械臂坐标控制订阅...')
            time.sleep(0.2)

    def load_calibration_parameters(self):
        try:
            camera_info = common.get_yaml_data(self.camera_info_path)
            matrix = camera_info.get('hand2cam_tf_matrix')
            if matrix is not None:
                self.hand2cam_tf_matrix = np.array(matrix, dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f'加载 hand2cam_tf_matrix 失败: {exc}')

        try:
            transform = common.get_yaml_data(os.path.join(self.config_path, self.transform_file))
            plane = transform.get('plane')
            if plane is not None:
                self.plane = tuple(float(v) for v in plane)
        except Exception as exc:
            self.get_logger().warn(f'加载 plane 失败: {exc}')

        try:
            calibration = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
            self.depth_offset = tuple(float(v) for v in calibration['depth']['offset'])
            self.depth_scale = tuple(float(v) for v in calibration['depth']['scale'])
            self.kinematics_offset = tuple(float(v) for v in calibration['kinematics']['offset'])
            self.kinematics_scale = tuple(float(v) for v in calibration['kinematics']['scale'])
        except Exception as exc:
            self.get_logger().warn(f'加载 calibration.yaml 失败: {exc}')

    def apply_depth_calibration(self, position):
        calibration = {
            'depth': {
                'offset': self.depth_offset,
                'scale': self.depth_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'depth').tolist()

    def apply_kinematics_calibration(self, position):
        calibration = {
            'kinematics': {
                'offset': self.kinematics_offset,
                'scale': self.kinematics_scale,
            }
        }
        return calibrated_pose.apply_axis_calibration(position, calibration, 'kinematics').tolist()

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        self.load_calibration_parameters()
        self.go_home(wait_time=1.5)
        if self.get_bool_param('start', False):
            self.target_color = str(self.get_parameter('color').value)
            req = SetString.Request()
            req.data = self.target_color
            self.set_color_srv_callback(req, SetString.Response())
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'track_and_grab ready')

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)
        self.known_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'pitch': float(pitch),
            'roll': float(roll),
            'claw': float(claw),
            'yaw': float(self.known_pose.get('yaw', 0.0)),
        }

    def go_home(self, wait_time=1.0):
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.OPEN_CLAW, 1500)
        time.sleep(wait_time)
        self.get_endpoint_matrix()

    def get_endpoint_matrix(self):
        p = self.get_pose_snapshot()
        if p is None:
            p = self.known_pose
        x_m = float(p['x']) / 1000.0
        y_m = float(p['y']) / 1000.0
        z_m = float(p['z']) / 1000.0
        yaw_deg = float(p.get('yaw', math.degrees(math.atan2(y_m, x_m if abs(x_m) > 1e-6 else 1e-6))))
        pitch_deg = float(p['pitch'])
        roll_deg = float(p['roll'])

        yaw = math.radians(yaw_deg)
        pitch_rad = math.radians(-pitch_deg)
        roll_rad = math.radians(roll_deg)

        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        rz_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        ry_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
        rx_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)

        endpoint = np.eye(4, dtype=np.float64)
        endpoint[:3, :3] = rz_yaw @ ry_pitch @ rx_roll
        endpoint[:3, 3] = [x_m, y_m, z_m]
        self.endpoint = endpoint
        return endpoint

    def get_endpoint(self):
        return self.get_endpoint_matrix()

    def set_color_srv_callback(self, request, response):
        self.target_color = request.data
        self.tracker = ColorTracker(self.target_color)
        self.start = True
        self.stable_count = 0
        self.last_world_position = None
        self.first_seen_stamp = None
        self.start_stamp = time.time() + self.MIN_TRACK_SECONDS
        response.success = True
        response.message = 'set_color'
        self.get_logger().info('\033[1;32mset color: %s\033[0m' % self.target_color)
        return response

    def start_srv_callback(self, request, response):
        self.start = True
        self.stable_count = 0
        self.last_world_position = None
        self.first_seen_stamp = None
        self.start_stamp = time.time() + self.MIN_TRACK_SECONDS
        response.success = True
        response.message = 'start'
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        self.moving = False
        self.stable_count = 0
        self.last_world_position = None
        self.first_seen_stamp = None
        self.go_home(wait_time=1.0)
        response.success = True
        response.message = 'stop'
        return response

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except Exception:
                pass
        rgb_image, _ = decode_color_image(ros_rgb_image)
        depth_image = np.ndarray(
            shape=(ros_depth_image.height, ros_depth_image.width),
            dtype=np.uint16,
            buffer=ros_depth_image.data,
        )
        self.image_queue.put((np.copy(rgb_image), np.copy(depth_image), depth_camera_info))

    def pick(self, position):
        try:
            position = list(position)
            if position[0] > 0.22:
                position[2] += 0.01
            position = self.apply_kinematics_calibration(position)

            x_mm = float(position[0]) * 1000.0
            y_mm = float(position[1]) * 1000.0
            z_mm = float(position[2]) * 1000.0
            pitch = self.INIT_PITCH

            self.publish_arm(x_mm, y_mm, max(z_mm + 50.0, 150.0), pitch, 0.0, self.OPEN_CLAW, 1800)
            time.sleep(1.2)

            self.publish_arm(x_mm, y_mm, z_mm, pitch, 0.0, self.OPEN_CLAW, 1500)
            time.sleep(1.5)

            z_down = z_mm - 18.0
            self.publish_arm(x_mm, y_mm, z_down, pitch, 0.0, self.OPEN_CLAW, 1200)
            time.sleep(1.2)

            time.sleep(1.0)
            self.publish_arm(x_mm, y_mm, z_down, pitch, 0.0, self.GRAB_CLAW, 700)
            time.sleep(0.8)

            self.publish_arm(x_mm, y_mm, z_mm + 40.0, pitch, 0.0, self.GRAB_CLAW, 1800)
            time.sleep(1.8)

            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_LIFT_Z, self.PLACE_PITCH, 0.0, self.GRAB_CLAW, 1600)
            time.sleep(1.6)
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_Z, self.PLACE_PITCH, 0.0, self.GRAB_CLAW, 1200)
            time.sleep(1.2)
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_Z, self.PLACE_PITCH, 0.0, self.OPEN_CLAW, 600)
            time.sleep(0.8)
            self.publish_arm(self.PLACE_X, self.PLACE_Y, self.PLACE_LIFT_Z, self.PLACE_PITCH, 0.0, self.OPEN_CLAW, 1500)
            time.sleep(1.5)

            self.go_home(wait_time=1.5)
        finally:
            self.stable_count = 0
            self.last_world_position = None
            self.first_seen_stamp = None
            self.last_pick_stamp = time.time()
            self.moving = False

    def main(self):
        while self.running:
            try:
                rgb_image, depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                result_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                h, w = depth_image.shape[:2]
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000.0 * 255.0
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                if self.tracker is not None and not self.moving and self.start and time.time() > self.start_stamp:
                    result_bgr, center, radius = self.tracker.proc(rgb_image, result_bgr, self.lab_data)
                    if center is not None:
                        if self.first_seen_stamp is None:
                            self.first_seen_stamp = time.time()
                        center_x = int(np.clip(center[0], 0, w - 1))
                        center_y = int(np.clip(center[1], 0, h - 1))
                        roi = [center_y - 5, center_y + 5, center_x - 5, center_x + 5]
                        roi[0] = max(0, roi[0])
                        roi[1] = min(h, roi[1])
                        roi[2] = max(0, roi[2])
                        roi[3] = min(w, roi[3])
                        roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
                        valid_depths = roi_distance[np.logical_and(roi_distance > 0, roi_distance < 10000)]

                        if valid_depths.size > 0 and self.hand2cam_tf_matrix is not None and self.plane is not None:
                            depth_value = float(np.median(valid_depths))
                            endpoint = self.get_endpoint()
                            pose_t = utils.calculate_world_position(
                                center_x,
                                center_y,
                                depth_value,
                                self.plane,
                                endpoint,
                                self.hand2cam_tf_matrix,
                                depth_camera_info.k,
                            )
                            pose_t = self.apply_depth_calibration(list(pose_t))

                            txt = 'Dist: {}mm'.format(int(np.median(valid_depths)))
                            position_text = f'x:{pose_t[0]:.3f}m y:{pose_t[1]:.3f}m z:{pose_t[2]:.3f}m'
                            cv2.circle(result_bgr, (center_x, center_y), 5, (255, 255, 255), -1)
                            cv2.circle(depth_color_map, (center_x, center_y), 5, (255, 255, 255), -1)
                            cv2.putText(depth_color_map, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 8, cv2.LINE_AA)
                            cv2.putText(depth_color_map, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                            cv2.putText(result_bgr, position_text, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 6, cv2.LINE_AA)
                            cv2.putText(result_bgr, position_text, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2, cv2.LINE_AA)

                            if self.last_world_position is not None:
                                delta = math.sqrt(
                                    (self.last_world_position[0] - pose_t[0]) ** 2 +
                                    (self.last_world_position[1] - pose_t[1]) ** 2 +
                                    (self.last_world_position[2] - pose_t[2]) ** 2
                                )
                                if delta < self.STABLE_DISTANCE_M:
                                    self.stable_count += 1
                                else:
                                    self.stable_count = 0
                            self.last_world_position = pose_t

                            tracked_long_enough = (
                                self.first_seen_stamp is not None
                                and time.time() - self.first_seen_stamp >= self.MIN_TRACK_SECONDS
                                and time.time() >= self.start_stamp
                            )
                            if self.stable_count >= self.STABLE_FRAMES and tracked_long_enough and time.time() - self.last_pick_stamp > 1.0:
                                self.stable_count = 0
                                self.first_seen_stamp = None
                                self.moving = True
                                self.get_logger().info(
                                    f'稳定追踪后夹取: stable={self.STABLE_FRAMES}, '
                                    f'pos=({pose_t[0]:.3f},{pose_t[1]:.3f},{pose_t[2]:.3f})'
                                )
                                threading.Thread(target=self.pick, args=(list(pose_t),), daemon=True).start()
                        else:
                            self.stable_count = 0
                            self.last_world_position = None
                            self.first_seen_stamp = None
                    else:
                        self.stable_count = 0
                        self.last_world_position = None
                        self.first_seen_stamp = None

                self.fps.update()
                result_image = np.concatenate([self.fps.show_fps(result_bgr), depth_color_map], axis=1)
                if self.should_display():
                    cv2.imshow('depth', result_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        self.running = False
                    elif key == ord('s'):
                        self.start = True
                        self.stable_count = 0
                        self.last_world_position = None
                        self.first_seen_stamp = None
                        self.start_stamp = time.time() + self.MIN_TRACK_SECONDS
                    elif key == ord('a'):
                        self.start = False
                        self.moving = False
            except Exception as e:
                self.get_logger().info('error: ' + str(e))

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrackAndGrabNode('track_and_grab')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
