#!/usr/bin/python3
# coding=utf8
# 通过深度图识别物体进行分类
# 机械臂向下识别，可识别长方体、球、圆柱体，并按类别分拣
import cv2
import math
import time
import queue
import signal
import threading
import numpy as np
import rclpy
import message_filters
from rclpy.node import Node
from sdk import common, fps
from interfaces.srv import SetStringList
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords, BuzzerState, ArmFullState
from example.scene_pose import load_scene_home_pose
from ros_robot_controller_msgs.srv import GetArmFullState
from rclpy.callback_groups import ReentrantCallbackGroup
from example.rgbd_function.include.grasp_height import side_grasp_z_from_top
from example.rgbd_function.include.position_change_detect import position_reorder

import traceback


def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z], dtype=np.float64)


def normalize_gripper_roll_deg(angle_deg: float, limit_deg: float = 120.0) -> float:
    normalized = ((float(angle_deg) + 90.0) % 180.0) - 90.0
    return float(np.clip(normalized, -limit_deg, limit_deg))


def fold_gripper_roll_deg(angle_deg: float) -> float:
    roll = normalize_gripper_roll_deg(angle_deg)
    if roll > 45.0:
        roll -= 90.0
    elif roll < -45.0:
        roll += 90.0
    return normalize_gripper_roll_deg(roll, limit_deg=60.0)


def image_angle_to_arm_roll_deg(position, image_angle: float) -> float:
    base_yaw = 0.0
    if position is not None and (abs(position[0]) > 1e-6 or abs(position[1]) > 1e-6):
        base_yaw = math.degrees(math.atan2(position[1], position[0]))
    return fold_gripper_roll_deg(base_yaw + float(image_angle))


def get_long_edge_angle(rect) -> float:
    box = cv2.boxPoints(rect).astype(np.float32)
    longest_vec = None
    longest_len = -1.0
    for i in range(4):
        vec = box[(i + 1) % 4] - box[i]
        length = float(np.linalg.norm(vec))
        if length > longest_len:
            longest_len = length
            longest_vec = vec
    if longest_vec is None:
        return 0.0
    angle_deg = math.degrees(math.atan2(float(longest_vec[1]), float(longest_vec[0])))
    return float(((angle_deg + 90.0) % 180.0) - 90.0)


class ObjectClassificationNode(Node):
    hand2cam_tf_matrix = np.array([
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.01],
        [0.0, -1.0, 0.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    pick_offset = [-0.03, 0.0, 0.0, 0.0, -0.09]  # x1, x2, y1, y2, z
    
    INIT_HOME = load_scene_home_pose()
    INIT_X = INIT_HOME['x']
    INIT_Y = INIT_HOME['y']
    INIT_Z = INIT_HOME['z']
    INIT_PITCH = INIT_HOME['pitch']
    INIT_ROLL = INIT_HOME['roll']
    INIT_CLAW = INIT_HOME['claw']
    GRAB_CLAW = -45.0
    OPEN_CLAW = -82.5

    PLACE_MAP = {
        'sphere': (95.0, -214.0, 20.0),
        'cylinder': (-18.0, -214.0, 20.0),
        'cuboid': (-70.0, -214.0, 20.0),
    }

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.running = True
        self.start = self.get_bool_param('start', False)
        self.display = self.get_bool_param('display', True)
        self.shapes = None
        self.target_shapes = None
        self.roi = [60, 350, 160, 540]
        self.endpoint = None
        self.last_position = (0.0, 0.0)
        self.last_object_info_list = []
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
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.debug = self.get_bool_param('debug', False)

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)

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
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.2)

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        self.goto_default(wait_time=1.5)

        self.shapes = ['sphere', 'cuboid', 'cylinder']

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shape_recognition ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum=None, frame=None):
        self.running = False

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

    def get_endpoint_matrix(self):
        p = self.get_pose_snapshot()
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

    def set_shape_srv_callback(self, request, response):
        self.shapes = list(request.data)
        self.start = True
        response.success = True
        response.message = 'set_shape'
        return response

    def start_srv_callback(self, request, response):
        self.start = True
        response.success = True
        response.message = 'start'
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        self.shapes = None
        self.moving = False
        self.count = 0
        self.target_shapes = None
        self.last_position = (0.0, 0.0)
        self.last_object_info_list = []
        self.goto_default(wait_time=1.0)
        response.success = True
        response.message = 'stop'
        return response

    def goto_default(self, wait_time=1.0):
        self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, self.INIT_ROLL, self.OPEN_CLAW, 1500)
        time.sleep(wait_time)
        self.get_endpoint_matrix()

    def buzz(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def place_to_shape_bin(self, shape_name):
        px, py, pz = self.PLACE_MAP.get(shape_name, self.PLACE_MAP['cuboid'])
        self.publish_arm(px, py, 120.0, self.INIT_PITCH, 0.0, self.GRAB_CLAW, 1500)
        time.sleep(1.5)
        self.publish_arm(px, py, pz, self.INIT_PITCH, 0.0, self.GRAB_CLAW, 1200)
        time.sleep(1.2)
        self.publish_arm(px, py, pz, self.INIT_PITCH, 0.0, self.OPEN_CLAW, 600)
        time.sleep(0.8)
        self.publish_arm(px, py, 120.0, self.INIT_PITCH, 0.0, self.OPEN_CLAW, 1500)
        time.sleep(1.5)

    def move(self, object_info):
        shape, index, pose_t, depth, rect_info, color, yaw = object_info
        self.get_logger().info(f'pick {shape}{index}: {pose_t}, yaw={yaw:.2f}')
        self.buzz()
        time.sleep(0.6)

        if shape == 'sphere':
            offset_z = self.pick_offset[-1]
            grasp_roll = 0.0
        elif shape == 'cylinder':
            offset_z = 0.03 + self.pick_offset[-1]
            grasp_roll = 0.0
        else:
            offset_z = 0.02 + self.pick_offset[-1]

        pose_t = list(pose_t)
        pose_t[0] += self.pick_offset[0] if pose_t[0] > 0.21 else self.pick_offset[1]
        pose_t[1] += self.pick_offset[2] if pose_t[1] > 0 else self.pick_offset[3]
        pose_t[2] += offset_z
        if shape == 'cuboid':
            grasp_roll = image_angle_to_arm_roll_deg(pose_t, yaw)

        grasp_z_mm = side_grasp_z_from_top(pose_t[2]) * 1000.0
        x_mm = pose_t[0] * 1000.0
        y_mm = pose_t[1] * 1000.0
        z_mm = grasp_z_mm + 18.0
        pitch = self.INIT_PITCH

        try:
            self.publish_arm(x_mm, y_mm, max(z_mm + 50.0, 150.0), pitch, 0.0, self.OPEN_CLAW, 1800)
            time.sleep(1.2)
            self.publish_arm(x_mm, y_mm, z_mm, pitch, grasp_roll, self.OPEN_CLAW, 1500)
            time.sleep(1.5)
            z_down = z_mm - 18.0
            self.publish_arm(x_mm, y_mm, z_down, pitch, grasp_roll, self.OPEN_CLAW, 1200)
            time.sleep(1.2)
            time.sleep(1.0)
            self.publish_arm(x_mm, y_mm, z_down, pitch, grasp_roll, self.GRAB_CLAW, 700)
            time.sleep(0.8)
            self.publish_arm(x_mm, y_mm, z_mm + 40.0, pitch, grasp_roll, self.GRAB_CLAW, 1800)
            time.sleep(1.8)
            self.publish_arm(self.INIT_X, self.INIT_Y, self.INIT_Z, self.INIT_PITCH, 0.0, self.GRAB_CLAW, 1500)
            time.sleep(1.5)
            self.place_to_shape_bin(shape)
            self.goto_default(wait_time=1.2)
        finally:
            self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            self.image_queue.get()
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        self.image_queue.put((np.copy(rgb_image), np.copy(depth_image), depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        endpoint = self.get_endpoint_matrix()
        position = depth_pixel_to_camera([x, y, depth / 1000.0], intrinsic_matrix)
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(endpoint, pose_end)
        pose_t, _ = common.mat_to_xyz_euler(world_pose)
        return pose_t

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        depth_image[:, :self.roi[2]] = np.array([[1000] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555
        min_index = int(np.argmin(depth))
        min_y = min_index // iw
        min_x = min_index - min_y * iw
        return depth_image[min_y, min_x]

    def get_contours(self, depth_image, min_dist):
        depth_image = np.where(depth_image > 300, 0, depth_image)
        depth_image = np.where(depth_image > min_dist + 20, 0, depth_image)
        sim_depth_image_sort = np.clip(depth_image, 0, max(min_dist - 10, 1)).astype(np.float64) / max(min_dist - 10, 1) * 255
        depth_gray = sim_depth_image_sort.astype(np.uint8)
        _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        if min_dist > 350:
            return object_info_list

        sphere_index = 0
        cuboid_index = 0
        cylinder_index = 0
        contours = self.get_contours(depth_image, min_dist)

        for obj in contours:
            area = cv2.contourArea(obj)
            if area < 300:
                continue

            perimeter = cv2.arcLength(obj, True)
            approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
            corner_num = len(approx)
            (cx, cy), _ = cv2.minEnclosingCircle(obj)
            rect = cv2.minAreaRect(obj)
            center, (width, height), rect_angle = rect
            long_edge_angle = get_long_edge_angle(rect)

            cx_i = int(np.clip(cx, 0, image_width - 1))
            cy_i = int(np.clip(cy, 0, image_height - 1))
            depth = depth_image[cy_i, cx_i]
            if depth <= 0:
                continue
            position = self.cal_position(cx, cy, depth, intrinsic_matrix)
            x, y, w, h = cv2.boundingRect(approx)

            mask = np.full((image_height, image_width), 0, dtype=np.uint8)
            cv2.drawContours(mask, [obj], -1, 255, cv2.FILLED)
            masked_depth = np.where(mask == 255, depth_image, np.nan)
            depth_std = float(np.nanstd(masked_depth)) if np.any(mask == 255) else 0.0

            obj_type = None
            yaw = 0.0
            if depth_std > 50.0 and corner_num > 4:
                sphere_index += 1
                obj_type = 'sphere'
                index = sphere_index
            elif depth_std < 35.0 and corner_num > 4:
                cylinder_index += 1
                obj_type = 'cylinder'
                index = cylinder_index
            else:
                cuboid_index += 1
                obj_type = 'cuboid'
                index = cuboid_index
                yaw = long_edge_angle

            object_info_list.append([
                obj_type,
                index,
                list(position),
                depth,
                [x, y, w, h, center, width, height, rect_angle],
                rgb_image[int(center[1]), int(center[0])],
                yaw,
            ])
            cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)
        return object_info_list

    def main(self):
        while self.running:
            try:
                rgb_image, depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                min_dist = self.get_min_distance(depth_image.copy())
                sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350.0 * 255.0
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                # bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                if not self.moving:
                    object_info_list = self.shape_recognition(rgb_image, depth_image.copy(), depth_color_map, depth_camera_info.k, min_dist)
                    reorder_object_info_list = object_info_list
                    if object_info_list and self.last_object_info_list:
                        reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)

                    if self.start and reorder_object_info_list:
                        if self.target_shapes is None:
                            indices = [i for i, info in enumerate(reorder_object_info_list) if self.shapes is None or info[0] in self.shapes]
                            if indices:
                                min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][3])
                                self.target_shapes = reorder_object_info_list[min_depth_index][0]
                        else:
                            target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0] == self.target_shapes]
                            if target_index:
                                target_index = target_index[0]
                                object_info = reorder_object_info_list[target_index]
                                x, y, w, h, center, width, height, angle = object_info[4]                             
                                cv2.putText(depth_color_map, object_info[0] + str(object_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
                                cv2.putText(depth_color_map, object_info[0] + str(object_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
                                cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1, (0, 0, 255), 2, cv2.LINE_AA)
                                position = object_info[2]
                                e_distance = round(math.sqrt((self.last_position[0] - position[0]) ** 2) + math.sqrt((self.last_position[1] - position[1]) ** 2), 5)
                                if e_distance <= 0.005:
                                    self.count += 1
                                else:
                                    self.count = 0
                                if self.count > 12:
                                    self.count = 0
                                    self.target_shapes = None
                                    self.moving = True
                                    threading.Thread(target=self.move, args=(object_info,), daemon=True).start()
                                self.last_position = (position[0], position[1])
                            else:
                                self.target_shapes = None
                    self.last_object_info_list = reorder_object_info_list

                self.fps.update()
                bgr_image = rgb_image               
                cv2.rectangle(bgr_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (0, 255, 255), 1)
                result_image = np.concatenate([self.fps.show_fps(bgr_image), depth_color_map], axis=1)
                if self.display:
                    cv2.imshow('depth', result_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        self.running = False
                    elif key == ord('s'):
                        self.start = True
                    elif key == ord('a'):
                        self.start = False
                        self.moving = False
                        self.goto_default(wait_time=1.0)
            except Exception as e:
                # self.get_logger().info(str(e))
                self.get_logger().error(traceback.format_exc())
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ObjectClassificationNode('shape_recognition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
