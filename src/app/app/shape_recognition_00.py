#!/usr/bin/python3
# coding=utf8

import os
import cv2
import yaml
import time
import math
import queue
import threading
import numpy as np
import rclpy
import message_filters
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener, TransformException
from sdk import common, fps
from app.common import Heart
from interfaces.srv import SetStringList
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.srv import GetArmFullState
from app.play_pose import get_use_scene_pose
from app.utils import utils, calculate_grasp_yaw_by_depth, position_change_detect, pick_and_place

class ShapeRecognitionNode(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.running = True
        self.hand2cam_tf_matrix = None
        self._init_parameters()
        self.lock = threading.RLock()
        self.gripper_depth = 0.015
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.peripherals_config_path = "/home/ubuntu/ros2_ws/src/peripherals/config/"
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.camera_info_file = 'camera_info.yaml'
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        if self.chassis_type == 'Slide_Rails':
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
        else:
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
        self.home_pose = self._load_home_pose_from_scene()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.current_pose = None
        self.known_pose = {
            'x': float(self.home_pose['x']),
            'y': float(self.home_pose['y']),
            'z': float(self.home_pose['z']),
            'pitch': float(self.home_pose['pitch']),
            'roll': float(self.home_pose['roll']),
            'claw': float(self.home_pose['claw']),
            'yaw': 0.0,
        }
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_srv = self.create_service(SetBool, '~/set_running', self.start_srv_callback)
        self.rgb_or_depth_srv = self.create_service(SetBool, '~/rgb_or_depth', self.rgb_or_depth_srv_callback)
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)

        self.timer_cb_group = ReentrantCallbackGroup()
        # self.set_joint_value_target_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target', callback_group=self.timer_cb_group)
        # self.set_joint_value_target_client.wait_for_service()
        # self.kinematics_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        # self.kinematics_client.wait_for_service()
        # self.get_link_client = self.create_client(GetLink, 'kinematics/get_link')
        # self.get_link_client.wait_for_service()
        # self.set_link_client = self.create_client(SetLink, 'kinematics/set_link')
        # self.set_link_client.wait_for_service()
        self.kinematics_client = None
        
        # self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/armpi_ultra_control/ActionGroups')
        self.controller = None
        self.base_gripper_height = utils.get_gripper_size(500)[1]

        self.load_camera_parameters()
        
        tf_buffer = Buffer()
        self.tf_listener = TransformListener(tf_buffer, self)
        tf_future = tf_buffer.wait_for_transform_async(
            target_frame='depth_cam_depth_optical_frame',
            source_frame='depth_cam_color_frame',
            time=rclpy.time.Time())
        rclpy.spin_until_future_complete(self, tf_future)
        try:
            transform = tf_buffer.lookup_transform(
                'depth_cam_color_frame', 'rgb_camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0))
            self.static_transform = transform
        except TransformException as e:
            self.get_logger().error(f'Failed to get static transform: {e}')

        if self.hand2cam_tf_matrix is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transform_matrix = common.xyz_quat_to_mat([translation.x, translation.y, translation.z],
                                                      [rotation.w, rotation.x, rotation.y, rotation.z])
            self.hand2cam_tf_matrix = np.matmul(transform_matrix, self.hand2cam_tf_matrix)
        
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def load_camera_parameters(self):
        try:
            with open(os.path.join(self.peripherals_config_path, self.camera_info_file), 'r') as f:
                config = yaml.safe_load(f)
                if 'hand2cam_tf_matrix' in config:
                    self.hand2cam_tf_matrix = np.array(config['hand2cam_tf_matrix'])
                    self.get_logger().info('Successfully loaded hand2cam_tf_matrix')
                else:
                    self.get_logger().error(f"'hand2cam_tf_matrix' not found in {self.camera_info_file}")
        except FileNotFoundError:
            self.get_logger().error(f"Camera config file not found: {os.path.join(self.peripherals_config_path, self.camera_info_file)}")
        except Exception as e:
            self.get_logger().error(f"Error loading camera parameters: {e}")

    def init_process(self):
        self.timer.cancel()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        if self.get_parameter('start').value:
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            req = SetBool.Request()
            req.data = True
            res = SetBool.Response()
            self.start_srv_callback(req, res)
            self.shapes = ['cuboid', 'sphere', 'cylinder']
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _init_parameters(self):
        self.heart = None
        self.sync = None
        self.start_transport = False
        self.enter = False
        self.count_still = 0
        self.count_move = 0
        self.enable_transport = False
        self.plane = []
        self.display_rgb = True
        self.shapes = None
        self.target = None
        self.image_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.depth_info_sub = None
        self.target_shape = None
        self.endpoint = None
        self.corners = []
        self.extristric = []
        self.last_position = 0, 0
        self.last_object_info_list = []
        self.display = self.get_parameter('display').value
        self.app = self.get_parameter('app').value

    def _load_scene_config(self):
        cfg = {}
        try:
            cfg = common.get_yaml_data(self.scene_config_path) or {}
        except Exception:
            cfg = {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            scenes = {'scene_1': {}}
        scene_name = str(cfg.get('current_scene', 'scene_1'))
        if scene_name not in scenes:
            scene_name = next(iter(scenes.keys()))
        return scenes.get(scene_name, {})

    def _load_home_pose_from_scene(self):
        default_pose = {
            'x': 110.0,
            'y': 0.0,
            'z': 220.0,
            'pitch': -90.0,
            'roll': 0.0,
            'claw': 0.0,
        }
        if not get_use_scene_pose(self):
            return default_pose
        scene_cfg = self._load_scene_config()
        home = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
        return {
            'x': float(home.get('x', default_pose['x'])),
            'y': float(home.get('y', default_pose['y'])),
            'z': float(home.get('z', default_pose['z'])),
            'pitch': float(home.get('pitch', default_pose['pitch'])),
            'roll': float(home.get('roll', default_pose['roll'])),
            'claw': float(home.get('claw', default_pose['claw'])),
        }

    def arm_state_callback(self, msg):
        self.current_pose = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'pitch': float(msg.pitch),
            'roll': float(msg.roll),
            'claw': float(msg.claw),
            'yaw': float(msg.yaw),
            'joint_angles': [float(v) for v in msg.joint_angles],
        }

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def request_real_pose_snapshot(self, timeout_sec=0.6):
        if not self.arm_state_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
            return None
        future = self.arm_state_client.call_async(GetArmFullState.Request())
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'获取真实末端状态失败: {exc}')
            return None
        if response is None or not response.success:
            return None
        pose = {
            'x': float(response.x),
            'y': float(response.y),
            'z': float(response.z),
            'pitch': float(response.pitch),
            'roll': float(response.roll),
            'claw': float(response.claw),
            'yaw': float(response.yaw),
            'joint_angles': [float(v) for v in response.joint_angles],
        }
        self.current_pose = dict(pose)
        return pose

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
        yaw_deg = math.degrees(math.atan2(float(y), float(x))) if (float(x) != 0.0 or float(y) != 0.0) else 0.0
        self.known_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'pitch': float(pitch),
            'roll': float(roll),
            'claw': float(claw),
            'yaw': yaw_deg,
        }

    def get_endpoint_matrix(self, pose=None):
        p = pose if pose is not None else (self.current_pose if self.current_pose is not None else self.known_pose)
        x_m = p['x'] / 1000.0
        y_m = p['y'] / 1000.0
        z_m = p['z'] / 1000.0
        yaw_deg = p.get('yaw')
        if yaw_deg is None:
            yaw_deg = math.degrees(math.atan2(p['y'], p['x'])) if p['x'] or p['y'] else 0.0
        return common.xyz_euler_to_mat(
            [x_m, y_m, z_m],
            [float(p.get('roll', 0.0)), -float(p.get('pitch', 0.0)), float(yaw_deg)],
            degrees=True,
        )

    def go_home(self, interrupt=True):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        if interrupt:
            self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], 30, 500)
            time.sleep(0.5)
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1.0)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        self.endpoint = self.get_endpoint_matrix(pose_snapshot)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter shape recognition")
        self._init_parameters()
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        config = common.get_yaml_data(os.path.join(self.config_path, self.config_file))
        self.plane = config['plane']
        self.corners = np.array(config['corners'])
        self.extristric = np.array(config['extristric'])
        if self.sync is None:
            self.rgb_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
            self.depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')
            self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/depth/camera_info')
            self.info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/rgb/camera_info')
            self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub, self.depth_info_sub], 3, 0.2)
            self.sync.registerCallback(self.multi_callback)
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        self.enter = True
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1.0)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        self.endpoint = self.get_endpoint_matrix(pose_snapshot)
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit shape recognition")
            if self.sync is not None:
                self.sync = None
                self.rgb_sub = None
                self.depth_sub = None
                self.depth_info_sub = None
                self.info_sub = None
            self.heart.destroy()
            self.heart = None
            pick_and_place.interrupt(True)
            self.enter = False
            self.start_transport = False
        response.success = True
        response.message = "stop"
        return response

    def start_srv_callback(self, request, response):
        if request.data:
            self.get_logger().info('\033[1;32m%s\033[0m' % "start shape recognition")
            if self.app:
                msg = SetStringList.Request()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg, SetStringList.Response())
            pick_and_place.interrupt(False)
            self.enable_transport = True
            response.success = True
            response.message = "start"
            return response
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % "stop shape recognition")
            self.enable_transport = False
            self.target_shape = None
            pick_and_place.interrupt(True)
            response.success = False
            response.message = "stop"
            return response

    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_shape")
        self.shapes = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % str(self.shapes))
        response.success = True
        response.message = "set_shape"
        return response

    def rgb_or_depth_srv_callback(self, request, response):
        self.display_rgb = request.data
        response.success = True
        return response

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                msg = BuzzerState()
                msg.freq = 1900
                msg.on_time = 0.2
                msg.off_time = 0.01
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                time.sleep(1)
                shape, position, yaw = self.transport_info[0], self.transport_info[2], self.transport_info[-1]
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]
                if "sphere" in shape:
                    finish = pick_and_place.pick(position, 85, yaw, 570, 0.05, self.arm_pub, None)
                else:
                    finish = pick_and_place.pick(position, 85, yaw, 570, 0.02, self.arm_pub, None)
                if finish:
                    if "sphere" in shape:
                        # self.controller.run_action("target_1")
                        self.publish_arm(95, -214, 20, -90, 0, -60, 1500)
                        time.sleep(1.5)
                    if "cylinder" in shape:
                        # self.controller.run_action("target_2")
                        self.publish_arm(-18, -214, 20, -90, 0, -60, 1500)
                        time.sleep(1.5)
                    if "cuboid" in shape:
                        # self.controller.run_action("target_3")
                        self.publish_arm(-70, -214, 20, -90, 0, -60, 1500)
                        time.sleep(1.5)
                    self.go_home(False)
                else:
                    self.go_home(True)
                self.start_transport = False
            else:
                time.sleep(0.1)

    def shape_recognition(self, bgr_image, depth_image, depth_color_map, depth_intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        min_dist_m = min_dist / 1000.0
        if min_dist_m <= 0.3:
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            
            plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
            
            binary_mask = np.where(plane_values > 0.015, 255, 0).astype(np.uint8)
            kernel = np.ones((5,5),np.uint8)
            cleaned_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel, iterations = 2)
            contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            fx, fy = depth_intrinsic_matrix[0], depth_intrinsic_matrix[4]
            
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 500:
                    continue

                perimeter = cv2.arcLength(obj, True)
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
                CornerNum = len(approx)

                (cx, cy), radius = cv2.minEnclosingCircle(obj)
                rect = cv2.minAreaRect(obj)
                center, (width, height), angle = rect
                long_edge_angle = utils.get_long_edge_angle(rect)
                if width < height:
                    width, height = height, width

                depth = depth_image[int(cy), int(cx)]
                if self.hand2cam_tf_matrix is None:
                    self.get_logger().warn("手眼变换矩阵未加载，跳过位置计算。")
                    continue
                
                position = utils.calculate_world_position(cx, cy, depth, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_intrinsic_matrix)
                if position is None:
                    continue

                mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                cv2.drawContours(mask, [obj], -1, 255, cv2.FILLED)
                
                top_surface_heights = plane_values[mask == 255]
                top_surface_heights = top_surface_heights[top_surface_heights > 0.005]
                
                height_std = np.std(top_surface_heights) if len(top_surface_heights) > 0 else 0
                
                objType = None
                log_msg_details = f"height_std={height_std:.6f}, CornerNum={CornerNum}"

                # 1. 判断表面是否有弧度
                if height_std > 0.003:
                    # 对于有弧度的表面，用宽高比区分球和躺倒的圆柱
                    if height > 0:
                        aspect_ratio = width / height
                        log_msg_details += f", aspect_ratio={aspect_ratio:.2f}"
                    else:
                        aspect_ratio = 1 # 避免除零错误

                    if aspect_ratio < 1.5:
                        sphere_index += 1
                        objType = 'sphere_' + str(sphere_index)
                    else:
                        cylinder_index += 1
                        objType = "cylinder_" + str(cylinder_index)
                    angle = 0 

                else:
                    if CornerNum >= 5: 
                        cylinder_index += 1
                        objType = "cylinder_" + str(cylinder_index)
                        angle = 0
                        long_edge_angle = 0
                    else: 
                        cuboid_index += 1
                        objType = "cuboid_" + str(cuboid_index)
                
                self.get_logger().info(f"检测到物体: {log_msg_details}  =>  分类结果: {objType}")
                
                if objType is not None:
                    object_height_m = (min_dist - depth) / 1000.0
                    object_top_z_world = position[2] + object_height_m
                    gripper_open_external_width = int(fx / (depth / 1000.0) * (0.08 + 0.01))
                    gripper_open_external_height = int(fy / (depth / 1000.0) * 0.015)
                    gripper_inner_width = int(fx / (depth / 1000.0) * 0.055)
                    gripper_close_w, _ = utils.get_gripper_size(570)
                    gripper_close_width = int(fx / (depth / 1000.0) * gripper_close_w)
                    
                    mask_rect1, mask_rect2 = calculate_grasp_yaw_by_depth.get_gripper_masks(depth_image, center, long_edge_angle, gripper_open_external_width, gripper_open_external_height)
                    obstacle_mask = calculate_grasp_yaw_by_depth.get_obstacle_mask(depth_image, obj, plane_values, 0.09, object_top_z_world, self.gripper_depth)
                    mask1 = cv2.bitwise_and(mask_rect1, obstacle_mask)
                    mask2 = cv2.bitwise_and(mask_rect2, obstacle_mask)
                    angle_list = calculate_grasp_yaw_by_depth.calculate_obj_angles(width, height, gripper_inner_width, [long_edge_angle, np.any(mask1)], [long_edge_angle - 90, np.any(mask2)])
                    yaw, gripper_angle = calculate_grasp_yaw_by_depth.calculate_grasp_angle(position, angle_list, long_edge_angle)
                    if yaw is not None:
                        yaw = utils.normalize_gripper_roll_deg(yaw)
                    
                    x, y, w, h = cv2.boundingRect(obj)
                    
                    if yaw is not None:
                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (gripper_open_external_width, gripper_open_external_height), gripper_angle)))], -1, (0, 255, 255), 2, cv2.LINE_AA)
                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (gripper_close_width, gripper_open_external_height), gripper_angle)))], -1, (255, 0, 255), 2, cv2.LINE_AA)
                        
                        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                        offset = tuple(config_data['depth']['offset'])
                        scale = tuple(config_data['depth']['scale'])
                        for i in range(3):
                            position[i] = position[i] * scale[i]
                            position[i] = position[i] + offset[i]
                            
                        index = int(objType.split('_')[-1])
                        obj_name = objType.split('_')[0]
                        object_info_list.append([obj_name, index, position, depth, [x, y, w, h, center, width, height, angle], bgr_image[int(center[1]), int(center[0])], yaw])

                    cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)

        return object_info_list
    def main(self):
        while self.running:
            if self.enter:
                try:
                    bgr_image, depth_image, camera_info, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue
                img = bgr_image.copy()
                try:
                    max_dist = 350
                    depth_image = utils.create_roi_mask(depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.08)
                    min_dist = utils.find_depth_range(depth_image, max_dist)
                    sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                    
                    if not self.start_transport and self.shapes is not None:
                        if self.enable_transport:
                            object_info_list = self.shape_recognition(bgr_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                            reorder_object_info_list = object_info_list
                            if object_info_list:
                                if self.last_object_info_list:
                                    reorder_object_info_list = position_change_detect.position_reorder(object_info_list, self.last_object_info_list, 20)
                            if reorder_object_info_list:
                                if self.target_shape is None:
                                    indices = [i for i, info in enumerate(reorder_object_info_list) if info[0] in self.shapes]
                                    if indices:
                                        min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][3])
                                        self.target_shape = reorder_object_info_list[min_depth_index][0]
                                else:
                                    target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0] == self.target_shape]
                                    if target_index:
                                        target_index = target_index[0]
                                        obejct_info = reorder_object_info_list[target_index]
                                        x, y, w, h, center, width, height, angle = obejct_info[4]
                                        cv2.putText(depth_color_map, obejct_info[0] + str(obejct_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
                                        cv2.putText(depth_color_map, obejct_info[0] + str(obejct_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
                                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1, (0, 0, 255), 2, cv2.LINE_AA)
                                        position = obejct_info[2]
                                        e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(pow(self.last_position[1] - position[1], 2)), 5)
                                        if e_distance <= 0.005:
                                            self.count_move = 0
                                            self.count_still += 1
                                        else:
                                            self.count_move += 1
                                            self.count_still = 0
                                        if self.count_still > 10:
                                            self.count_still = 0
                                            self.count_move = 0
                                            self.start_transport = True
                                            self.transport_info = obejct_info
                                        self.last_position = position
                                    else:
                                        self.target_shape = None
                            self.last_object_info_list = reorder_object_info_list
                    
                    zero_color = depth_color_map[sim_depth_image == 0][0]
                    depth_color_map_padded = cv2.copyMakeBorder(
                        depth_color_map,
                        0, 0, 0, 0,
                        borderType=cv2.BORDER_CONSTANT,
                        value=tuple(map(int, zero_color)))
                    if self.display_rgb:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB), "rgb8"))
                    else:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(depth_color_map_padded, "bgr8"))
                    if self.display:
                        result_image = np.concatenate([bgr_image, depth_color_map_padded], axis=1)
                        cv2.imshow("depth", result_image)
                        cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().info(str(e))
            else:
                time.sleep(0.1)

    def multi_callback(self, ros_rgb_image, ros_depth_image, camera_info, depth_camera_info):
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        bgr_image = rgb_image
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((bgr_image, depth_image, camera_info, depth_camera_info))

def main():
    rclpy.init()
    node = ShapeRecognitionNode('shape_recognition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()

if __name__ == "__main__":
    main()
