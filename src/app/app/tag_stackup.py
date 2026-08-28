#!/usr/bin/env python3
# coding: utf8
# 标签码垛
import os
import cv2
import copy
import yaml
import time
import math
import queue
import rclpy
import threading
import numpy as np
from sdk import common, fps
from rclpy.node import Node
from app.common import Heart
from app import scene_play_registry
from cv_bridge import CvBridge
from dt_apriltags import Detector
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import SetStringBool
from ros_robot_controller_msgs.msg import ArmCoords, ArmFullState
from ros_robot_controller_msgs.srv import GetArmFullState
from rclpy.callback_groups import ReentrantCallbackGroup
from app import calibrated_pose
from app.play_pose import get_use_scene_pose
from app.utils import calculate_grasp_yaw, position_change_detect, pick_and_place, image_process, distortion_inverse_map, utils

DEFAULT_SCENE_ID = 'scene_1'
DEFAULT_PLACE_POLICY = {
    'only_left_y_positive': True,
    'min_place_z': 0.025,
}
DEFAULT_SCENE_PLACE_TARGETS = {
    'tag_stackup': [0.0, 0.13, 0.025],
}

class TagStackup(Node):
    # hand2cam_tf_matrix 
    place_position = [0.0, 0.13, 0.025]

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.tag_size = 0.025
        self.target_miss_count = 0
        self.count_move = 0
        self.count_still = 0
        self.running = True
        self.enter = False
        self.get_height = False
        self.intrinsic = None
        self.last_object_info_list = None
        self.endpoint = None
        self.start_get_roi = False
        self.transport_info = None
        self.start_transport = False
        self.distortion = None
        self.extristric = None
        self.white_area_center = None
        self.roi = None
        self.enable_stackup = False
        self.last_position = None
        self.target = None
        self.count = 0
        self.err_msg = None
        self.target_labels = ["tag1", "tag2", "tag3"]
        self.hand2cam_tf_matrix = None # 初始化手眼变换矩阵 (Initialize hand-eye transformation matrix)

        self.camera_type = os.environ.get('CAMERA_TYPE', '').lower()
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        # 更新配置文件路径 (Update configuration file paths)
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.camera_info_file = 'camera_info.yaml'
        self.app_config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        if self.chassis_type == 'Slide_Rails':
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
        else:
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
        self.home_pose = self._load_home_pose_from_scene()
        self.peripherals_config_path = "/home/ubuntu/ros2_ws/src/peripherals/config/"
        
        self.image_queue = queue.Queue(maxsize=2)

        self.lock = threading.RLock()
        self.fps = fps.FPS()  # 帧率统计器 (frame rate counter)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换 (For converting between ROS Image messages and OpenCV images)

        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        # 服务和话题 (services and topics)
        self.image_sub = None
        self.camera_info_sub = None

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.current_pose = None
        self.create_subscription(ArmFullState, '/ros_robot_controller/arm/full_state', self.arm_state_callback, 5)
        self.arm_state_client = self.create_client(GetArmFullState, '/ros_robot_controller/arm/get_full_state')
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_stack_up_srv = self.create_service(SetBool, '~/enable_stackup', self.enable_stackup_srv_callback)

        timer_cb_group = ReentrantCallbackGroup()

        self.known_pose = {
            'x': float(self.home_pose['x']),
            'y': float(self.home_pose['y']),
            'z': float(self.home_pose['z']),
            'pitch': float(self.home_pose['pitch']),
            'roll': float(self.home_pose['roll']),
            'claw': float(self.home_pose['claw']),
            'yaw': 0.0,
        }
        self.endpoint = np.eye(4)

        self.load_camera_parameters() # 节点初始化时加载相机参数 (Load camera parameters on node initialization)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

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

    def load_camera_parameters(self):
        """
        从 camera_info.yaml 加载手眼变换矩阵 (Load hand-eye transformation matrix from camera_info.yaml)
        """
        try:
            with open(os.path.join(self.peripherals_config_path, self.camera_info_file), 'r') as f:
                config = yaml.safe_load(f)
                if 'hand2cam_tf_matrix' in config:
                    self.hand2cam_tf_matrix = np.array(config['hand2cam_tf_matrix'])
                    self.get_logger().info('成功加载 hand2cam_tf_matrix (Successfully loaded hand2cam_tf_matrix)')
                else:
                    self.get_logger().error(f"在 {self.camera_info_file} 中未找到 'hand2cam_tf_matrix' ('hand2cam_tf_matrix' not found in {self.camera_info_file})")
        except FileNotFoundError:
            self.get_logger().error(f"相机配置文件未找到 (Camera config file not found): {os.path.join(self.peripherals_config_path, self.camera_info_file)}")
        except Exception as e:
            self.get_logger().error(f"加载相机参数时出错 (Error loading camera parameters): {e}")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _load_scene_config(self):
        cfg = {}
        try:
            cfg = common.get_yaml_data(self.scene_config_path) or {}
        except Exception:
            cfg = {}
        scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
        if not isinstance(scenes, dict) or not scenes:
            scenes = {DEFAULT_SCENE_ID: {}}
        scene_name = str(cfg.get('current_scene', DEFAULT_SCENE_ID))
        if scene_name not in scenes:
            scene_name = next(iter(scenes.keys()))
        scene_cfg = scenes.get(scene_name, {})
        if not isinstance(scene_cfg, dict):
            scene_cfg = {}
        return scene_play_registry.merge_play_into_scene(scene_name, scene_cfg)

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

    def resolve_place_position(self):
        scene_cfg = self._load_scene_config()
        place_targets = scene_cfg.get('place_targets', {}) if isinstance(scene_cfg.get('place_targets'), dict) else {}
        raw = place_targets.get('tag_stackup', DEFAULT_SCENE_PLACE_TARGETS['tag_stackup'])
        try:
            pos = [float(raw[0]), float(raw[1]), float(raw[2])]
        except Exception:
            pos = copy.deepcopy(DEFAULT_SCENE_PLACE_TARGETS['tag_stackup'])

        # 保持当前码垛高度逻辑，只将场景配置用于 XY。
        pos[2] = float(self.place_position[2])
        policy = DEFAULT_PLACE_POLICY.copy()
        if isinstance(scene_cfg.get('place_policy'), dict):
            policy.update(scene_cfg['place_policy'])
        if bool(policy.get('only_left_y_positive', False)) and pos[1] < 0.0:
            pos[1] = abs(pos[1])
        try:
            min_place_z = float(policy.get('min_place_z', DEFAULT_PLACE_POLICY['min_place_z']))
            if pos[2] < min_place_z:
                pos[2] = min_place_z
        except Exception:
            pass
        return pos

    def init_process(self):
        self.timer.cancel()

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % '启动 (start)')

    def get_endpoint(self, pose=None):
        p = pose if pose is not None else (self.current_pose if self.current_pose is not None else self.known_pose)
        x_m = p['x'] / 1000.0
        y_m = p['y'] / 1000.0
        z_m = p['z'] / 1000.0
        yaw_deg = p.get('yaw')
        if yaw_deg is None:
            yaw_deg = math.degrees(math.atan2(p['y'], p['x'])) if p['x'] or p['y'] else 0.0
        self.endpoint = common.xyz_euler_to_mat(
            [x_m, y_m, z_m],
            [float(p.get('roll', 0.0)), -float(p.get('pitch', 0.0)), float(yaw_deg)],
            degrees=True,
        )

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "加载标签码垛 (Loading tag stackup)")
        with self.lock:
            self.enter = True
            self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(None, response=Trigger.Response()))  # 心跳包 (heartbeat package)
            self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
            self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)

            self.home_pose = self._load_home_pose_from_scene()
            hp = self.home_pose
            self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
            time.sleep(1)

            self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
            time.sleep(1)
            pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
            self.get_endpoint(pose_snapshot)
        self.start_get_roi = True

        response.success = True
        response.message = "enter"
        return response

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def exit_srv_callback(self, request, response):
        if self.enter and request is not None:
            self.get_logger().info('\033[1;32m%s\033[0m' % "退出标签码垛 (exit tag stackup)")
            with self.lock:
                self.enter = False
                self.start_transport = False
                self.enable_stackup = False
                try:
                    if self.image_sub is not None:
                        self.destroy_subscription(self.image_sub)
                        self.image_sub = None
                    if self.camera_info_sub is not None:
                        self.destroy_subscription(self.camera_info_sub)
                        self.camera_info_sub = None
                except Exception as e:
                    self.get_logger().error(str(e))
                self.heart.destroy()
                self.heart = None
                self.err_msg = None
                pick_and_place.interrupt(True)
        elif not self.enter and request is None:
            self.get_logger().info('\033[1;32m%s\033[0m' % "心跳已停止 (heart already stop)")

        response.success = True
        response.message = "exit"
        return response

    def enable_stackup_srv_callback(self, request, response):
        with self.lock:
            if request.data:
                self.get_logger().info('\033[1;32m%s\033[0m' % "开始标签码垛 (start tag stackup)")
                self.enable_stackup = True
                self.last_position = None
                pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
                self.get_endpoint(pose_snapshot)
                self.go_left()
                pick_and_place.interrupt(False)
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % "停止标签码垛 (stop tag stackup)")
                self.enable_stackup = False
                pick_and_place.interrupt(True)
                self.err_msg = None
        response.success = True
        response.message = "start"
        return response

    def go_home(self):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1)

        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        self.get_endpoint(pose_snapshot)

    def go_left(self):
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1000)
        time.sleep(1)
        pose_snapshot = self.request_real_pose_snapshot() or self.current_pose or self.known_pose
        self.get_endpoint(pose_snapshot)
        self.get_height = True

    def image_callback(self, ros_image):
        # 将ros格式图像转换为opencv格式 (convert the ros format image to opencv format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)

        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像 (If the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列 (Put the image into the queue)
        self.image_queue.put(bgr_image)

    def get_roi(self):
        with open(os.path.join(self.app_config_path, self.config_file), 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组 (Convert to numpy array)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]  # 取第一行 (Take the first row)
        rmat = extristric[1:]  # 取后面三行 (Take the next three rows)

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # 裁切出ROI区域 (crop ROI region)
        x_min = min(imgpts, key=lambda p: p[0])[0]  # x轴最小值 (x-axis minimum value)
        x_max = max(imgpts, key=lambda p: p[0])[0]  # x轴最大值 (x-axis maximum value)
        y_min = min(imgpts, key=lambda p: p[1])[1]  # y轴最小值 (y-axis minimum value)
        y_max = max(imgpts, key=lambda p: p[1])[1]  # y轴最大值 (y-axis maximum value)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

        self.roi = roi

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.03):
        config_data = calibrated_pose.load_axis_calibration(self.app_config_path, self.calibration_file)
        return calibrated_pose.pixel_to_calibrated_world(
            position,
            intrinsic,
            extristric,
            white_area_center,
            config_data,
            height=height,
        )

    def calculate_pick_grasp_yaw(self, position, target, target_info, intrinsic, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        # 0.09x0.02
        gripper_size = [common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
                        common.calculate_pixel_length(0.02, intrinsic, projection_matrix)]

        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info, gripper_size, yaw)

    def calculate_place_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        yaw1 = yaw + angle
        if yaw < 0:
            yaw2 = yaw1 + 90
        else:
            yaw2 = yaw1 - 90

        yaw = yaw2
        if abs(yaw1) < abs(yaw2):
            yaw = yaw1

        return utils.normalize_gripper_roll_deg(yaw)

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                position, yaw, target = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.app_config_path, self.calibration_file))
                position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics').tolist()

                finish = pick_and_place.pick(position, 80, yaw, 540, 0.02, self.arm_pub)
                if finish:
                    position = self.resolve_place_position()

                    yaw = self.calculate_place_grasp_yaw(position, 0)
                    config_data = common.get_yaml_data(os.path.join(self.app_config_path, self.calibration_file))
                    offset = tuple(config_data['kinematics']['offset'])
                    scale = tuple(config_data['kinematics']['scale'])
                    angle = math.degrees(math.atan2(position[1], position[0]))
                    if angle > 45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] - offset[1], position[1] + offset[0], position[2] + offset[2]]
                    elif angle < -45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
                    else:
                        position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
                        position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

                    finish = pick_and_place.place(position, 80, yaw, 200, self.arm_pub)
                    if finish:
                        self.go_left()
                    else:
                        self.go_home()
                else:
                    self.go_home()
                self.target = None
                self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while self.running:
            if self.enter:
                try:
                    bgr_image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue

                if self.start_get_roi:
                    self.get_roi()
                    self.start_get_roi = False
                
                target_info = []
                if self.enable_stackup and not self.start_transport:
                    if self.hand2cam_tf_matrix is None:
                        self.get_logger().warn("手眼矩阵未加载，跳过处理 (Hand-eye matrix not loaded, skipping processing)")
                        time.sleep(0.5)
                        continue

                    tags = self.at_detector.detect(cv2.cvtColor(bgr_image, cv2.COLOR_RGB2GRAY), True, (self.intrinsic[0,0], self.intrinsic[1,1], self.intrinsic[0,2], self.intrinsic[1,2]), self.tag_size)
                    if len(tags) > 0:
                        index = 0
                        for tag in tags:
                            if 'tag%d' % tag.tag_id in self.target_labels:
                                corners = tag.corners.astype(int)
                                cv2.drawContours(bgr_image, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
                                rect = cv2.minAreaRect(np.array(tag.corners).astype(np.float32))
                                (center, (width, height), _) = rect
                                angle = utils.get_long_edge_angle(rect)
                                index += 1
                                target_info.append(['tag%d' % tag.tag_id, index, (int(center[0]), int(center[1])), (int(width), int(height)), angle])

                        if self.get_height:
                            # 获取标签木块的高度 (Get the height of the tag block)
                            self.count += 1
                            if self.count > 15:
                                self.count = 0
                                pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_rot_to_mat(tags[0].pose_t, tags[0].pose_R))  # (转换到末端相对坐标) (Convert to relative end-effector coordinates)
                                pose_world = np.matmul(self.endpoint, pose_end)  # (转换到机械臂世界坐标) (Convert to robot arm world coordinates)
                                pose_world_T, _ = common.mat_to_xyz_euler(pose_world, degrees=True)
                                if self.camera_type == 'usb_cam':
                                    pose_world_T[2] += 0.05
                                if pose_world_T[-1] > 0.09:
                                    self.err_msg = "太高了，请先移除一些方块!!! (Too high, please remove some blocks first!!!)"
                                else:
                                    self.err_msg = None
                                    self.place_position[2] = pose_world_T[2] -0.01
                                    self.get_height = False
                                    self.go_home()

                    if target_info:
                        if self.last_object_info_list:
                            # 对比上一次的物体的位置来重新排序 (Reorder based on the previous object positions)
                            target_info = position_change_detect.position_reorder(target_info, self.last_object_info_list, 20)
                    self.last_object_info_list = copy.deepcopy(target_info)
                    for target in target_info:
                        cv2.putText(bgr_image, '{}'.format(target[0]), (target[2][0] - 4 * len(target[0] + str(target[1])), target[2][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    target_miss = True
                    for target in target_info:  # 检测 (detect)
                        if self.target is not None:  # 如果已经有了目标，其他物体就直接跳过 (If a target already exists, skip other objects)
                            if self.target[0] != target[0] or self.target[1] != target[1]:
                                continue
                            else:
                                target_miss = False
                                self.target = target
                        if self.camera_type == 'usb_cam':
                            # 将校正后图像的像素点反推回原始（带畸变）图像中的像素点 (Map undistorted pixel points back to the original distorted image)
                            x, y = distortion_inverse_map.undistorted_to_distorted_pixel(target[2][0], target[2][1], self.intrinsic, self.distortion)
                            target[2] = (x, y)

                        position, projection_matrix = self.get_object_world_position(target[2], self.intrinsic, self.extristric, self.white_area_center)
                        result = self.calculate_pick_grasp_yaw(position, target, target_info, self.intrinsic, projection_matrix)
                        if result is not None and self.target is None:
                            self.target = target
                            break

                        if self.last_position is not None and self.target is not None and result is not None and not self.get_height:
                            e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                pow(self.last_position[1] - position[1], 2)), 5)
                            if e_distance <= 0.005:  # 欧式距离小于0.005, 防止物体还在移动时就去夹取 (Euclidean distance is less than 0.005 to prevent grasping while the object is still moving)
                                cv2.line(bgr_image, result[1][0], result[1][1], (255, 255, 0), 2, cv2.LINE_AA)
                                self.count_move = 0
                                self.count_still += 1
                            else:
                                self.count_move += 1
                                self.count_still = 0

                            if self.count_move > 10:
                                self.target = None
                            if self.count_still > 20:
                                self.count_still = 0
                                self.count_move = 0
                                self.target = target
                                yaw = utils.normalize_gripper_roll_deg(result[0])
                                self.transport_info = [position, yaw, target]
                                self.start_transport = True
                        self.last_position = position
                    if target_miss:
                        self.target_miss_count += 1
                    if self.target_miss_count > 10:
                        self.target_miss_count = 0
                        self.target = None

                if bgr_image is not None and self.get_parameter('display').value:
                    cv2.imshow('result_image', bgr_image)
                    cv2.waitKey(1)
                if self.err_msg is not None:
                    self.get_logger().error(self.err_msg)
                    err_msg = self.err_msg.split(';')
                    for i, m in enumerate(err_msg):
                        cv2.putText(bgr_image, m, (10, 150 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 7)
                        cv2.putText(bgr_image, m, (10, 150 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
            else:
                time.sleep(0.1)


def main():
    rclpy.init()
    node = TagStackup('tag_stackup')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
