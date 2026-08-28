#!/usr/bin/env python3
# coding: utf8
# Tag Sorting 标签分拣

import os
import cv2
import yaml
import time
import math
import queue
import rclpy
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from dt_apriltags import Detector
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import SetStringBool
from ros_robot_controller_msgs.msg import ArmCoords
from ros_robot_controller_msgs.srv import MoveStepper
from rclpy.callback_groups import ReentrantCallbackGroup
from app import calibrated_pose
from app.utils import calculate_grasp_yaw, position_change_detect, pick_and_place, image_process, distortion_inverse_map, utils



TARGET_POSITION = {
    'tag1': (2600),
    'tag2': (4000),
    'tag3': (5200),
}

class TagSorting(Node):

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.tag_size = 0.025
        self.start_get_roi = True
        self.last_position = None
        self.imgpts = None
        self.roi = None
        self.count_move = 0
        self.count_still = 0
        self.last_position = None
        self.start_transport = False
        self.extristric = None
        self.intrinsic = None
        self.distortion = None
        self.target = None
        self.target_miss_count = 0
        self.camera_type = os.environ['CAMERA_TYPE']
        self.calibration_file = 'calibration.yaml'
        self.config_file = 'transform.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/"
        self.scene_config_path = os.path.join(self.config_path, 'calibration_scene.yaml')
        self.home_pose = self._load_home_pose()

        self.image_queue = queue.Queue(maxsize=2)
        self.lock = threading.RLock()
        self.bridge = CvBridge()  # Used for the conversion between ROS Image messages and OpenCV images. 用于ROS Image消息与OpenCV图像之间的转换
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)


        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.move_stepper_client = self.create_client(MoveStepper, '/ros_robot_controller/move_stepper', callback_group=self.timer_cb_group)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)

    def _load_home_pose(self):
        home = {
            'x': 105.0,
            'y': 0.0,
            'z': 200.0,
            'pitch': -90.0,
            'roll': 0.0,
            'claw': -60.0,
        }
        try:
            cfg = common.get_yaml_data(self.scene_config_path) or {}
            scenes = cfg.get('scenes') if isinstance(cfg, dict) else None
            if isinstance(scenes, dict) and scenes:
                scene_name = str(cfg.get('current_scene', 'scene_1'))
                if scene_name not in scenes:
                    scene_name = next(iter(scenes.keys()))
                scene_cfg = scenes.get(scene_name, {}) if isinstance(scenes.get(scene_name), dict) else {}
                hp = scene_cfg.get('home_pose', {}) if isinstance(scene_cfg.get('home_pose'), dict) else {}
                home['y'] = float(hp.get('y', home['y']))
                home['z'] = float(hp.get('z', home['z']))
                home['pitch'] = float(hp.get('pitch', home['pitch']))
                home['roll'] = float(hp.get('roll', home['roll']))
                home['claw'] = float(hp.get('claw', home['claw']))
        except Exception:
            pass
        return home

    def init_process(self):
        self.timer.cancel()
        req = MoveStepper.Request()
        req.steps = 0
        self.send_request(self.move_stepper_client, req)
        self.home_pose = self._load_home_pose()
        hp = self.home_pose
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1500)
        time.sleep(1.5)
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_image):
        # Convert ros format images to opencv format. 将ros格式图像转换为opencv格式
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)

        if self.image_queue.full():
            # If the queue is full, discard the oldest image. 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # Put the image into the queue. 将图像放入队列
        self.image_queue.put(bgr_image)

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # Convert to numpy array. 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]  # Take the first row. 取第一行
        rmat = extristric[1:]  # Take the last three rows. 取后面三行

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.00)
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        self.imgpts = imgpts
        # Crop RIO region 裁切出ROI区域
        x_min = min(imgpts, key=lambda p: p[0])[0] # The minimum value of the X-axis. x轴最小值
        x_max = max(imgpts, key=lambda p: p[0])[0] # The maximum value of the X-axis. x轴最大值
        y_min = min(imgpts, key=lambda p: p[1])[1] # The minimum value of the Y-axis. y轴最小值
        y_max = max(imgpts, key=lambda p: p[1])[1] # The maximum value of the Y-axis. y轴最大值
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            
        self.roi = roi

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.03):
        config_data = calibrated_pose.load_axis_calibration(self.config_path, self.calibration_file)
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
        while True:
            if self.start_transport:
                position, yaw, target = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                position = calibrated_pose.apply_axis_calibration(position, config_data, 'kinematics').tolist()

                finish = pick_and_place.pick(position, 80, yaw, 540, 0.02, self.arm_pub)
                if finish:
                    self.home_pose = self._load_home_pose()
                    hp = self.home_pose
                    self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], 30, 1500)
                    time.sleep(1.5)
                    stepper_position = TARGET_POSITION[str(self.target[0])]     
                    req = MoveStepper.Request()
                    req.steps = stepper_position
                    self.send_request(self.move_stepper_client, req)
                    stepper_time = stepper_position/1000 # Calculate the required time. 计算需要的时间
                    time.sleep(stepper_time)

                    position =  [0.25, 0, 0.015] # Set the placement position. 设置放置位置

                    yaw = self.calculate_place_grasp_yaw(position, 0)
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['kinematics']['offset'])
                    scale = tuple(config_data['kinematics']['scale'])
                    angle = math.degrees(math.atan2(position[1], position[0]))
                    if angle > 45:
                        # self.get_logger().info(f'1:{position}')
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] - offset[1], position[1] + offset[0], position[2] + offset[2]]
                    elif angle < -45:
                        # self.get_logger().info(f'2:{position}')
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
                    else:
                        # self.get_logger().info(f'3:{position}')
                        position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
                        position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

                    # self.get_logger().info(f'{position}')
                    finish = pick_and_place.place(position, 80, yaw, 200, self.arm_pub)
                    if finish:
                        self.home_pose = self._load_home_pose()
                        hp = self.home_pose
                        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], 1500)
                        time.sleep(1.5)
                        req = MoveStepper.Request()
                        req.steps = -stepper_position
                        self.send_request(self.move_stepper_client, req)
                        stepper_time = stepper_position/1000 # 计算需要的时间
                        time.sleep(stepper_time)
                        self.target = None
                        self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while True:
            try:
                bgr_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            if self.start_get_roi:
                self.get_roi()
                self.start_get_roi = False
            roi = self.roi.copy()
            intrinsic = self.intrinsic
            target_info = []
            if not self.start_transport:
                tags = self.at_detector.detect(cv2.cvtColor(bgr_image, cv2.COLOR_RGB2GRAY), True, (intrinsic[0,0], intrinsic[1,1], intrinsic[0,2], intrinsic[1,2]), self.tag_size)
                if len(tags) > 0 and roi[0] < tags[0].center[1] < roi[1] and roi[2] < tags[0].center[0] < roi[3]:
                    common.draw_tags(bgr_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
                    index = 0
                    for tag in tags:
                        corners = tag.corners.astype(int)
                        rect = cv2.minAreaRect(np.array(tag.corners).astype(np.float32))
                        # rect includes center point, (width, height), rotation Angle. rect 包含 (中心点, (宽度, 高度), 旋转角度)
                        (center, (width, height), angle) = rect
                        index += 1
                        target_info.append(['tag%d'%tag.tag_id, index, (int(center[0]), int(center[1])), (int(width), int(height)), angle])
                        cv2.putText(bgr_image, "%d"%tag.tag_id, (int(tag.center[0] - (7 * len("%d"%tag.tag_id))), int(tag.center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    target_miss = True 
                    for target in target_info: 
                        if self.target is not None :  # If there is already a target, skip the other objects directly. 如果已经有了目标，其他物体就直接跳过
                            if self.target[0] != target[0] or self.target[1] != target[1]:
                                continue
                            else:
                                target_miss = False
                                self.target = target
                        position, projection_matrix = self.get_object_world_position(target[2], intrinsic, self.extristric, self.white_area_center, 0.03)
                        result = self.calculate_pick_grasp_yaw(position, target, target_info, intrinsic, projection_matrix)
                        if result is not None and self.target is None:
                            self.target = target
                            break
                        if self.last_position is not None and self.target is not None :
                            e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(pow(self.last_position[1] - position[1], 2)), 5)
                            # self.get_logger().info(f'e_distance: {e_distance}')
                            if e_distance <= 0.001:  # Euclidean distance less than 1mm to prevent grasping while the object is still moving. 欧式距离小于1mm, 防止物体还在移动时就去夹取了
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
                cv2.drawContours(bgr_image, [self.imgpts], -1, (0, 255, 255), 2, cv2.LINE_AA) # Draw rectangle 绘制矩形
            if bgr_image is not None:
                cv2.imshow('result', bgr_image)
                cv2.waitKey(1)

def main():
    rclpy.init()
    node = TagSorting('tag_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
