#!/usr/bin/env python3
# encoding: utf-8
# 深度图转换点云
import time
import cv2
import rclpy
import queue
import signal
import threading
import numpy as np
import open3d as o3d
import message_filters
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from example.scene_pose import load_scene_home_pose
from rclpy.callback_groups import ReentrantCallbackGroup


class TrackObjectNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        signal.signal(signal.SIGINT, self.shutdown)
        self.scale = 4
        self.proc_size = [int(640 / self.scale), int(480 / self.scale)]
        self.haved_add = False
        self.display = self.get_bool_param('display', True)
        self.running = True
        self.pc_queue = queue.Queue(maxsize=1)
        self.target_cloud = o3d.geometry.PointCloud()
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        timer_cb_group = ReentrantCallbackGroup()
        camera_name = 'depth_cam'
        rgb_sub = message_filters.Subscriber(self, Image, f'/{camera_name}/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, f'/{camera_name}/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, f'/{camera_name}/depth/camera_info')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.2)
        sync.registerCallback(self.multi_callback)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_bool_param(self, name, default=False):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return bool(value)
        except Exception:
            return default

    def wait_for_motion_ready(self):
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.2)

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        home = load_scene_home_pose()
        self.publish_arm(home['x'], home['y'], home['z'], 0.0, home['roll'], home['claw'], 1200)
        time.sleep(1.2)
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'rgb_depth_to_pointcloud ready')

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

    def get_node_state(self, request, response):
        response.success = True
        return response

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            rgb_image = cv2.resize(rgb_image, tuple(self.proc_size), interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.resize(depth_image, tuple(self.proc_size), interpolation=cv2.INTER_NEAREST)

            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                int(depth_camera_info.width / self.scale),
                int(depth_camera_info.height / self.scale),
                float(depth_camera_info.k[0] / self.scale),
                float(depth_camera_info.k[4] / self.scale),
                float(depth_camera_info.k[2] / self.scale),
                float(depth_camera_info.k[5] / self.scale),
            )

            o3d_image_rgb = o3d.geometry.Image(rgb_image)
            o3d_image_depth = o3d.geometry.Image(np.ascontiguousarray(depth_image))
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d_image_rgb,
                o3d_image_depth,
                convert_rgb_to_intensity=False,
            )
            pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            _, inliers = pc.segment_plane(distance_threshold=0.05, ransac_n=10, num_iterations=50)
            inlier_cloud = pc.select_by_index(inliers, invert=True)
            display_cloud = o3d.geometry.PointCloud(inlier_cloud)
            display_cloud.transform(np.asarray([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], dtype=np.float64))
            self.target_cloud = display_cloud
            try:
                self.pc_queue.put_nowait(display_cloud)
            except queue.Full:
                pass
        except BaseException as e:
            self.get_logger().info('callback error: ' + str(e))

    def shutdown(self, signum=None, frame=None):
        self.running = False

    def main(self):
        vis = None
        if self.display:
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='point cloud', width=640, height=400, visible=1)
        while self.running:
            if not self.haved_add:
                if self.display:
                    try:
                        point_cloud = self.pc_queue.get(block=True, timeout=2)
                    except queue.Empty:
                        continue
                    vis.add_geometry(point_cloud)
                self.haved_add = True

            if self.haved_add:
                try:
                    point_cloud = self.pc_queue.get(block=True, timeout=2)
                except queue.Empty:
                    continue
                points = np.asarray(point_cloud.points)
                if len(points) > 0 and self.display:
                    vis.update_geometry(point_cloud)
                    vis.poll_events()
                    vis.update_renderer()
            else:
                time.sleep(0.01)
        if vis is not None:
            vis.destroy_window()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrackObjectNode('track_object')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
