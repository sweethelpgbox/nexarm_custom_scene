#!/usr/bin/python3
# coding=utf8
# 距离测量
import cv2
import time
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import message_filters
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from example.scene_pose import load_scene_home_pose
from rclpy.callback_groups import ReentrantCallbackGroup


class DistanceMeasureNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.fps = fps.FPS()
        self.display = self.get_bool_param('display', True)
        self.image_queue = queue.Queue(maxsize=2)
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.controller_init_client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.kinematics_init_client = self.create_client(Trigger, '/kinematics/init_finish')

        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 4, 0.5)
        sync.registerCallback(self.multi_callback)

        self.target_point = None
        self.last_event = 0
        if self.display:
            cv2.namedWindow('depth')
            cv2.setMouseCallback('depth', self.click_callback)

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

    def wait_for_motion_ready(self):
        self.controller_init_client.wait_for_service()
        self.kinematics_init_client.wait_for_service()
        while self.arm_pub.get_subscription_count() == 0:
            time.sleep(0.2)

    def init_process(self):
        self.timer.cancel()
        self.wait_for_motion_ready()
        home = load_scene_home_pose()
        self.publish_arm(home['x'], home['y'], home['z'], 0.0, home['roll'], home['claw'], 1500)
        time.sleep(1.5)
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'distance_measure ready')

    def get_node_state(self, request, response):
        response.success = True
        return response

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

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((ros_rgb_image, ros_depth_image))

    def click_callback(self, event, x, y, flags, params):
        if event in (cv2.EVENT_RBUTTONDOWN, cv2.EVENT_MBUTTONDOWN, cv2.EVENT_LBUTTONDBLCLK):
            self.target_point = None
        if event == cv2.EVENT_LBUTTONDOWN and self.last_event != cv2.EVENT_LBUTTONDBLCLK:
            self.target_point = (x - 640, y) if x >= 640 else (x, y)
        self.last_event = event

    def main(self):
        while self.running:
            try:
                ros_rgb_image, ros_depth_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

                h, w = depth_image.shape[:2]
                depth = np.copy(depth_image).reshape((-1,))
                depth[depth <= 0] = 55555
                min_index = int(np.argmin(depth))
                min_y = min_index // w
                min_x = min_index - min_y * w
                if self.target_point is not None:
                    min_x, min_y = self.target_point
                    min_x = int(np.clip(min_x, 0, w - 1))
                    min_y = int(np.clip(min_y, 0, h - 1))

                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000.0 * 255.0
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                txt = 'Dist: {}mm'.format(int(depth_image[min_y, min_x]))

                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(depth_color_map, txt, (11, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(depth_color_map, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                bgr_image = np.copy(rgb_image)
                cv2.circle(bgr_image, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                cv2.circle(bgr_image, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(bgr_image, txt, (11, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(bgr_image, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                self.fps.update()
                result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                result_image = self.fps.show_fps(result_image)

                if self.display:
                    cv2.imshow('depth', result_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        self.running = False
            except Exception as e:
                self.get_logger().info('error: ' + str(e))

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DistanceMeasureNode('distance_measure')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == '__main__':
    main()
