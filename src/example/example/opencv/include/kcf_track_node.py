#!/usr/bin/env python3
# encoding: utf-8
# 标签追踪
import cv2
import time
import rclpy
import queue
import signal
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetString
from ros_robot_controller_msgs.msg import ArmCoords
from example.scene_pose import load_scene_home_pose

class KcfTrackNode(Node):
    # Constants (mm coords)
    INIT_HOME = load_scene_home_pose()
    INIT_X = INIT_HOME['x']
    INIT_Y = INIT_HOME['y']
    INIT_Z = INIT_HOME['z']
    INIT_PITCH = 0.0
    INIT_ROLL = INIT_HOME['roll']
    INIT_CLAW = INIT_HOME['claw']

    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.z_dis = float(self.INIT_Z)   # mm (was 0.15m)
        self.y_dis = 0.0     # mm (was 500 pulse = center)
        self.x_init = float(self.INIT_X)  # mm (was 0.16m)
        self.bridge = CvBridge()
        self.center = None
        self.tracker = None
        self.enable_select = False
        self.running = True
        self.start = False
        self.name = name
        self.image_queue = queue.Queue(maxsize=2)
        self.pid_z = pid.PID(0.1, 0.0, 0.0)    # output in mm now (was 0.0001 for meters)
        self.pid_y = pid.PID(0.045, 0.0, 0.0)   # output in mm now (was 0.09 for pulse)
        self.fps = fps.FPS()

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        # 等底层驱动就绪再发指令，避免消息丢失
        client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.get_logger().info('Waiting for controller_manager...')
        client.wait_for_service()
        self.get_logger().info('Controller ready')
        self.destroy_client(client)

        self.init_action()
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.publish_arm(self.x_init, 0.0, self.z_dis, self.INIT_PITCH, self.INIT_ROLL, self.INIT_CLAW, 1500)
        time.sleep(1.8)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)


    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start kcf track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop kcf track")
        self.start = False
        res = self.send_request(ColorDetect.Request())
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set kcf success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set kcf fail')
        response.success = True
        response.message = "stop"
        return response
    
    def image_callback(self, ros_image):
        # 将画面转为 opencv 格式(convert the screen to opencv format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)

        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(bgr_image)


    def main(self):
        while self.running:
            rgb_image = self.image_queue.get()
            result_image = np.copy(rgb_image)
            factor = 4
            rgb_image = cv2.resize(rgb_image, (int(result_image.shape[1]/ factor), int(result_image.shape[0]/ factor)))

            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("result", result_image, False)
                    roi =  tuple(int(i / factor)for i in roi)
                    if roi:
                        param = cv2.TrackerKCF.Params()
                        param.detect_thresh = 0.2
                        self.tracker = cv2.TrackerKCF_create(param)
                        self.tracker.init(rgb_image, roi)
            else:
                status, box = self.tracker.update(rgb_image)
                if status:
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)
                    center_x, center_y = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
           

                    t1 = time.time()
                    self.pid_y.SetPoint = result_image.shape[1]/2 
                    self.pid_y.update(center_x)
                    self.y_dis += self.pid_y.output
                    if self.y_dis < -450:
                        self.y_dis = -450
                    if self.y_dis > 450:
                        self.y_dis = 450

                    self.pid_z.SetPoint = result_image.shape[0]/2 
                    self.pid_z.update(center_y)
                    self.z_dis += self.pid_z.output
                    if self.z_dis > 450:
                        self.z_dis = 450
                    if self.z_dis < 160:
                        self.z_dis = 160
                    self.publish_arm(self.x_init, self.y_dis, self.z_dis, self.INIT_PITCH, self.INIT_ROLL, self.INIT_CLAW, 80)
                    t2 = time.time()
                    t = t2 - t1
                    if t < 0.02:
                        time.sleep(0.02 - t)
                else:
                    time.sleep(0.01)
            
            self.fps.update()
            if self.get_parameter('display').value:
                self.fps.show_fps(result_image)
                cv2.imshow("result", result_image)

            key = cv2.waitKey(1)
            if key == ord('s'): # 按下s开始选择追踪目标(press 's' to start selecting the tracking target)
                self.get_logger().info("在画面窗口按下s开始选择追踪目标,然后按下空格开始追踪")
                self.tracker = None
                self.enable_select = True

        self.init_action()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = KcfTrackNode('kcf_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
