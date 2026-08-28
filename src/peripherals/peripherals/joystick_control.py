#!/usr/bin/env python3
# coding=utf8

import os
import sys
import threading
import pygame as pg
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import BuzzerState, ArmCoords

###############################################################################
AXES_MAP =  ('0', '1', '2', '3', 'hat_x', 'hat_y')


BUTTONS = [("cross", "circle", "", "square","triangle", "", "l1",
           "r1", "l2", "r2", "select", "start", "mode","lc","rc"),
           
           ("triangle", "circle", "cross",  "square","l1", "r1", "l2", "r2", 
            "select", "start",  "lc","rc","mode","","")]


class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pg.display.init()

        self.BUTTONS = BUTTONS[0]
        self.last_buttons = dict.fromkeys(self.BUTTONS, 0)

        # Constants
        INIT_X = 200.0; INIT_Y = 0.0; INIT_Z = 200.0
        INIT_PITCH = -90.0; INIT_ROLL = 0.0; INIT_CLAW = 0.0

        # Current arm pose (mm / degrees)
        self.current_pose = {
            'x': INIT_X, 'y': INIT_Y, 'z': INIT_Z,
            'pitch': INIT_PITCH, 'roll': INIT_ROLL, 'claw': INIT_CLAW,
        }

        # 发布器
        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=timer_cb_group)
        self.client.wait_for_service()

        # 手柄
        self.js = None
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))        
        self.last_buttons = [0] * len(BUTTONS[0])

        self.lock = threading.Lock()

        self.create_timer(0.1, self.update_buttons)
        threading.Thread(target=self.connect, daemon=True).start()

        # 初始化姿态
        self.publish_arm(**self.current_pose, time_ms=1000)

    def publish_arm(self, x, y, z, pitch, roll, claw, time_ms):
        msg = ArmCoords()
        msg.x = float(x); msg.y = float(y); msg.z = float(z)
        msg.pitch = float(pitch); msg.roll = float(roll); msg.claw = float(claw)
        msg.time_ms = int(time_ms)
        self.arm_pub.publish(msg)
        


    def get_button_state(self, button):
        if button in self.BUTTONS:
            return self.last_buttons[self.BUTTONS.index(button)]
        return 0

    def run_async(self, func, *args):
        """ 在新线程执行 func """
        threading.Thread(target=func, args=args, daemon=True).start()

    def _move(self, dx=0, dy=0, dz=0, dpitch=0, droll=0, dclaw=0):
        """Incremental move helper (mm / degrees)."""
        self.current_pose['x'] += dx
        self.current_pose['y'] += dy
        self.current_pose['z'] += dz
        self.current_pose['pitch'] += dpitch
        self.current_pose['roll'] += droll
        self.current_pose['claw'] += dclaw
        self.publish_arm(**self.current_pose, time_ms=50)


    def connect(self):
        while True:
            if os.path.exists("/dev/input/js0"):
                with self.lock:
                    if self.js is None:
                        pg.joystick.init()
                        try:
                            self.js = pg.joystick.Joystick(0)
                            self.js.init()
                            
                            # old device
                            if(self.js.get_name() == 'SHANWAN Android Gamepad'):                              
                                self.BUTTONS = BUTTONS[0]
                                self.last_buttons = [0] * len(self.BUTTONS)
                            
                            # new device
                            elif(self.js.get_name() == 'USB WirelessGamepad'):
                                self.BUTTONS = BUTTONS[1]        
                        except Exception as e:
                            print(e)
                            self.js = None
            else:
                with self.lock:
                    if self.js is not None:
                        self.js.quit()
                        self.js = None
            pg.time.delay(200)       
            

    def axes_callback(self, axes):
        step = 5  # mm per tick
        if axes['hat_x'] == 1:      # forward (+X)
            self._move(dx=step)
        elif axes['hat_x'] == -1:   # backward (-X)
            self._move(dx=-step)

        if axes['hat_y'] == 1:      # right (-Y)
            self._move(dy=-step)
        elif axes['hat_y'] == -1:   # left (+Y)
            self._move(dy=step)        
            
        # self.mecanum_pub.publish(twist)
   
    def handle_button_event(self, button_state, pressed):
        """
        处理按键事件：按下和释放
        """
        callback = "".join([self.BUTTONS[button_state], '_callback'])
        if hasattr(self, callback):
            try:
                getattr(self, callback)(pressed)
            except Exception as e:
                self.get_logger().error(str(e))
    
        
    def mode_callback(self, new_state):
        if new_state:
            msg = BuzzerState()
            msg.freq = 2000
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
        

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        if new_state:
            self._move(dz=5)

    def l2_callback(self, new_state):
        if new_state:
            self._move(dz=-5)

    def r1_callback(self, new_state):
        if new_state:
            self._move(dpitch=-2)


    def r2_callback(self, new_state):
        if new_state:
            self._move(dpitch=2)

    def square_callback(self, new_state):
        if new_state:
            self._move(droll=2)


    def circle_callback(self, new_state):
        if new_state:
            self._move(droll=-2)

    def cross_callback(self, new_state):
        if new_state:
            # Open claw (toward -60)
            self.current_pose['claw'] = max(self.current_pose['claw'] - 10, -60.0)
            self.publish_arm(**self.current_pose, time_ms=50)


    def triangle_callback(self, new_state):
        if new_state:
            # Close claw (toward 30)
            self.current_pose['claw'] = min(self.current_pose['claw'] + 10, 30.0)
            self.publish_arm(**self.current_pose, time_ms=50)



    def start_callback(self, new_state):
        if new_state:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
            # Go home
            self.current_pose = {
                'x': 200.0, 'y': 0.0, 'z': 200.0,
                'pitch': -90.0, 'roll': 0.0, 'claw': 0.0,
            }
            self.publish_arm(**self.current_pose, time_ms=1000)  
    
    def update_buttons(self):
        try:
            while True:
                for event in pg.event.get():
                    if event.type == pg.QUIT:

                        sys.exit(0)

                    # 处理手柄摇杆事件
                    elif event.type == pg.JOYAXISMOTION:
                        axis_index = event.axis  # 获取当前轴的索引
                        axis_value = event.value
                        axis_key = AXES_MAP[axis_index]
                        self.last_axes[axis_key] = axis_value
                    # 处理十字键    
                    elif event.type == pg.JOYHATMOTION:
                        hat_y, hat_x = event.value                                              
                        self.last_axes['hat_x'] = hat_x
                        self.last_axes['hat_y'] = hat_y    
                    # 处理按键                                                
                    elif event.type == pg.JOYBUTTONDOWN:                        
                        self.handle_button_event(event.button, True)

                    elif event.type == pg.JOYBUTTONUP:
                        self.handle_button_event(event.button, False)

                    
                    self.axes_callback(self.last_axes)

        except KeyboardInterrupt:
            print("\n程序已手动退出")
            pg.quit()
            sys.exit(0)
            
        

def main():
    node = JoystickController('joystick_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node() 

if __name__ == "__main__":
    main()