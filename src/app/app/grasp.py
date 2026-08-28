import time
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import ArmCoords
from app.play_pose import get_use_scene_pose


class GraspNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.grasp = None
        self.chassis_type = os.environ.get('CHASSIS_TYPE', '')
        if self.chassis_type == 'Slide_Rails':
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/example/example/stepper/config/calibration_scene.yaml"
        else:
            self.scene_config_path = "/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml"
        self.home_pose = self._load_home_pose_from_scene()
        self.known_pose = {
            'x': float(self.home_pose['x']),
            'y': float(self.home_pose['y']),
            'z': float(self.home_pose['z']),
            'pitch': float(self.home_pose['pitch']),
            'roll': float(self.home_pose['roll']),
        }

        self.arm_pub = self.create_publisher(ArmCoords, '/ros_robot_controller/arm/set_coords', 5)
        # self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        # self.kinematics_client.wait_for_service()
        self.kinematics_client = None

        # NOTE: Grasp message type from servo_controller_msgs no longer exists
        # self.create_subscription(Grasp, '/grasp', self.grasp_callback, 1)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def _load_scene_config(self):
        try:
            with open(self.scene_config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
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
        self.known_pose = {'x': float(x), 'y': float(y), 'z': float(z), 'pitch': float(pitch), 'roll': float(roll)}

    def grasp_callback(self, grasp):
        self.set_target(grasp)  # 调用设置目标方法
        if grasp.mode == 'pick':
            self.execute_pick_sequence()
        elif grasp.mode == 'place':
            self.execute_place_sequence()

    def set_target(self, grasp):
        model_name = grasp.mode
        pose_t = grasp.position
        pick_pitch = grasp.pitch
        self.angle = grasp.angle
        self.grasp = grasp

        # Above target (+50mm)
        x_mm = pose_t[0] * 1000.0
        y_mm = pose_t[1] * 1000.0
        z_above = (pose_t[2] + 0.05) * 1000.0
        z_mm = pose_t[2] * 1000.0

        self.target1 = [x_mm, y_mm, z_above, float(pick_pitch)]
        self.target2 = [x_mm, y_mm, z_mm, float(pick_pitch)]
        self.target3 = [x_mm, y_mm, z_above, float(pick_pitch)]


    def execute_pick_sequence(self):
        # 处理夹取的动作序列
        queue_list = [
            [self.move_toward, 1],
            [self.move_approach, 1],
            [self.gripper_align, 0.5],
            [self.move_target, 1],
            [self.gripper_move, 0.3],
            [self.move_retreat, 0.5],
            [self.move_toward_init, 1]
        ]
        self.execute_queue(queue_list)

    def execute_place_sequence(self):
        # 处理放置的动作序列
        queue_list = [
            [self.move_toward, 1],
            [self.move_approach, 1],
            [self.move_target, 1],
            [self.gripper_move_, 0.05],
            [self.gripper_align, 0.5],
            [self.gripper_move, 0.3],
            [self.move_retreat, 1],
            [self.move_toward_init, 1],
            [self.move_init, 1]
        ]
        self.execute_queue(queue_list)

    def execute_queue(self, queue_list):
        for action, duration in queue_list:
            action(duration)    

    def move_toward(self, t=1): 
        # 移到朝向物体(move towards the object's direction)
        if self.target2 is not None:
            self.publish_arm(self.target2[0], self.target2[1], self.target2[2], self.target2[3], 0.0, -60.0, int(t * 1000))
            time.sleep(t)

    def move_approach(self, t=1.5): 
        # 移到物体上方(move above the object)
        if self.target1 is not None:
            self.publish_arm(self.target1[0], self.target1[1], self.target1[2], self.target1[3], 0.0, -60.0, int(t * 1000))
            time.sleep(t + 0.1)
    
    def move_target(self, t=1.8):
        # 移到目标位置(move to the target position)
        if self.target2 is not None:
            self.publish_arm(self.target2[0], self.target2[1], self.target2[2], self.target2[3], 0.0, -60.0, int(t * 1000))
            time.sleep(t + 0.5)

    def gripper_move_(self, t=0.05):
        # 夹持器开合一点点(the gripper opens or closes slightly)
        # gripper_control.set_grasp(self.grasp_pub, t, self.grasp.pre_grasp_posture - 30)
        time.sleep(t + 0.1)

    def gripper_align(self, t=0.5):
        # 夹持器对齐(the gripper aligns)
        # Roll angle from self.angle + target yaw
        roll_deg = float(self.angle) if self.angle is not None else 0.0
        p = self.known_pose
        self.publish_arm(p['x'], p['y'], p['z'], p['pitch'], roll_deg, -60.0, int(t * 1000))
        time.sleep(t + 0.3)

    def gripper_move(self, t=0.5):
        # 夹持器开合(the gripper opens and closes)
        claw = float(self.grasp.grasp_posture) if self.grasp else 30.0
        p = self.known_pose
        self.publish_arm(p['x'], p['y'], p['z'], p['pitch'], p['roll'], claw, int(t * 1000))
        time.sleep(t + 0.3)
    
    def move_retreat(self, t=1):
        # 远离物体(move away from the object)
        if self.target3 is not None:
            self.publish_arm(self.target3[0], self.target3[1], self.target3[2], self.target3[3], 0.0, self.known_pose.get('claw', 30.0), int(t * 1000))
            time.sleep(t)
        
    
    def move_toward_init(self, t=1): 
        # 移到朝向物体(move towards the object's direction)
        self.home_pose = self._load_home_pose_from_scene()
        hp = self.home_pose
        self.publish_arm(hp['x'], hp['y'], hp['z'], hp['pitch'], hp['roll'], hp['claw'], int(t * 1000))

    def move_init(self, t=1): 
        pass
        #set_servo_position(self.joints_pub, t, ((1, 500),))
        #rospy.sleep(t/1000.0)  
def main():
    rclpy.init()
    node = GraspNode('grasp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
