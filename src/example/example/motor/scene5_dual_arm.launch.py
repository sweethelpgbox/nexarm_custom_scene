import os

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sdk.scene_context import load_scene_environment


DEFAULT_ARM_A_PREFIX = '/arm_a/ros_robot_controller'
DEFAULT_ARM_B_PREFIX = '/arm_b/ros_robot_controller'
GARBAGE_NAMES = [
    'BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks',
    'Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate',
    'StorageBattery', 'Toothbrush', 'Umbrella',
]


def scene5_env():
    return load_scene_environment()


def scene5_arm_role():
    return scene5_env().arm_role


def default_camera_topic():
    env = scene5_env()
    if env.is_scene5:
        return env.camera_topic('rgb/image_raw')
    return '/usb_cam/image_raw' if 'Pro' in os.environ.get('MACHINE_TYPE', '') else '/depth_cam/rgb/image_raw'


def default_camera_info_topic():
    return scene5_env().camera_topic('rgb/camera_info')


def default_depth_topic():
    return scene5_env().camera_topic('depth/image_raw')


def default_depth_info_topic():
    return scene5_env().camera_topic('depth/camera_info')


def scene5_role_namespace():
    return scene5_env().role_namespace.strip('/')


def default_conveyor_topic():
    return f'{DEFAULT_ARM_B_PREFIX}/conveyor/set'


def launch_setup(context):
    scene_env = scene5_env()
    role_namespace = scene_env.role_namespace.strip('/')
    config_path = LaunchConfiguration('config_path')
    play_config_path = LaunchConfiguration('play_config_path')
    arm_a_prefix = LaunchConfiguration('arm_a_prefix')
    arm_b_prefix = LaunchConfiguration('arm_b_prefix')
    conveyor_topic = LaunchConfiguration('conveyor_topic')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')

    arm_a_loader = Node(
        package='example',
        executable='scene5_arm_a_loader',
        namespace=role_namespace,
        output='screen',
        parameters=[
            {'config_path': config_path},
            {'play_config_path': play_config_path},
            {'arm_a_prefix': arm_a_prefix},
            {'image_topic': camera_topic},
            {'camera_info_topic': camera_info_topic},
            {'service_prefix': scene_env.role_namespace},
            {'object_topic': scene_env.topic('scene5_arm_a_yolo/yolo/object_detect')},
            {'yolo_start_service': scene_env.topic('scene5_arm_a_yolo/start')},
            {'yolo_stop_service': scene_env.topic('scene5_arm_a_yolo/stop')},
            {'yolo_box_format': 'center_wh'},
        ],
    )
    arm_a_yolo_node = Node(
        package='example',
        executable='yolo_node',
        namespace=f'{role_namespace}/scene5_arm_a_yolo' if role_namespace else 'scene5_arm_a_yolo',
        output='screen',
        parameters=[{
            'classes': GARBAGE_NAMES,
            'model': 'best_garbage_11',
            'engine': 'best_garbage_11',
            'conf': 0.8,
            'task': 'obb',
            'display': False,
            'image_topic': camera_topic,
            'start_service': scene_env.topic('scene5_arm_a_yolo/start'),
            'stop_service': scene_env.topic('scene5_arm_a_yolo/stop'),
        }],
    )
    arm_b_worker = Node(
        package='example',
        executable='waste_classification_motor_depth',
        namespace=role_namespace,
        output='screen',
        parameters=[
            {'start': False},
            {'display': False},
            {'config_path': config_path},
            {'play_config_path': play_config_path},
            {'controller_prefix': arm_b_prefix},
            {'control_conveyor': False},
            {'service_prefix': scene_env.role_namespace},
            {'object_topic': scene_env.topic('yolo/yolo/object_detect')},
            {'yolo_start_service': scene_env.topic('yolo/start')},
            {'yolo_stop_service': scene_env.topic('yolo/stop')},
            {'rgb_topic': camera_topic},
            {'depth_topic': depth_topic},
            {'depth_info_topic': depth_info_topic},
        ],
    )
    coordinator = Node(
        package='example',
        executable='scene5_coordinator',
        output='screen',
        parameters=[
            {'config_path': config_path},
            {'play_config_path': play_config_path},
            {'conveyor_topic': conveyor_topic.perform(context)},
            {'service_prefix': scene_env.role_namespace},
            {'arm_a_service_prefix': DEFAULT_ARM_A_PREFIX.rsplit('/ros_robot_controller', 1)[0]},
            {'arm_b_service_prefix': DEFAULT_ARM_B_PREFIX.rsplit('/ros_robot_controller', 1)[0]},
        ],
    )
    yolo_node = Node(
        package='example',
        executable='yolo_node',
        namespace=f'{role_namespace}/yolo' if role_namespace else 'yolo',
        output='screen',
        parameters=[{
            'classes': GARBAGE_NAMES,
            'model': 'best_garbage_26',
            'engine': 'best_garbage_26',
            'conf': 0.8,
            'task': 'obb',
            'display': False,
            'image_topic': camera_topic,
            'start_service': scene_env.topic('yolo/start'),
            'stop_service': scene_env.topic('yolo/stop'),
        }],
    )

    if scene_env.arm_role == 'B':
        return [yolo_node, arm_b_worker, coordinator]
    return [arm_a_yolo_node, arm_a_loader, coordinator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value='/home/ubuntu/ros2_ws/src/app/config/calibration_scene.yaml'),
        DeclareLaunchArgument('play_config_path', default_value='/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml'),
        DeclareLaunchArgument('arm_a_prefix', default_value=DEFAULT_ARM_A_PREFIX),
        DeclareLaunchArgument('arm_b_prefix', default_value=DEFAULT_ARM_B_PREFIX),
        DeclareLaunchArgument('conveyor_topic', default_value=default_conveyor_topic()),
        DeclareLaunchArgument('camera_topic', default_value=default_camera_topic()),
        DeclareLaunchArgument('camera_info_topic', default_value=default_camera_info_topic()),
        DeclareLaunchArgument('depth_topic', default_value=default_depth_topic()),
        DeclareLaunchArgument('depth_info_topic', default_value=default_depth_info_topic()),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
