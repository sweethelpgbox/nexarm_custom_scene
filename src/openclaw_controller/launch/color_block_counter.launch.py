from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    lab_config_file = LaunchConfiguration('lab_config_file')
    enabled_on_start = LaunchConfiguration('enabled_on_start')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/depth_cam/rgb/image_raw'),
        DeclareLaunchArgument(
            'lab_config_file',
            default_value='/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml',
        ),
        DeclareLaunchArgument('enabled_on_start', default_value='true'),
        Node(
            package='openclaw_controller',
            executable='color_block_counter',
            output='screen',
            parameters=[
                {
                    'image_topic': image_topic,
                    'lab_config_file': lab_config_file,
                    'enabled_on_start': enabled_on_start,
                }
            ],
        ),
    ])
