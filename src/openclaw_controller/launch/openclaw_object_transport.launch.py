#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    stream = LaunchConfiguration('stream').perform(context).lower() == 'true'
    feishu_enable = LaunchConfiguration('feishu_enable').perform(context).lower() == 'true'

    openclaw_controller_package_path = get_package_share_directory('openclaw_controller')

    robot_base_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(openclaw_controller_package_path, 'launch/include/device_base.launch.py')
        ]),
    )

    openclaw_object_transport_node = Node(
        package='openclaw_controller',
        executable='openclaw_object_transport',
        output='screen',
    )
    
    
    return [
        robot_base_control_launch,
        openclaw_object_transport_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stream',
            default_value='false',
            description='Enable stream mode for openclaw_object_transport'),
        DeclareLaunchArgument(
            'feishu_enable',
            default_value='false',
            description='Enable feishu message sending'),
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
