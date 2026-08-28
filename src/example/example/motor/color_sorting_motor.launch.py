import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']

    function_value  = LaunchConfiguration('function', default='pull').perform(context)
    function_arg = DeclareLaunchArgument('function', default_value=function_value)
    play_config_path = LaunchConfiguration('play_config_path', default='/home/ubuntu/ros2_ws/src/example/example/motor/plays/scene5_dual_arm.yaml')
    play_config_path_arg = DeclareLaunchArgument('play_config_path', default_value=play_config_path)

    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/nexarm.launch.py')),
    )
    
    color_sorting_node = Node(
        package='example',
        executable='color_sorting_motor',
        output='screen',
        parameters=[{'function': function_value, 'play_config_path': play_config_path}]
    )

    return [function_arg,
            play_config_path_arg,
            # sdk_launch,
            # depth_camera_launch,
            color_sorting_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
