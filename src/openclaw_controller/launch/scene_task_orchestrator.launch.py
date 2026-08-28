import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    launch_depth_camera = LaunchConfiguration('launch_depth_camera', default='true')
    launch_nexarm = LaunchConfiguration('launch_nexarm', default='true')
    launch_object_sorting = LaunchConfiguration('launch_object_sorting', default='true')
    launch_object_transport = LaunchConfiguration('launch_object_transport', default='true')
    runtime_delay = LaunchConfiguration('runtime_delay', default='3.0')

    args = [
        DeclareLaunchArgument('launch_depth_camera', default_value=launch_depth_camera),
        DeclareLaunchArgument('launch_nexarm', default_value=launch_nexarm),
        DeclareLaunchArgument('launch_object_sorting', default_value=launch_object_sorting),
        DeclareLaunchArgument('launch_object_transport', default_value=launch_object_transport),
        DeclareLaunchArgument('runtime_delay', default_value=runtime_delay),
    ]

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        sdk_package_path = get_package_share_directory('sdk')
        openclaw_controller_package_path = get_package_share_directory('openclaw_controller')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        openclaw_controller_package_path = '/home/ubuntu/ros2_ws/src/openclaw_controller'

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')
        ),
        condition=IfCondition(launch_depth_camera),
    )

    nexarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/nexarm.launch.py')
        ),
        condition=IfCondition(launch_nexarm),
    )

    orchestrator_config = os.path.join(openclaw_controller_package_path, 'config', 'scene_task_orchestrator.yaml')

    delayed_runtime = TimerAction(
        period=float(runtime_delay.perform(context)),
        actions=[
            Node(
                package='app',
                executable='object_sorting',
                output='screen',
                condition=IfCondition(launch_object_sorting),
            ),
            Node(
                package='large_models',
                executable='object_transport',
                output='screen',
                condition=IfCondition(launch_object_transport),
            ),
            Node(
                package='openclaw_controller',
                executable='scene_task_orchestrator',
                output='screen',
                parameters=[orchestrator_config],
            ),
        ],
    )

    return args + [
        depth_camera_launch,
        nexarm_launch,
        delayed_runtime,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
