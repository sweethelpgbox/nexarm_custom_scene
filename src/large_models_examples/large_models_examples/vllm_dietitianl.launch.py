import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction

def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    offline = LaunchConfiguration('offline', default='false').perform(context)
    offline_arg = DeclareLaunchArgument('offline', default_value=offline)
    interruption = LaunchConfiguration('interruption', default=False)
    interruption_arg = DeclareLaunchArgument('interruption', default_value=interruption)

    peripherals_package_path = get_package_share_directory('peripherals')
    sdk_package_path = get_package_share_directory('sdk')
    large_models_package_path = get_package_share_directory('large_models')

    # 立即启动 depth_camera_launch
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/nexarm.launch.py')),
    )

    # 在5秒后启动 sdk_launch、large_models_launch、intelligent_grasp_node 和 vllm_dietitianl_node
    delayed_launch = TimerAction(
        period=5.0,  # 延迟时间为5秒
        actions=[            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(large_models_package_path, 'launch/start.launch.py')),
                    launch_arguments={
                        'mode': mode,
                        'offline': offline,
                        'camera_topic': camera_topic,
                    }.items(),
                ),
            Node(
                package='large_models_examples',
                executable='intelligent_grasp',
                output='screen',
            ),
            Node(
                package='large_models_examples',
                executable='vllm_dietitianl',
                output='screen',
            ),
        ]
    )

    return [mode_arg,
            depth_camera_launch,  # 立即启动
            control_launch,
            delayed_launch,        # 5秒后启动其他部分
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
