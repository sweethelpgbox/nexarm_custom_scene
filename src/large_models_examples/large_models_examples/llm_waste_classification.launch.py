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

    large_models_examples_package_path = get_package_share_directory('large_models_examples')

    # 立即启动 waste_classification_launch
    waste_classification_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_examples_package_path, 'large_models_examples','waste_classification.launch.py')),
    )

    # 在5秒后启动 large_models_launch 和 llm_waste_classification_node
    delayed_launch = TimerAction(
        period=5.0,  # 延迟时间为5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('large_models'), 'launch/start.launch.py')),
                    launch_arguments={
                        'mode': mode,
                        'offline': offline,
                        'camera_topic': camera_topic,
                    }.items(),
                ),
            Node(
                package='large_models_examples',
                executable='llm_waste_classification',
                output='screen',
            ),
        ]
    )

    return [mode_arg,
            waste_classification_launch,  # 立即启动
            delayed_launch,                # 5秒后启动
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