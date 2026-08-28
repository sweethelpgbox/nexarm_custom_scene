import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    if compiled == 'True':
        example_package_path = get_package_share_directory('example')
        app_package_path = get_package_share_directory('app')
    else:
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'



    tag_stackup_node = Node(
        package='app',
        executable='tag_stackup',
        output='screen',
        parameters=[{'use_scene_pose': use_scene_pose}],
    )

    return [
            use_scene_pose_arg,
            tag_stackup_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
