import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    play_config_path = LaunchConfiguration('play_config_path', default='')
    play_config_path_arg = DeclareLaunchArgument('play_config_path', default_value=play_config_path)

    if compiled == 'True':
        app_package_path = get_package_share_directory('app')
    else:
        app_package_path = '/home/ubuntu/ros2_ws/src/app'

    scene4_waste_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/waste_classification.launch.py')),
        launch_arguments={
            'model_name': 'best_garbage_11',
            'use_scene_pose': use_scene_pose,
            'play_config_path': play_config_path,
        }.items(),
    )

    return [
        use_scene_pose_arg,
        play_config_path_arg,
        scene4_waste_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
