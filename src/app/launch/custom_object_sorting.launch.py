import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# scene_6 — custom_object_sorting.
#
# Launches the shared `example` YOLO detector (same node used by
# waste_classification.launch.py) against the "strawberry shortcake ice
# cream bar" class, plus the new custom_object_sorting behavior node. Swap
# `model_name` for a real trained TensorRT engine at
# src/example/example/yolo_detect/models/<model_name>.engine
# (or models/v11/<model_name>.engine / models/26/<model_name>.engine if the
# name contains "11"/"26", matching the existing model layout) before this
# scene will actually detect anything.


def launch_setup(context):
    camera_topic = LaunchConfiguration('camera_topic', default='/depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    model_name = LaunchConfiguration('model_name', default='strawberry_shortcake_ice_cream_bar').perform(context)
    model_name_arg = DeclareLaunchArgument('model_name', default_value=model_name)

    conf = LaunchConfiguration('conf', default=0.6).perform(context)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)

    model_size = LaunchConfiguration('model_size', default=320).perform(context)
    model_size_arg = DeclareLaunchArgument('model_size', default_value=model_size)

    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    play_config_path = LaunchConfiguration('play_config_path', default='')
    play_config_path_arg = DeclareLaunchArgument('play_config_path', default_value=play_config_path)

    yolo_node = Node(
        package='example',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'classes': ['strawberry shortcake ice cream bar'],
            'model': model_name,
            'engine': model_name,
            'conf': conf,
            'task': 'detect',
            'display': False,
            'image_topic': camera_topic,
            'model_size': model_size,
        }]
    )

    custom_object_sorting_node = Node(
        package='app',
        executable='custom_object_sorting',
        output='screen',
        parameters=[{
            'start': False,
            'use_scene_pose': use_scene_pose,
            'play_config_path': play_config_path,
        }],
    )

    return [
        conf_arg,
        model_name_arg,
        model_size_arg,
        camera_topic_arg,
        use_scene_pose_arg,
        play_config_path_arg,
        yolo_node,
        custom_object_sorting_node,
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
