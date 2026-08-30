import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# scene_6 — custom_object_sorting.
#
# Launches the shared `example` YOLO detector (same node used by
# waste_classification.launch.py) against the "strawberry shortcake ice
# cream bar" class, plus the new custom_object_sorting behavior node.
#
# IMPORTANT: this file's own launch argument names are namespaced
# (scene6_*) rather than reusing waste_classification.launch.py's names
# (model_name/conf/model_size/camera_topic). ROS 2 launch arguments are
# global to the whole launch tree by name -- the *first*
# DeclareLaunchArgument for a given name wins, and later re-declarations
# of the same name elsewhere are silently ignored. Since
# waste_classification.launch.py is one of start_app.launch.py's
# always-on baseline includes (evaluated before any scene-specific
# include), reusing its argument names here meant its defaults silently
# won over ours for every one of them.
#
# Model format: this yolo_node.py loads OpenVINO model directories
# (models/<name>_openvino_model/ or models/v11|26/<name>_openvino_model/
# for names containing "11"/"26"), NOT TensorRT .engine files.


def launch_setup(context):
    camera_topic = LaunchConfiguration('scene6_camera_topic', default='/depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('scene6_camera_topic', default_value=camera_topic)

    model_name = LaunchConfiguration('scene6_model_name', default='strawberry_shortcake_ice_cream_bar').perform(context)
    model_name_arg = DeclareLaunchArgument('scene6_model_name', default_value=model_name)

    conf = LaunchConfiguration('scene6_conf', default=0.8).perform(context)
    conf_arg = DeclareLaunchArgument('scene6_conf', default_value=conf)

    # Must match the imgsz used to train/export the model (yolo export
    # ... imgsz=640) -- a mismatch here causes an OpenVINO input-tensor
    # shape error at inference time, confirmed on hardware.
    model_size = LaunchConfiguration('scene6_model_size', default=640).perform(context)
    model_size_arg = DeclareLaunchArgument('scene6_model_size', default_value=model_size)

    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    play_config_path = LaunchConfiguration('play_config_path', default='')
    play_config_path_arg = DeclareLaunchArgument('play_config_path', default_value=play_config_path)

    # Explicit unique node name: the shared yolo_node executable defaults to
    # node name 'yolo', which collides with waste_classification.launch.py's
    # always-on yolo_node instance (see start_app.launch.py) -- without this,
    # both instances fight over the same /yolo/... topics and services.
    yolo_node = Node(
        package='example',
        executable='yolo_node',
        name='strawberry_shortcake_detect',
        output='screen',
        parameters=[{
            'classes': ['strawberry shortcake ice cream bar'],
            'model': model_name,
            'conf': conf,
            'task': 'detect',
            'display': False,
            'image_topic': camera_topic,
            'model_size': model_size,
            # start_service/stop_service default to the absolute path
            # '/yolo/start'/'/yolo/stop' regardless of node name -- must be
            # overridden explicitly to avoid the same collision.
            'start_service': '/strawberry_shortcake_detect/start',
            'stop_service': '/strawberry_shortcake_detect/stop',
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
