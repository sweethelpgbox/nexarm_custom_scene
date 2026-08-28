import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from app import scene_play_registry


def source_package_path(package_name, compiled):
    if compiled == 'True':
        return get_package_share_directory(package_name)
    if package_name == 'example':
        return '/home/ubuntu/ros2_ws/src/example'
    return '/home/ubuntu/ros2_ws/src/app'


def launch_file_path(package_name, launch_name, package_path, compiled):
    if package_name == 'example' and compiled != 'True':
        return os.path.join(package_path, 'example/motor', launch_name)
    return os.path.join(package_path, 'launch', launch_name)


def play_config_path(entry, package_paths, compiled):
    filename = entry.get('play_config')
    if not filename:
        return ''
    package_name = entry.get('play_config_package', 'app')
    package_path = package_paths[package_name]
    if package_name == 'example':
        if compiled == 'True':
            return os.path.join(package_path, 'config', 'plays', filename)
        return os.path.join(package_path, 'example', 'motor', 'plays', filename)
    return os.path.join(package_path, 'config', 'plays', filename)


def include_play_launches(entry, package_paths, compiled, use_scene_pose):
    actions = []
    launch_entries = entry.get('launches')
    if not launch_entries:
        package_name = entry.get('package')
        launch_name = entry.get('launch')
        launch_entries = [{"package": package_name, "launch": launch_name}] if package_name and launch_name else []

    play_config = play_config_path(entry, package_paths, compiled)
    for launch_entry in launch_entries:
        package_name = launch_entry.get('package')
        launch_name = launch_entry.get('launch')
        if not package_name or not launch_name:
            continue
        package_path = package_paths[package_name]
        launch_arguments = {
            'use_scene_pose': use_scene_pose,
            'play_config_path': play_config,
        }
        extra_arguments = launch_entry.get('arguments', {})
        if isinstance(extra_arguments, dict):
            launch_arguments.update(extra_arguments)
        if entry.get('play_id') == 'dual_arm_conveyor':
            launch_arguments.update({
                'controller_prefix': LaunchConfiguration('controller_prefix'),
                'camera_prefix': LaunchConfiguration('camera_prefix'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            })
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_file_path(package_name, launch_name, package_path, compiled)
                ),
                launch_arguments=launch_arguments.items(),
            )
        )
    return actions


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    active_scene = scene_play_registry.active_scene_id(default_scene=scene_play_registry.SCENE0_ID)
    entry = scene_play_registry.play_for_scene(active_scene)
    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    controller_prefix_arg = DeclareLaunchArgument('controller_prefix', default_value='/ros_robot_controller')
    camera_prefix_arg = DeclareLaunchArgument('camera_prefix', default_value='/depth_cam')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/depth_cam/rgb/image_raw')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/depth_cam/rgb/camera_info')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/depth_cam/depth/image_raw')
    depth_info_topic_arg = DeclareLaunchArgument('depth_info_topic', default_value='/depth_cam/depth/camera_info')
    calibration_enabled_arg = DeclareLaunchArgument('calibration_enabled', default_value='true')
    package_paths = {
        'app': source_package_path('app', compiled),
        'example': source_package_path('example', compiled),
    }

    lab_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/lab_manager.launch.py')),
    )
    calibration_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/calibration_node.launch.py')),
        launch_arguments={
            'calibration_enabled': LaunchConfiguration('calibration_enabled'),
        }.items(),
    )

    object_tracking_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/object_tracking.launch.py')),
    )

    finger_trace_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/finger_trace.launch.py')),
    )

    object_sorting_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/object_sorting.launch.py')),
    )

    shape_recognition_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/shape_recognition.launch.py')),
    )

    waste_classification_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_paths['app'], 'launch/waste_classification.launch.py')),
    )

    task_launches = include_play_launches(entry, package_paths, compiled, use_scene_pose)

    

    return [
        use_scene_pose_arg,
        controller_prefix_arg,
        camera_prefix_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        calibration_enabled_arg,
        lab_manager_launch,
        calibration_node_launch,
        finger_trace_node_launch,
        object_tracking_node_launch,
        object_sorting_node_launch,
        shape_recognition_node_launch,
        waste_classification_node_launch,
        *task_launches,
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
