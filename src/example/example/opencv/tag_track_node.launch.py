import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    start = LaunchConfiguration('start', default='true')
    tag_id = LaunchConfiguration('tag_id', default='-1')
    display = LaunchConfiguration('display', default='true')
    image_topic = LaunchConfiguration('image_topic', default='/depth_cam/rgb/image_raw')
    scene = LaunchConfiguration('scene', default='scene_0')
    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    tag_id_arg = DeclareLaunchArgument('tag_id', default_value=tag_id)
    display_arg = DeclareLaunchArgument('display', default_value=display)
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value=image_topic)
    scene_arg = DeclareLaunchArgument('scene', default_value='scene_0')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        peripherals_package_path = get_package_share_directory('peripherals')
        example_package_path = get_package_share_directory('example')
    else:
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/nexarm.launch.py')),
    )

    
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    tag_track_node = Node(
        package='example',
        executable='tag_track',
        output='screen',
        parameters=[{
            'start': start,
            'tag_id': tag_id,
            'display': display,
            'image_topic': image_topic,
            'scene': scene,
            'broadcast': 'true',
            'use_scene_pose': use_scene_pose,
        }]
    )

    return [start_arg,
            tag_id_arg,
            display_arg,
            image_topic_arg,
            scene_arg,
            use_scene_pose_arg,
            sdk_launch,
            depth_camera_launch,
            tag_track_node,
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
