import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # noqa: E402
from launch import LaunchDescription, LaunchService  # noqa: E402
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
    camera_name = LaunchConfiguration('camera_name')
    def camera_topic(suffix):
        return ['/', camera_name, '/', suffix]

    camera_nodes = Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name='usb_cam',
            parameters=[os.path.join(peripherals_package_path, 'config', 'usb_cam_param.yaml'),
                        {"pixel_format": "yuyv2rgb"},
                        ],
            remappings = [
                ('image_raw/compressed', camera_topic('rgb/image_compressed')),
                ('image_raw/compressedDepth', camera_topic('rgb/compressedDepth')),
                ('image_raw/theora', camera_topic('rgb/image_raw/theora')),
                ('camera_info', camera_topic('rgb/camera_info')),
            ]
        )
    image_proc_node = Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_node',
            remappings=[
                        ('image', 'image_raw'),
                        ('image_raw/compressed', camera_topic('rgb/image_compressed')),
                        ('image_raw/compressedDepth', camera_topic('rgb/compressedDepth')),
                        ('image_raw/theora', camera_topic('rgb/image_raw/theora')),
                        ('image_rect', camera_topic('rgb/image_raw')),
                        ('camera_info', camera_topic('rgb/camera_info')),
                        ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'error']  
        )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='depth_cam'),
        camera_nodes,
        image_proc_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     compiled = os.environ.get('need_compile', 'False') 
#     if compiled == 'True':
#         peripherals_package_path = get_package_share_directory('peripherals')
#     else:
#         peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

#     # 定义 usb_cam 节点的组件
#     usb_cam_node = ComposableNode(
#         package='usb_cam',
#         plugin='usb_cam::UsbCamNode',
#         name='usb_cam',
#         parameters=[
#             os.path.join(peripherals_package_path, 'config', 'usb_cam_param.yaml'),
#             {"pixel_format": "yuyv2rgb"},
#         ],
#         remappings=[
#             ('image_raw', '/depth_cam/rgb/image_unrect'), 
#             ('camera_info', '/depth_cam/rgb/camera_info'),
#         ],
#         extra_arguments=[{'use_intra_process_comms': True}]
#     )

#     image_rectify_node = ComposableNode(
#         package='image_proc',
#         plugin='image_proc::RectifyNode', 
#         name='image_rectify_node',
#         remappings=[
#             ('image', '/depth_cam/rgb/image_unrect'), 
#             ('camera_info', '/depth_cam/rgb/camera_info'), 
#             ('image_rect', '/depth_cam/rgb/image_raw'), 
#         ],
#         extra_arguments=[{'use_intra_process_comms': True}]
#     )

#     image_container = ComposableNodeContainer(
#         name='image_container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container',
#         composable_node_descriptions=[
#             usb_cam_node,
#             image_rectify_node, 
#         ],
#         output='screen',
#     )

#     return LaunchDescription([image_container])

# if __name__ == '__main__':
#     from launch import LaunchService
#     ld = generate_launch_description()
#     ls = LaunchService()
#     ls.include_launch_description(ld)
#     ls.run()
