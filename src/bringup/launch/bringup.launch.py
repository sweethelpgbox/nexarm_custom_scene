import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sdk.scene_context import load_scene_environment


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    scene_env = load_scene_environment()
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/nexarm.launch.py')),
        launch_arguments={
            'namespace': scene_env.role_namespace.strip('/'),
        }.items(),
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
        launch_arguments={
            'depth_camera_name': scene_env.camera_name,
        }.items(),
    )

    rosbridge_websocket_launch = ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
    )
    startup_check_node = Node(
        package='app',
        executable='startup_check',
        output='screen',
    )

    start_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/start_app.launch.py')),
        launch_arguments={
            'controller_prefix': scene_env.controller_prefix,
            'camera_prefix': scene_env.camera_prefix,
            'camera_topic': scene_env.camera_topic('rgb/image_raw'),
            'camera_info_topic': scene_env.camera_topic('rgb/camera_info'),
            'depth_topic': scene_env.camera_topic('depth/image_raw'),
            'depth_info_topic': scene_env.camera_topic('depth/camera_info'),
            'calibration_enabled': scene_env.calibration_enabled_launch_value,
        }.items(),
    )

    startup_check_node = Node(
        package='bringup',
        executable='startup_check',
        output='screen',
    )
    bringup_launch = GroupAction(
        actions=[
            start_app_launch,
            TimerAction(
                # 延时等待其它节点启动好(delay and wait for other nodes to start up properly)
                period=10.0,
                actions=[depth_camera_launch],
            ),
        ]
    )
    # scene5 B 机不需要 rosbridge 和 web_video_server（上位机在 A 机）
    is_scene5_b = scene_env.is_scene5 and scene_env.arm_role == 'B'
    extra_nodes = [] if is_scene5_b else [web_video_server_node, rosbridge_websocket_launch]

    # scene5 两个 ROS_DOMAIN_ID 只在 A 机桥接图像和传送带控制
    bridge_actions = []
    if scene_env.is_scene5 and scene_env.arm_role == 'A':
        bridge_script = os.path.join(app_package_path, 'app/scene5_tcp_bridge.py')
        bridge_cmd = (
            'source /opt/ros/humble/setup.bash && '
            'source /home/ubuntu/ros2_ws/install/setup.bash && '
            f'python3 {bridge_script}'
        )
        bridge_env = {
            'SCENE5_ARM_ROLE': 'A',
            'SCENE5_BRIDGE_A_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '78'),
            'SCENE5_BRIDGE_B_DOMAIN_ID': os.environ.get('SCENE5_B_DOMAIN_ID', '79'),
        }
        bridge_actions = [
            TimerAction(
                period=5.0,  # 等基础节点启动后开 bridge（含服务代理，需要尽早就绪）
                actions=[
                    ExecuteProcess(
                        cmd=['bash', '-c', bridge_cmd],
                        output='screen',
                        additional_env=bridge_env,
                    )
                ],
            )
        ]

    # Scene5 配对接收器随 bringup 常驻，但 agent 内部只接受 scene_5 配置。
    peer_agent_script = os.path.join(app_package_path, 'app/scene5_peer_agent.py')
    peer_agent_cmd = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/ubuntu/ros2_ws/install/setup.bash && '
        f'python3 {peer_agent_script}'
    )
    peer_agent_actions = [
        ExecuteProcess(
            cmd=['bash', '-c', peer_agent_cmd],
            output='screen',
        )
    ]

    return [
        sdk_launch,
        bringup_launch,
        startup_check_node,
        *extra_nodes,
        *peer_agent_actions,
        *bridge_actions,
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
