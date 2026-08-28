import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchService, LaunchContext
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value=use_rviz)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    if compiled == 'True':
        nexarm_description_package_path = get_package_share_directory('nexarm_description')
    else:
        nexarm_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/nexarm_description'
    urdf_path = os.path.join(nexarm_description_package_path, 'urdf/nexarm.urdf.xacro')
    rviz_config_file = os.path.join(nexarm_description_package_path, 'rviz/view.rviz')

    robot_description = Command(['xacro ', urdf_path])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # 静态TF(static TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time}],

        arguments=[urdf_path],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nexarm_description_package_path, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'rviz_config': rviz_config_file}.items())

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        namespace_arg,
        use_namespace_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_launch,
    ])


if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
