from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    start = LaunchConfiguration('start', default='false')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    display = LaunchConfiguration('display', default='false')
    display_arg = DeclareLaunchArgument('display', default_value=display)
    broadcast = LaunchConfiguration('broadcast', default='true')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)

    object_sorting_node = Node(
        package='openclaw_controller',
        executable='object_sorting',
        output='screen',
        parameters=[{ 'start': start, 'display': display, 'broadcast': broadcast}]
    )
    object_placement_node = Node(
        package='openclaw_controller',
        executable='object_placement',
        output='screen',
    )

    return [start_arg,
            display_arg,
            broadcast_arg,
            object_sorting_node,
            object_placement_node,
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
