import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    machine_type = os.environ['MACHINE_TYPE']
    if 'Pro' in machine_type:
        topic_name = '/usb_cam/image_raw'
    else:
        topic_name = '/depth_cam/rgb/image_raw'
    camera_topic = LaunchConfiguration('camera_topic', default=topic_name)
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)
    
    model_name = LaunchConfiguration('model_name', default='best_garbage_26').perform(context)
    model_name_arg = DeclareLaunchArgument('model_name', default_value=model_name)

    conf = LaunchConfiguration('conf', default=0.8).perform(context)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)


    model_size = LaunchConfiguration('model_size', default=320).perform(context)
    model_size_arg = DeclareLaunchArgument('model_size', default_value=model_size)
    use_scene_pose = LaunchConfiguration('use_scene_pose', default='true')
    use_scene_pose_arg = DeclareLaunchArgument('use_scene_pose', default_value=use_scene_pose)
    play_config_path = LaunchConfiguration('play_config_path', default='')
    play_config_path_arg = DeclareLaunchArgument('play_config_path', default_value=play_config_path)

    if compiled == 'True':
        example_package_path = get_package_share_directory('example')
        app_package_path = get_package_share_directory('app')
    else:
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'


    garbage_names = ['BananaPeel','BrokenBones','CigaretteEnd','DisposableChopsticks','Ketchup','Marker','OralLiquidBottle','PlasticBottle','Plate','StorageBattery','Toothbrush', 'Umbrella']
    traffic_names = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
    yolo_names = [
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
        "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
        "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
        "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
        "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
        "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
        "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
        "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

    if 'traffic' in model_name:
        model_classes_names = traffic_names
        model_detect_method = 'detect'
    elif 'garbage' in model_name:
        model_classes_names = garbage_names
        model_detect_method = 'obb'
    else:
        model_classes_names = yolo_names
        model_detect_method = 'detect'

    yolo_node = Node(
        package='example', 
        executable='yolo_node',
        output='screen',
        parameters=[{
            'classes':  model_classes_names,
            'model': model_name,
            'engine': model_name,
            'conf': conf,
            'task': model_detect_method,
            'display': False,
            'image_topic': camera_topic,
            'model_size': model_size,
        }]
    )

    waste_classification_node = Node(
        package='app',
        executable='waste_classification',
        output='screen',
        parameters=[{
            'start': False,
            'display': False,
            'app': True,
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
            waste_classification_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
