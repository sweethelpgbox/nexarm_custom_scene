#!/bin/bash
# JetAuto (Pro)
# @author:LinQ
# 检查传递的参数，并根据参数调用不同的功能
case $1 in
    #################################  语音功能
    1)
        # echo "slam"
        # gnome-terminal \
        # --tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node.service;ros2 launch slam slam.launch.py enable_save:=false'" \
        # --tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;ros2 launch peripherals teleop_key_control.launch.py'" \
        # --tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;rviz2 rviz2 -d /home/ubuntu/ros2_ws/src/slam/rviz/slam_desktop.rviz'" \
        # --tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;ros2 run slam map_save'"
        ;;
    2)
        echo "voice_control_move"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_move.launch.py"
        ;;
    3)
        echo "voice_control_color_detect"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_color_detect.launch.py"
        ;; 
    4)
        echo "voice_control_color_track"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_color_track.launch.py"
        ;; 	
    5)
        echo "voice_control_color_sorting"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_color_sorting.launch.py"
        ;;    
    6)
        echo "voice_control_garbage_classification"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_garbage_classification.launch.py"
        ;; 
    7)
        echo "voice_control_navigation"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_navigation.launch.py map_name:=map_01"
        ;;
    8)
        echo "voice_control_navigation_transport"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_navigation_transport.launch.py map_name:=map_01"
        ;; 
    9)
        echo "voice_control_arm"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch xf_mic_asr_offline voice_control_arm.launch.py"
        ;; 
    #################################  软件
    10)
        echo "lab_tool"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py" 
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;sleep 5;python3 ~/software/lab_tool/main.py" 
        ;;
    11)
        echo "collect_picture"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py" 
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;sleep 8;python3 ~/software/collect_picture/main.py" 
        ;;
    12)
        echo "servo_tool"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;python3 ~/software/servo_tool/main.py"
        ;; 
    13)
        echo "arm_pc"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;python3 ~/software/arm_pc/main.py"
        ;; 
     #################################  无人驾驶       
    14)
        echo "yolov5_trt"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/yolov5_detect/ && python3 yolov5_trt.py"
        ;; 
    15)
        echo "self_driving"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch example self_driving.launch.py"
        ;; 
    16)
        echo "self_driving"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;ros2 launch example self_driving.launch.py only_line_follow:=true"
        ;; 
    17)
        echo "空 执行下一个编号"
        ;;
     #################################  OpenCV功能

    18)
        echo "color_detect_demo"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/color_detect && python3 color_detect_demo.py" 
        ;; 
    19)
        echo "qrcode_creater"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_creater.py" 
        ;;     
    20)
        echo "qrcode_detecter"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_detecter.py" 
        ;; 
    21)
        echo "line_following_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch app line_following_node.launch.py debug:=true"
        sleep 8  # 添加 8 秒的延时  点击选择要寻的线
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 service call /line_following/enter std_srvs/srv/Trigger {}" 
        sleep 20  # 添加 8 秒的延时 没有开启就再次手动输入
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"" 
        ;;
     #################################  深度相机 
    21)
        echo "depth_camera  手动操作rviz"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py"
        ;;  
        # depth_cam_link  
        # 依次按照顺序依次点击，查看点云图像
        # Add->By topic->depth_cam->depth->points->pointCloud2
        # 按照顺序依次点击，查看查看红外图象
        # Add->By topic->depth_cam->ir->image_raw->image
     #################################  三维建图导航
    22)
        echo "rtabmap_slam"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch slam rtabmap_slam.launch.py"
        sleep 8  # 添加 8 秒的延时 
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch slam rviz_rtabmap.launch.py" 
        sleep 20  # 添加 8 秒的延时 没有开启就手动输入
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals teleop_key_control.launch.py" 
        ;;  
    23)
        echo "rtabmap_navigation"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch navigation rtabmap_navigation.launch.py"
        sleep 8  # 添加 8 秒的延时 
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch navigation rviz_rtabmap_navigation.launch.py" 
        ;;
    #################################  MediaPipe（上）
    24)
        echo "self_segmentation 开启的depth_camera.launch.py节点不用关闭后面还会用到"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals depth_camera.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 self_segmentation.py" 
        ;; 
    25)
        echo "objectron 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 objectron.py" 
        ;;
    26)
        echo "face_detect 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_detect.py" 
        ;;  
    27)
        echo "face_mesh 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py" 
        ;;
    28)
        echo "hand 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand.py" 
        ;;
    29)
        echo "pose 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 pose.py" 
        ;;     
    30)
        echo "hand_gesture 默认depth_camera.launch.py节点已经开启"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand_gesture.py" 
        ;; 

    #################################  MediaPipe（下）

    31)
        echo "body_control  先手动关闭depth_camera.launch.py节点"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example body_control.launch.py" 
        ;;    
    32)
        echo "body_track"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example body_track.launch.py" 
        ;;
    33)
        echo "body_and_rgb_control"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example body_and_rgb_control.launch.py" 
        ;;
    34)
        echo "fall_down_detect"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example fall_down_detect.launch.py" 
        ;;
    35)
        echo "hand_track_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example hand_track_node.launch.py" 
        ;;
    36)
        echo "hand_gesture_control_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example hand_gesture_control_node.launch.py" 
        ;;  

    ################################# 机械臂控制 2D视觉
    37)
        echo "color_sorting_node debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example color_sorting_node.launch.py debug:=true" 
        ;;    
    38)
        echo "color_sorting_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example color_sorting_node.launch.py" 
        ;;     
    39)
        echo "color_track_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example color_track_node.launch.py" 
        ;; 
    40)
        echo "line_follow_clean_node debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example line_follow_clean_node.launch.py debug:=true" 
        ;; 
    41)
        echo "line_follow_clean_node"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example line_follow_clean_node.launch.py" 
        ;; 
    42)
        echo "garbage_classification debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example garbage_classification.launch.py debug:=true" 
        ;;  
    43)
        echo "garbage_classification"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example garbage_classification.launch.py" 
        ;;  
    44)
        echo "automatic_pick debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example automatic_pick.launch.py debug:=true" 
        ;;            
    45)
        echo "navigation_transport"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example navigation_transport.launch.py map:=map_01" 
        ;; 
    ################################# 机械臂控制 3D视觉  适用于带深度相机的机器
    46)
        echo "prevent_falling debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example prevent_falling.launch.py debug:=true" 
        ;;  
    47)
        echo "prevent_falling"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example prevent_falling.launch.py" 
        ;; 
    48)
        echo "cross_bridge debug"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example cross_bridge.launch.py debug:=true" 
        ;; 
    49)
        echo "cross_bridge"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example cross_bridge.launch.py" 
        ;; 
    50)
        echo "track_object"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example track_object.launch.py" 
        ;;  
    51)
        echo "track_and_grab"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example track_and_grab.launch.py" 
        ;;
    52)
        echo "object_classification"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch example object_classification.launch.py" 
        ;; 
    #################################  Yolov5
    53)
        echo "my_data 导入较少的数据集放到桌面"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;python3 ~/software/xml2yolo.py --data ~/Desktop/my_data --yaml ~/Desktop/my_data/data.yaml; exec bash"
        ;; 
    54)
        echo "yolov5 train"
        gnome-terminal \
        --tab -- bash -c "source $HOME/.zshrc;cd ~/third_party_ros2/yolov5/ && python3 train.py --img 160 --batch 8 --epochs 10 --data ~/Desktop/my_data/data.yaml --weights yolov5n.pt; exec bash"
        ;;
     #################################  二维建图导航
    56)
        echo "slam"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch slam slam.launch.py"
        sleep 10  
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch slam rviz_slam.launch.py" 
        sleep 10  
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals teleop_key_control.launch.py" 
        ;;  
    57)
        echo "save map_01"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "map_01" --ros-args -p map_subscribe_transient_local:=true; exec bash" 
        ;; 
    58)
        echo "navigation"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch navigation navigation.launch.py map:=map_01"
        sleep 10  
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch navigation rviz_navigation.launch.py" 
        ;; 
    #################################  底盘控制
    59)
        echo "imu_calib"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch ros_robot_controller ros_robot_controller.launch.py; exec bash"
        sleep 8  
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml; exec bash" 
        ;; 
    60)
        echo "imu_view"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals imu_view.launch.py" 
        ;; 
    61)
        echo "calibrate_params"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;cd ~/ros2_ws/src/driver/controller/config && vim calibrate_params.yaml"
        ;;    
    62)
        echo "angular_calib"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch calibration angular_calib.launch.py"
        ;; 
    63)
        echo "linear_calib"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch calibration linear_calib.launch.py"
        ;; 
    64)
        echo "imu_filter"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch ros_robot_controller ros_robot_controller.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch peripherals imu_filter.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic echo /imu"
        sleep 8
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic list; exec bash"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic info /imu; exec bash"
        ;;
     65)
        echo "odom_publisher"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch controller odom_publisher.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic echo /odom_raw; exec bash"
        sleep 8
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic list; exec bash"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic info /odom_raw; exec bash"
        ;;  
     66)
        echo "odom_publisher"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 launch controller controller.launch.py"
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic pub /controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2""
        sleep 5
        gnome-terminal --tab -- bash -c "source $HOME/.zshrc;ros2 topic echo /odom --field pose.pose.position; exec bash"
        ;;   
    
    

esac

