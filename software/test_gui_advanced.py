#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetAuto (Pro) 测试程序可视化界面 - 高级版本
@author: LinQ
直接执行命令，无需依赖test.sh脚本
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import os
import time
import json
from datetime import datetime
import signal
import psutil

class JetAutoTestGUIAdvanced:
    def __init__(self, root):
        self.root = root
        self.root.title("JetAuto Pro 测试程序 - 高级版")
        self.root.geometry("1200x800")
        
        # 设置主题样式
        style = ttk.Style()
        style.theme_use('clam')
        
        # 配置样式
        style.configure('Title.TLabel', font=('Arial', 20, 'bold'))
        style.configure('Category.TLabel', font=('Arial', 14, 'bold'))
        style.configure('Run.TButton', foreground='green')
        style.configure('Stop.TButton', foreground='red')
        
        # 定义测试命令
        self.test_commands = {
            "语音功能": {
                "语音控制移动": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_move.launch.py"],
                    "description": "通过语音命令控制机器人移动"
                },
                "语音控制颜色检测": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_color_detect.launch.py"],
                    "description": "通过语音命令进行颜色检测"
                },
                "语音控制颜色追踪": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_color_track.launch.py"],
                    "description": "通过语音命令追踪指定颜色"
                },
                "语音控制颜色分拣": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_color_sorting.launch.py"],
                    "description": "通过语音命令进行颜色分拣"
                },
                "语音控制垃圾分类": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_garbage_classification.launch.py"],
                    "description": "通过语音命令进行垃圾分类"
                },
                "语音控制导航": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_navigation.launch.py map_name:=map_01"],
                    "description": "通过语音命令进行导航，使用map_01地图"
                },
                "语音控制导航运输": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_navigation_transport.launch.py map_name:=map_01"],
                    "description": "通过语音命令进行导航运输任务"
                },
                "语音控制机械臂": {
                    "commands": ["ros2 launch xf_mic_asr_offline voice_control_arm.launch.py"],
                    "description": "通过语音命令控制机械臂"
                }
            },
            "软件工具": {
                "Lab工具": {
                    "commands": [
                        "ros2 launch peripherals depth_camera.launch.py",
                        "sleep 5 && python3 ~/software/lab_tool/main.py"
                    ],
                    "description": "启动实验室工具，用于颜色阈值调试"
                },
                "图片采集工具": {
                    "commands": [
                        "ros2 launch peripherals depth_camera.launch.py",
                        "sleep 8 && python3 ~/software/collect_picture/main.py"
                    ],
                    "description": "启动图片采集工具"
                },
                "舵机工具": {
                    "commands": ["python3 ~/software/servo_tool/main.py"],
                    "description": "舵机调试和控制工具"
                },
                "机械臂PC控制": {
                    "commands": ["python3 ~/software/arm_pc/main.py"],
                    "description": "机械臂PC端控制程序"
                }
            },
            "无人驾驶": {
                "YOLOv5 TRT检测": {
                    "commands": ["cd ~/ros2_ws/src/example/example/yolov5_detect/ && python3 yolov5_trt.py"],
                    "description": "运行YOLOv5 TensorRT加速检测"
                },
                "自动驾驶": {
                    "commands": ["ros2 launch example self_driving.launch.py"],
                    "description": "启动完整的自动驾驶功能"
                },
                "自动驾驶(仅巡线)": {
                    "commands": ["ros2 launch example self_driving.launch.py only_line_follow:=true"],
                    "description": "启动仅巡线的自动驾驶模式"
                }
            },
            "OpenCV功能": {
                "颜色检测演示": {
                    "commands": [
                        "ros2 launch peripherals depth_camera.launch.py",
                        "cd ~/ros2_ws/src/example/example/color_detect && python3 color_detect_demo.py"
                    ],
                    "description": "OpenCV颜色检测演示"
                },
                "二维码生成器": {
                    "commands": ["cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_creater.py"],
                    "description": "生成二维码"
                },
                "二维码检测器": {
                    "commands": [
                        "ros2 launch peripherals depth_camera.launch.py",
                        "cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_detecter.py"
                    ],
                    "description": "检测和识别二维码"
                },
                "巡线功能": {
                    "commands": [
                        "ros2 launch app line_following_node.launch.py debug:=true",
                        "sleep 8 && ros2 service call /line_following/enter std_srvs/srv/Trigger {}",
                        "sleep 20 && ros2 service call /line_following/set_running std_srvs/srv/SetBool \"{data: True}\""
                    ],
                    "description": "启动巡线功能，自动开始巡线"
                }
            },
            "深度相机": {
                "深度相机查看": {
                    "commands": ["ros2 launch peripherals depth_camera.launch.py"],
                    "description": "启动深度相机，可在RViz中查看点云和红外图像"
                }
            },
            "三维建图导航": {
                "RTAB-Map三维建图": {
                    "commands": [
                        "ros2 launch slam rtabmap_slam.launch.py",
                        "sleep 8 && ros2 launch slam rviz_rtabmap.launch.py",
                        "sleep 20 && ros2 launch peripherals teleop_key_control.launch.py"
                    ],
                    "description": "使用RTAB-Map进行三维建图"
                },
                "RTAB-Map三维导航": {
                    "commands": [
                        "ros2 launch navigation rtabmap_navigation.launch.py",
                        "sleep 8 && ros2 launch navigation rviz_rtabmap_navigation.launch.py"
                    ],
                    "description": "使用RTAB-Map进行三维导航"
                }
            },
            "MediaPipe(上)": {
                "人像分割": {
                    "commands": [
                        "ros2 launch peripherals depth_camera.launch.py",
                        "cd ~/ros2_ws/src/example/example/mediapipe_example && python3 self_segmentation.py"
                    ],
                    "description": "MediaPipe人像分割功能"
                },
                "3D物体检测": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 objectron.py"],
                    "description": "MediaPipe 3D物体检测"
                },
                "人脸检测": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_detect.py"],
                    "description": "MediaPipe人脸检测"
                },
                "人脸网格": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py"],
                    "description": "MediaPipe人脸网格"
                },
                "手部检测": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand.py"],
                    "description": "MediaPipe手部检测"
                },
                "姿态检测": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 pose.py"],
                    "description": "MediaPipe姿态检测"
                },
                "手势识别": {
                    "commands": ["cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand_gesture.py"],
                    "description": "MediaPipe手势识别"
                }
            },
            "MediaPipe(下)": {
                "身体控制": {
                    "commands": ["ros2 launch example body_control.launch.py"],
                    "description": "通过身体姿态控制机器人",
                    "note": "需要先关闭depth_camera节点"
                },
                "身体追踪": {
                    "commands": ["ros2 launch example body_track.launch.py"],
                    "description": "追踪人体并跟随"
                },
                "身体和RGB控制": {
                    "commands": ["ros2 launch example body_and_rgb_control.launch.py"],
                    "description": "结合身体姿态和RGB灯控制"
                },
                "跌倒检测": {
                    "commands": ["ros2 launch example fall_down_detect.launch.py"],
                    "description": "检测人体跌倒事件"
                },
                "手部追踪": {
                    "commands": ["ros2 launch example hand_track_node.launch.py"],
                    "description": "追踪手部运动"
                },
                "手势控制": {
                    "commands": ["ros2 launch example hand_gesture_control_node.launch.py"],
                    "description": "通过手势控制机器人"
                }
            },
            "机械臂2D视觉": {
                "颜色分拣(调试)": {
                    "commands": ["ros2 launch example color_sorting_node.launch.py debug:=true"],
                    "description": "颜色分拣功能调试模式"
                },
                "颜色分拣": {
                    "commands": ["ros2 launch example color_sorting_node.launch.py"],
                    "description": "自动颜色分拣"
                },
                "颜色追踪": {
                    "commands": ["ros2 launch example color_track_node.launch.py"],
                    "description": "追踪指定颜色物体"
                },
                "巡线清理(调试)": {
                    "commands": ["ros2 launch example line_follow_clean_node.launch.py debug:=true"],
                    "description": "巡线清理功能调试模式"
                },
                "巡线清理": {
                    "commands": ["ros2 launch example line_follow_clean_node.launch.py"],
                    "description": "沿线清理物体"
                },
                "垃圾分类(调试)": {
                    "commands": ["ros2 launch example garbage_classification.launch.py debug:=true"],
                    "description": "垃圾分类功能调试模式"
                },
                "垃圾分类": {
                    "commands": ["ros2 launch example garbage_classification.launch.py"],
                    "description": "自动垃圾分类"
                },
                "自动拾取(调试)": {
                    "commands": ["ros2 launch example automatic_pick.launch.py debug:=true"],
                    "description": "自动拾取功能调试模式"
                },
                "导航运输": {
                    "commands": ["ros2 launch example navigation_transport.launch.py map:=map_01"],
                    "description": "导航运输任务"
                }
            },
            "机械臂3D视觉": {
                "防跌落(调试)": {
                    "commands": ["ros2 launch example prevent_falling.launch.py debug:=true"],
                    "description": "防跌落功能调试模式"
                },
                "防跌落": {
                    "commands": ["ros2 launch example prevent_falling.launch.py"],
                    "description": "防止从高处跌落"
                },
                "过桥(调试)": {
                    "commands": ["ros2 launch example cross_bridge.launch.py debug:=true"],
                    "description": "过桥功能调试模式"
                },
                "过桥": {
                    "commands": ["ros2 launch example cross_bridge.launch.py"],
                    "description": "自动过桥"
                },
                "物体追踪": {
                    "commands": ["ros2 launch example track_object.launch.py"],
                    "description": "3D物体追踪"
                },
                "追踪并抓取": {
                    "commands": ["ros2 launch example track_and_grab.launch.py"],
                    "description": "追踪并抓取物体"
                },
                "物体分类": {
                    "commands": ["ros2 launch example object_classification.launch.py"],
                    "description": "3D物体分类"
                }
            },
            "YOLOv5": {
                "准备数据集": {
                    "commands": ["python3 ~/software/xml2yolo.py --data ~/Desktop/my_data --yaml ~/Desktop/my_data/data.yaml"],
                    "description": "转换数据集格式为YOLO格式"
                },
                "YOLOv5训练": {
                    "commands": ["cd ~/third_party_ros2/yolov5/ && python3 train.py --img 160 --batch 8 --epochs 10 --data ~/Desktop/my_data/data.yaml --weights yolov5n.pt"],
                    "description": "训练YOLOv5模型"
                }
            },
            "二维建图导航": {
                "SLAM建图": {
                    "commands": [
                        "ros2 launch slam slam.launch.py",
                        "sleep 10 && ros2 launch slam rviz_slam.launch.py",
                        "sleep 10 && ros2 launch peripherals teleop_key_control.launch.py"
                    ],
                    "description": "启动SLAM建图"
                },
                "保存地图": {
                    "commands": ["cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f \"map_01\" --ros-args -p map_subscribe_transient_local:=true"],
                    "description": "保存当前地图为map_01"
                },
                "导航": {
                    "commands": [
                        "ros2 launch navigation navigation.launch.py map:=map_01",
                        "sleep 10 && ros2 launch navigation rviz_navigation.launch.py"
                    ],
                    "description": "使用map_01进行导航"
                }
            },
            "底盘控制": {
                "IMU校准": {
                    "commands": [
                        "ros2 launch ros_robot_controller ros_robot_controller.launch.py",
                        "sleep 8 && ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml"
                    ],
                    "description": "校准IMU传感器"
                },
                "IMU查看": {
                    "commands": ["ros2 launch peripherals imu_view.launch.py"],
                    "description": "查看IMU数据"
                },
                "校准参数编辑": {
                    "commands": ["gnome-terminal -- vim ~/ros2_ws/src/driver/controller/config/calibrate_params.yaml"],
                    "description": "编辑校准参数文件"
                },
                "角速度校准": {
                    "commands": ["ros2 launch calibration angular_calib.launch.py"],
                    "description": "校准角速度"
                },
                "线速度校准": {
                    "commands": ["ros2 launch calibration linear_calib.launch.py"],
                    "description": "校准线速度"
                },
                "IMU滤波": {
                    "commands": [
                        "ros2 launch ros_robot_controller ros_robot_controller.launch.py",
                        "ros2 launch peripherals imu_filter.launch.py",
                        "ros2 topic echo /imu",
                        "sleep 8 && ros2 topic list",
                        "ros2 topic info /imu"
                    ],
                    "description": "IMU数据滤波"
                },
                "里程计发布": {
                    "commands": [
                        "ros2 launch controller odom_publisher.launch.py",
                        "ros2 topic echo /odom_raw",
                        "sleep 8 && ros2 topic list",
                        "ros2 topic info /odom_raw"
                    ],
                    "description": "发布里程计数据"
                },
                "控制器测试": {
                    "commands": [
                        "ros2 launch controller controller.launch.py",
                        "ros2 topic pub /controller/cmd_vel geometry_msgs/Twist \"linear:\n  x: 0.2\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.2\"",
                        "sleep 5 && ros2 topic echo /odom --field pose.pose.position"
                    ],
                    "description": "测试控制器"
                }
            }
        }
        
        # 创建主界面
        self.create_widgets()
        
        # 存储正在运行的进程
        self.running_processes = {}
        
        # 加载历史记录
        self.load_history()
        
    def create_widgets(self):
        """创建界面组件"""
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=2)
        main_frame.rowconfigure(1, weight=1)
        
        # 标题
        title_label = ttk.Label(main_frame, text="JetAuto Pro 测试程序 - 高级版", style='Title.TLabel')
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # 左侧：功能分类树
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        left_frame.rowconfigure(1, weight=1)
        
        tree_label = ttk.Label(left_frame, text="功能分类", style='Category.TLabel')
        tree_label.grid(row=0, column=0, pady=(0, 5))
        
        # 搜索框
        search_frame = ttk.Frame(left_frame)
        search_frame.grid(row=0, column=1, sticky=(tk.E), padx=(10, 0))
        
        ttk.Label(search_frame, text="搜索:").pack(side=tk.LEFT, padx=(0, 5))
        self.search_var = tk.StringVar()
        self.search_entry = ttk.Entry(search_frame, textvariable=self.search_var, width=20)
        self.search_entry.pack(side=tk.LEFT)
        self.search_var.trace('w', self.filter_tree)
        
        # 创建树形视图
        tree_frame = ttk.Frame(left_frame)
        tree_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.tree = ttk.Treeview(tree_frame, selectmode='browse', columns=('status',), displaycolumns=('status',))
        self.tree.heading('#0', text='功能名称')
        self.tree.heading('status', text='状态')
        self.tree.column('status', width=80)
        
        # 添加滚动条
        tree_vscroll = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        tree_vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        tree_hscroll = ttk.Scrollbar(tree_frame, orient="horizontal", command=self.tree.xview)
        tree_hscroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.tree.configure(yscrollcommand=tree_vscroll.set, xscrollcommand=tree_hscroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 填充树形视图
        self.populate_tree()
        
        # 绑定事件
        self.tree.bind('<<TreeviewSelect>>', self.on_tree_select)
        self.tree.bind('<Double-Button-1>', self.on_tree_double_click)
        
        # 右侧：控制面板和输出
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        right_frame.rowconfigure(2, weight=1)
        right_frame.columnconfigure(0, weight=1)
        
        # 控制按钮框架
        control_frame = ttk.LabelFrame(right_frame, text="控制面板", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        # 选中的功能标签
        self.selected_label = ttk.Label(control_frame, text="请选择一个功能", font=('Arial', 12))
        self.selected_label.grid(row=0, column=0, columnspan=4, pady=(0, 10))
        
        # 功能描述
        self.description_label = ttk.Label(control_frame, text="", wraplength=500)
        self.description_label.grid(row=1, column=0, columnspan=4, pady=(0, 10))
        
        # 按钮行
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=2, column=0, columnspan=4)
        
        # 运行按钮
        self.run_button = ttk.Button(button_frame, text="▶ 运行", command=self.run_selected, state='disabled', style='Run.TButton')
        self.run_button.pack(side=tk.LEFT, padx=5)
        
        # 停止按钮
        self.stop_current_button = ttk.Button(button_frame, text="■ 停止当前", command=self.stop_current, state='disabled')
        self.stop_current_button.pack(side=tk.LEFT, padx=5)
        
        # 停止所有按钮
        self.stop_all_button = ttk.Button(button_frame, text="■ 停止所有", command=self.stop_all, style='Stop.TButton')
        self.stop_all_button.pack(side=tk.LEFT, padx=5)
        
        # 清空输出按钮
        self.clear_button = ttk.Button(button_frame, text="清空输出", command=self.clear_output)
        self.clear_button.pack(side=tk.LEFT, padx=5)
        
        # 命令预览框架
        cmd_frame = ttk.LabelFrame(right_frame, text="命令预览", padding="10")
        cmd_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        cmd_frame.columnconfigure(0, weight=1)
        
        self.cmd_text = tk.Text(cmd_frame, height=4, wrap=tk.WORD)
        self.cmd_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        cmd_scroll = ttk.Scrollbar(cmd_frame, orient="vertical", command=self.cmd_text.yview)
        cmd_scroll.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.cmd_text.configure(yscrollcommand=cmd_scroll.set)
        
        # 输出框架
        output_frame = ttk.LabelFrame(right_frame, text="输出信息", padding="10")
        output_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)
        
        # 输出标签页
        self.notebook = ttk.Notebook(output_frame)
        self.notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 主输出标签页
        self.main_output_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.main_output_frame, text="主输出")
        self.main_output_frame.rowconfigure(0, weight=1)
        self.main_output_frame.columnconfigure(0, weight=1)
        
        self.output_text = scrolledtext.ScrolledText(self.main_output_frame, wrap=tk.WORD)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置文本标签
        self.output_text.tag_config("timestamp", foreground="gray")
        self.output_text.tag_config("info", foreground="blue")
        self.output_text.tag_config("error", foreground="red")
        self.output_text.tag_config("success", foreground="green")
        self.output_text.tag_config("warning", foreground="orange")
        
        # 底部状态栏
        status_frame = ttk.Frame(self.root)
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), padx=10, pady=5)
        status_frame.columnconfigure(0, weight=1)
        
        self.status_label = ttk.Label(status_frame, text="就绪", relief=tk.SUNKEN)
        self.status_label.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        self.process_count_label = ttk.Label(status_frame, text="运行中: 0", relief=tk.SUNKEN)
        self.process_count_label.grid(row=0, column=1, padx=(5, 0))
        
    def populate_tree(self):
        """填充树形视图"""
        for category, items in self.test_commands.items():
            category_id = self.tree.insert('', 'end', text=category, values=('',))
            for name, config in items.items():
                self.tree.insert(category_id, 'end', text=name, values=('就绪',), tags=(name,))
        
        # 展开所有分类
        for item in self.tree.get_children():
            self.tree.item(item, open=True)
    
    def filter_tree(self, *args):
        """搜索过滤树形视图"""
        search_term = self.search_var.get().lower()
        
        # 首先隐藏所有项
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        # 重新填充符合搜索条件的项
        for category, items in self.test_commands.items():
            category_items = []
            for name, config in items.items():
                if search_term in name.lower() or search_term in category.lower():
                    category_items.append((name, config))
            
            if category_items:
                category_id = self.tree.insert('', 'end', text=category, values=('',))
                for name, config in category_items:
                    status = '运行中' if name in self.running_processes else '就绪'
                    self.tree.insert(category_id, 'end', text=name, values=(status,), tags=(name,))
                self.tree.item(category_id, open=True)
    
    def on_tree_select(self, event):
        """树形视图选择事件"""
        selected = self.tree.selection()
        if selected:
            item = self.tree.item(selected[0])
            item_text = item['text']
            
            # 查找对应的命令配置
            for category, items in self.test_commands.items():
                if item_text in items:
                    config = items[item_text]
                    self.selected_label.config(text=f"选中: {item_text}")
                    self.description_label.config(text=config.get('description', ''))
                    
                    # 显示命令预览
                    self.cmd_text.delete(1.0, tk.END)
                    for i, cmd in enumerate(config['commands']):
                        self.cmd_text.insert(tk.END, f"{i+1}. {cmd}\n")
                    
                    # 显示注意事项
                    if 'note' in config:
                        self.cmd_text.insert(tk.END, f"\n注意: {config['note']}\n")
                    
                    # 更新按钮状态
                    if item_text in self.running_processes:
                        self.run_button.config(state='disabled')
                        self.stop_current_button.config(state='normal')
                    else:
                        self.run_button.config(state='normal')
                        self.stop_current_button.config(state='disabled')
                    
                    self.current_selection = (category, item_text)
                    return
            
            # 如果选中的是分类
            self.selected_label.config(text="请选择一个功能")
            self.description_label.config(text="")
            self.cmd_text.delete(1.0, tk.END)
            self.run_button.config(state='disabled')
            self.stop_current_button.config(state='disabled')
            self.current_selection = None
    
    def on_tree_double_click(self, event):
        """双击运行功能"""
        if hasattr(self, 'current_selection') and self.current_selection:
            self.run_selected()
    
    def run_selected(self):
        """运行选中的功能"""
        if not hasattr(self, 'current_selection') or not self.current_selection:
            return
        
        category, name = self.current_selection
        config = self.test_commands[category][name]
        
        # 检查是否已在运行
        if name in self.running_processes:
            self.log_output(f"{name} 已经在运行中\n", "warning")
            return
        
        self.log_output(f"正在启动: {name}\n", "info")
        self.status_label.config(text=f"正在运行: {name}")
        
        # 创建新的输出标签页
        output_frame = ttk.Frame(self.notebook)
        self.notebook.add(output_frame, text=name[:15] + "...")
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)
        
        output_text = scrolledtext.ScrolledText(output_frame, wrap=tk.WORD)
        output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_text.tag_config("timestamp", foreground="gray")
        output_text.tag_config("info", foreground="blue")
        output_text.tag_config("error", foreground="red")
        
        # 切换到新标签页
        self.notebook.select(output_frame)
        
        # 先存储进程信息占位，确保停止按钮能够工作
        self.running_processes[name] = {
            'processes': [],
            'pids': [],
            'start_time': datetime.now(),
            'output_text': output_text,
            'output_frame': output_frame,
            'commands': config['commands']
        }
        
        # 更新状态
        self.update_process_status(name, '运行中')
        self.update_process_count()
        
        # 更新按钮状态
        self.run_button.config(state='disabled')
        self.stop_current_button.config(state='normal')
        
        # 在新线程中运行命令
        thread = threading.Thread(target=self.execute_commands, args=(name, config, output_text, output_frame))
        thread.daemon = True
        thread.start()
        
        # 保存到历史记录
        self.save_to_history(name)
    
    def execute_commands(self, name, config, output_text, output_frame):
        """执行命令列表"""
        processes = []
        pids = []  # 存储所有进程的PID
        
        try:
            # 记录开始时间
            start_time = datetime.now()
            self.log_to_widget(output_text, f"[{start_time.strftime('%H:%M:%S')}] 开始执行: {name}\n", "info")
            
            # 执行每个命令
            for i, cmd in enumerate(config['commands']):
                # 检查是否已被停止
                if name not in self.running_processes:
                    self.log_to_widget(output_text, f"\n任务被停止\n", "warning")
                    return
                
                self.log_to_widget(output_text, f"\n[命令 {i+1}] {cmd}\n", "timestamp")
                
                # 处理特殊命令
                if cmd.startswith("sleep"):
                    # 处理延时命令
                    parts = cmd.split("&&", 1)
                    sleep_time = int(parts[0].strip().split()[1])
                    self.log_to_widget(output_text, f"等待 {sleep_time} 秒...\n", "info")
                    
                    # 分段等待，以便能够响应停止命令
                    for _ in range(sleep_time):
                        if name not in self.running_processes:
                            return
                        time.sleep(1)
                    
                    if len(parts) > 1:
                        cmd = parts[1].strip()
                    else:
                        continue
                
                # 执行命令
                env = os.environ.copy()
                env['HOME'] = os.path.expanduser('~')
                
                # 使用 preexec_fn 来设置进程组
                process = subprocess.Popen(
                    f"source $HOME/ros2_ws/.robotrc && {cmd}",
                    shell=True,
                    executable='/bin/bash',
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    universal_newlines=True,
                    env=env,
                    preexec_fn=os.setsid  # 创建新的进程组
                )
                
                processes.append(process)
                pids.append(process.pid)
                
                # 更新进程信息
                if name in self.running_processes:
                    self.running_processes[name]['processes'] = processes
                    self.running_processes[name]['pids'] = pids
                
                # 如果不是后台命令，等待输出
                if not cmd.endswith('&'):
                    # 实时读取输出
                    for line in iter(process.stdout.readline, ''):
                        if line:
                            self.log_to_widget(output_text, line, "")
                        if name not in self.running_processes:
                            # 进程被停止
                            break
            
            # 监控进程状态
            while name in self.running_processes:
                all_finished = True
                for process in processes:
                    if process.poll() is None:
                        all_finished = False
                        break
                if all_finished:
                    break
                time.sleep(0.5)
            
            # 如果正常结束
            if name in self.running_processes:
                # 计算运行时间
                end_time = datetime.now()
                duration = end_time - start_time
                
                self.log_to_widget(output_text, f"\n[{end_time.strftime('%H:%M:%S')}] {name} 运行完成\n", "success")
                self.log_to_widget(output_text, f"总耗时: {duration}\n", "info")
            
        except Exception as e:
            self.log_to_widget(output_text, f"\n执行出错: {str(e)}\n", "error")
        finally:
            # 清理进程信息
            if name in self.running_processes:
                del self.running_processes[name]
            
            # 更新状态
            self.update_process_status(name, '就绪')
            self.update_process_count()
            
            # 更新UI
            self.root.after(0, self.on_process_finished, name)
    
    def log_output(self, text, tag=""):
        """添加输出到主日志"""
        timestamp = datetime.now().strftime('[%H:%M:%S] ')
        self.output_text.insert(tk.END, timestamp, "timestamp")
        self.output_text.insert(tk.END, text, tag)
        self.output_text.see(tk.END)
    
    def log_to_widget(self, widget, text, tag=""):
        """添加输出到指定widget"""
        def append():
            widget.insert(tk.END, text, tag)
            widget.see(tk.END)
        self.root.after(0, append)
    
    def clear_output(self):
        """清空主输出"""
        self.output_text.delete(1.0, tk.END)
    
    def stop_current(self):
        """停止当前选中的进程"""
        if hasattr(self, 'current_selection') and self.current_selection:
            category, name = self.current_selection
            if name in self.running_processes:
                self.stop_process(name)
    
    def stop_process(self, name):
        """停止指定进程"""
        if name in self.running_processes:
            info = self.running_processes[name]
            self.log_output(f"正在停止: {name}\n", "warning")
            
            # 使用psutil终止进程树
            for pid in info.get('pids', []):
                try:
                    # 获取进程组ID并终止整个进程组
                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                    self.log_output(f"发送SIGTERM信号到进程组 {pid}\n", "info")
                except Exception as e:
                    self.log_output(f"无法终止进程组 {pid}: {e}\n", "warning")
                
                # 使用psutil查找并终止所有子进程
                try:
                    parent = psutil.Process(pid)
                    children = parent.children(recursive=True)
                    
                    # 先终止子进程
                    for child in children:
                        try:
                            child.terminate()
                        except psutil.NoSuchProcess:
                            pass
                    
                    # 等待子进程结束
                    gone, alive = psutil.wait_procs(children, timeout=3)
                    
                    # 强制杀死还活着的进程
                    for p in alive:
                        try:
                            p.kill()
                        except psutil.NoSuchProcess:
                            pass
                    
                    # 终止父进程
                    try:
                        parent.terminate()
                        parent.wait(timeout=3)
                    except psutil.TimeoutExpired:
                        parent.kill()
                    except psutil.NoSuchProcess:
                        pass
                        
                except psutil.NoSuchProcess:
                    pass
                except Exception as e:
                    self.log_output(f"psutil处理进程 {pid} 时出错: {e}\n", "warning")
            
            # 同时尝试终止subprocess.Popen对象
            for process in info['processes']:
                try:
                    process.terminate()
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                    except:
                        pass
                except:
                    pass
            
            # 额外的安全措施：查找并终止所有相关的ROS2进程
            try:
                # 根据功能名称查找可能的进程
                for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                    try:
                        cmdline = ' '.join(proc.info['cmdline'] or [])
                        # 检查是否包含相关的ROS2命令
                        if 'ros2' in cmdline and any(cmd_part in cmdline for cmd in info.get('commands', []) for cmd_part in cmd.split()):
                            self.log_output(f"终止相关进程: PID={proc.info['pid']} CMD={cmdline[:50]}...\n", "info")
                            proc.terminate()
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
            except Exception as e:
                self.log_output(f"查找相关进程时出错: {e}\n", "warning")
            
            # 记录停止信息
            self.log_to_widget(info['output_text'], f"\n进程被用户停止\n", "warning")
            
            # 清理
            del self.running_processes[name]
            self.update_process_status(name, '就绪')
            self.update_process_count()
            self.on_process_finished(name)
            
            self.log_output(f"{name} 已停止\n", "success")
    
    def stop_all(self):
        """停止所有进程"""
        if not self.running_processes:
            self.log_output("没有正在运行的进程\n", "info")
            return
        
        if messagebox.askyesno("确认", f"确定要停止所有 {len(self.running_processes)} 个正在运行的进程吗？"):
            # 复制键列表，避免在迭代时修改字典
            process_names = list(self.running_processes.keys())
            for name in process_names:
                self.stop_process(name)
            
            # 额外的强制措施：杀死所有ROS2相关进程
            try:
                # 杀死所有ros2进程
                subprocess.run("pkill -f 'ros2'", shell=True)
                time.sleep(0.5)
                # 强制杀死
                subprocess.run("pkill -9 -f 'ros2'", shell=True)
                
                # 杀死python3进程（可能是ROS2节点）
                subprocess.run("pkill -f 'python3.*ros2_ws'", shell=True)
                
                # 杀死rviz进程
                subprocess.run("pkill -f 'rviz'", shell=True)
                
                # 杀死gnome-terminal进程
                subprocess.run("pkill -f 'gnome-terminal'", shell=True)
                
                self.log_output("已强制停止所有ROS2相关进程\n", "warning")
            except Exception as e:
                self.log_output(f"强制停止进程时出错: {e}\n", "error")
            
            self.log_output("已停止所有进程\n", "info")
            self.status_label.config(text="就绪")
    
    def update_process_status(self, name, status):
        """更新进程状态显示"""
        def update():
            # 更新树形视图中的状态
            for item in self.tree.get_children():
                for child in self.tree.get_children(item):
                    if self.tree.item(child)['text'] == name:
                        self.tree.set(child, 'status', status)
                        break
        self.root.after(0, update)
    
    def update_process_count(self):
        """更新运行中的进程计数"""
        count = len(self.running_processes)
        self.process_count_label.config(text=f"运行中: {count}")
        
        if count == 0:
            self.status_label.config(text="就绪")
        else:
            names = list(self.running_processes.keys())
            self.status_label.config(text=f"运行中: {', '.join(names[:3])}{'...' if len(names) > 3 else ''}")
    
    def on_process_finished(self, name):
        """进程结束时的处理"""
        # 更新当前选中项的按钮状态
        if hasattr(self, 'current_selection') and self.current_selection:
            _, selected_name = self.current_selection
            if selected_name == name:
                self.run_button.config(state='normal')
                self.stop_current_button.config(state='disabled')
    
    def save_to_history(self, name):
        """保存到历史记录"""
        history_file = os.path.expanduser("~/.jetauto_test_history.json")
        try:
            if os.path.exists(history_file):
                with open(history_file, 'r') as f:
                    history = json.load(f)
            else:
                history = []
            
            history.append({
                'name': name,
                'timestamp': datetime.now().isoformat()
            })
            
            # 只保留最近100条
            history = history[-100:]
            
            with open(history_file, 'w') as f:
                json.dump(history, f, indent=2)
        except:
            pass
    
    def load_history(self):
        """加载历史记录"""
        history_file = os.path.expanduser("~/.jetauto_test_history.json")
        try:
            if os.path.exists(history_file):
                with open(history_file, 'r') as f:
                    self.history = json.load(f)
            else:
                self.history = []
        except:
            self.history = []
    
    def on_closing(self):
        """关闭窗口时的处理"""
        if self.running_processes:
            if messagebox.askyesno("确认", f"还有 {len(self.running_processes)} 个进程在运行，确定要退出吗？"):
                self.stop_all()
                self.root.destroy()
        else:
            self.root.destroy()

def main():
    root = tk.Tk()
    app = JetAutoTestGUIAdvanced(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # 设置图标（如果有的话）
    try:
        root.iconbitmap('@/usr/share/pixmaps/python.xpm')
    except:
        pass
    
    root.mainloop()

if __name__ == "__main__":
    main() 