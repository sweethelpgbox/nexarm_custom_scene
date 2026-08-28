#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetAuto (Pro) 测试程序可视化界面
@author: LinQ
将原test.sh脚本转换为可视化GUI程序
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import os
import time

class JetAutoTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("JetAuto Pro 测试程序")
        self.root.geometry("1000x700")
        
        # 设置主题样式
        style = ttk.Style()
        style.theme_use('clam')
        
        # 定义功能分类和对应的测试项
        self.test_categories = {
            "语音功能": [
                (2, "voice_control_move", "语音控制移动"),
                (3, "voice_control_color_detect", "语音控制颜色检测"),
                (4, "voice_control_color_track", "语音控制颜色追踪"),
                (5, "voice_control_color_sorting", "语音控制颜色分拣"),
                (6, "voice_control_garbage_classification", "语音控制垃圾分类"),
                (7, "voice_control_navigation", "语音控制导航"),
                (8, "voice_control_navigation_transport", "语音控制导航运输"),
                (9, "voice_control_arm", "语音控制机械臂"),
            ],
            "软件工具": [
                (10, "lab_tool", "Lab工具"),
                (11, "collect_picture", "图片采集工具"),
                (12, "servo_tool", "舵机工具"),
                (13, "arm_pc", "机械臂PC控制"),
            ],
            "无人驾驶": [
                (14, "yolov5_trt", "YOLOv5 TRT检测"),
                (15, "self_driving", "自动驾驶"),
                (16, "self_driving_line_follow", "自动驾驶(仅巡线)"),
            ],
            "OpenCV功能": [
                (18, "color_detect_demo", "颜色检测演示"),
                (19, "qrcode_creater", "二维码生成器"),
                (20, "qrcode_detecter", "二维码检测器"),
                (21, "line_following_node", "巡线功能"),
            ],
            "深度相机": [
                (21, "depth_camera", "深度相机查看"),
            ],
            "三维建图导航": [
                (22, "rtabmap_slam", "RTAB-Map 三维建图"),
                (23, "rtabmap_navigation", "RTAB-Map 三维导航"),
            ],
            "MediaPipe(上)": [
                (24, "self_segmentation", "人像分割"),
                (25, "objectron", "3D物体检测"),
                (26, "face_detect", "人脸检测"),
                (27, "face_mesh", "人脸网格"),
                (28, "hand", "手部检测"),
                (29, "pose", "姿态检测"),
                (30, "hand_gesture", "手势识别"),
            ],
            "MediaPipe(下)": [
                (31, "body_control", "身体控制"),
                (32, "body_track", "身体追踪"),
                (33, "body_and_rgb_control", "身体和RGB控制"),
                (34, "fall_down_detect", "跌倒检测"),
                (35, "hand_track_node", "手部追踪"),
                (36, "hand_gesture_control_node", "手势控制"),
            ],
            "机械臂2D视觉": [
                (37, "color_sorting_debug", "颜色分拣(调试)"),
                (38, "color_sorting", "颜色分拣"),
                (39, "color_track", "颜色追踪"),
                (40, "line_follow_clean_debug", "巡线清理(调试)"),
                (41, "line_follow_clean", "巡线清理"),
                (42, "garbage_classification_debug", "垃圾分类(调试)"),
                (43, "garbage_classification", "垃圾分类"),
                (44, "automatic_pick_debug", "自动拾取(调试)"),
                (45, "navigation_transport", "导航运输"),
            ],
            "机械臂3D视觉": [
                (46, "prevent_falling_debug", "防跌落(调试)"),
                (47, "prevent_falling", "防跌落"),
                (48, "cross_bridge_debug", "过桥(调试)"),
                (49, "cross_bridge", "过桥"),
                (50, "track_object", "物体追踪"),
                (51, "track_and_grab", "追踪并抓取"),
                (52, "object_classification", "物体分类"),
            ],
            "YOLOv5": [
                (53, "prepare_dataset", "准备数据集"),
                (54, "yolov5_train", "YOLOv5训练"),
            ],
            "二维建图导航": [
                (56, "slam", "SLAM建图"),
                (57, "save_map", "保存地图"),
                (58, "navigation", "导航"),
            ],
            "底盘控制": [
                (59, "imu_calib", "IMU校准"),
                (60, "imu_view", "IMU查看"),
                (61, "calibrate_params", "校准参数编辑"),
                (62, "angular_calib", "角速度校准"),
                (63, "linear_calib", "线速度校准"),
                (64, "imu_filter", "IMU滤波"),
                (65, "odom_publisher", "里程计发布"),
                (66, "controller_test", "控制器测试"),
            ]
        }
        
        # 创建主界面
        self.create_widgets()
        
        # 存储正在运行的进程
        self.running_processes = []
        
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
        title_label = ttk.Label(main_frame, text="JetAuto Pro 测试程序", font=('Arial', 20, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # 左侧：功能分类树
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        tree_label = ttk.Label(left_frame, text="功能分类", font=('Arial', 14, 'bold'))
        tree_label.pack(pady=(0, 5))
        
        # 创建树形视图
        self.tree = ttk.Treeview(left_frame, selectmode='browse')
        self.tree.pack(fill=tk.BOTH, expand=True)
        
        # 添加滚动条
        tree_scroll = ttk.Scrollbar(left_frame, orient="vertical", command=self.tree.yview)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        
        # 填充树形视图
        self.populate_tree()
        
        # 绑定选择事件
        self.tree.bind('<<TreeviewSelect>>', self.on_tree_select)
        
        # 右侧：控制面板和输出
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        right_frame.rowconfigure(1, weight=1)
        right_frame.columnconfigure(0, weight=1)
        
        # 控制按钮框架
        control_frame = ttk.LabelFrame(right_frame, text="控制面板", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 选中的功能标签
        self.selected_label = ttk.Label(control_frame, text="请选择一个功能", font=('Arial', 12))
        self.selected_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
        # 运行按钮
        self.run_button = ttk.Button(control_frame, text="运行", command=self.run_selected, state='disabled')
        self.run_button.grid(row=1, column=0, padx=5)
        
        # 停止按钮
        self.stop_button = ttk.Button(control_frame, text="停止所有", command=self.stop_all)
        self.stop_button.grid(row=1, column=1, padx=5)
        
        # 清空输出按钮
        self.clear_button = ttk.Button(control_frame, text="清空输出", command=self.clear_output)
        self.clear_button.grid(row=1, column=2, padx=5)
        
        # 输出框架
        output_frame = ttk.LabelFrame(right_frame, text="输出信息", padding="10")
        output_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)
        
        # 输出文本框
        self.output_text = scrolledtext.ScrolledText(output_frame, wrap=tk.WORD, width=60, height=20)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置文本标签
        self.output_text.tag_config("info", foreground="blue")
        self.output_text.tag_config("error", foreground="red")
        self.output_text.tag_config("success", foreground="green")
        
        # 底部状态栏
        status_frame = ttk.Frame(self.root)
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), padx=10, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="就绪", relief=tk.SUNKEN)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
    def populate_tree(self):
        """填充树形视图"""
        for category, items in self.test_categories.items():
            category_id = self.tree.insert('', 'end', text=category, values=())
            for num, name, desc in items:
                self.tree.insert(category_id, 'end', text=desc, values=(num, name))
        
        # 展开所有分类
        for item in self.tree.get_children():
            self.tree.item(item, open=True)
    
    def on_tree_select(self, event):
        """树形视图选择事件"""
        selected = self.tree.selection()
        if selected:
            item = self.tree.item(selected[0])
            values = item['values']
            if values:  # 如果是功能项（不是分类）
                self.selected_label.config(text=f"选中: {item['text']} (编号: {values[0]})")
                self.run_button.config(state='normal')
                self.current_selection = values
            else:
                self.selected_label.config(text="请选择一个功能")
                self.run_button.config(state='disabled')
                self.current_selection = None
    
    def run_selected(self):
        """运行选中的功能"""
        if not hasattr(self, 'current_selection') or not self.current_selection:
            return
        
        num, name = self.current_selection
        self.log_output(f"正在启动: {name} (编号: {num})\n", "info")
        self.status_label.config(text=f"正在运行: {name}")
        
        # 在新线程中运行命令
        thread = threading.Thread(target=self.execute_test, args=(num, name))
        thread.daemon = True
        thread.start()
    
    def execute_test(self, num, name):
        """执行测试脚本"""
        try:
            # 构建命令
            script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test.sh")
            cmd = f"bash {script_path} {num}"
            
            # 执行命令
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.running_processes.append(process)
            
            # 实时读取输出
            for line in process.stdout:
                self.log_output(line, "")
            
            # 等待进程结束
            process.wait()
            
            if process.returncode == 0:
                self.log_output(f"\n{name} 运行完成\n", "success")
            else:
                self.log_output(f"\n{name} 运行出错 (返回码: {process.returncode})\n", "error")
            
        except Exception as e:
            self.log_output(f"\n执行出错: {str(e)}\n", "error")
        finally:
            self.root.after(0, lambda: self.status_label.config(text="就绪"))
    
    def log_output(self, text, tag=""):
        """添加输出日志"""
        def append():
            self.output_text.insert(tk.END, text, tag)
            self.output_text.see(tk.END)
        self.root.after(0, append)
    
    def clear_output(self):
        """清空输出"""
        self.output_text.delete(1.0, tk.END)
    
    def stop_all(self):
        """停止所有进程"""
        if messagebox.askyesno("确认", "确定要停止所有正在运行的进程吗？"):
            for process in self.running_processes:
                try:
                    process.terminate()
                except:
                    pass
            
            # 同时杀死所有gnome-terminal进程
            try:
                subprocess.run("pkill -f gnome-terminal", shell=True)
                self.log_output("已停止所有进程\n", "info")
            except:
                pass
            
            self.running_processes.clear()
            self.status_label.config(text="就绪")
    
    def on_closing(self):
        """关闭窗口时的处理"""
        if self.running_processes:
            if messagebox.askyesno("确认", "还有进程在运行，确定要退出吗？"):
                self.stop_all()
                self.root.destroy()
        else:
            self.root.destroy()

def main():
    root = tk.Tk()
    app = JetAutoTestGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main() 