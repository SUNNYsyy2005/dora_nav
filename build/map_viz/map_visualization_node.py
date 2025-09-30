#!/usr/bin/env python3
"""
统一的地图管理+可视化节点
功能：
1. 自动加载和管理地图配置
2. 向其他节点提供统一的地图信息
3. 实时可视化显示
4. 用户交互设置初始位姿和目标点
5. 显示机器人状态、路径规划等
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import json
import time
import threading
import queue
from datetime import datetime
import yaml
import os
from pathlib import Path

# Dora imports
from dora import Node

# 项目路径管理
from project_paths import ProjectPaths

class MapVisualizationNode:
    def __init__(self, node_id):
        self.node = Node(node_id)
        self.running = True
        
        # 地图配置和数据
        self.map_config = {}
        self.map_data = None
        self.map_loaded = False
        
        # 机器人状态
        self.robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}  # AMCL估计位姿
        self.imu_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}  # IMU预估位姿
        self.fused_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "confidence": 0.0}  # 融合位姿
        self.true_robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}  # 真实位姿（仿真）
        self.target_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.laser_scan = None
        self.global_path = []
        self.planning_status = "IDLE"
        
        # 交互状态
        self.setting_initial_pose = False
        self.setting_goal = False
        self.goal_counter = 0
        
        # 初始位姿设置的两步状态
        self.initial_pose_step = 0  # 0: 不在设置模式, 1: 已设置位置等待朝向, 2: 完成
        self.initial_pose_position = None  # 存储第一次点击的位置
        self.initial_pose_temp_circle = None  # 临时显示第一步设置的位置
        
        # 数据队列
        self.update_queue = queue.Queue()
        
        # 输出消息队列 - 用于线程安全的消息发送
        self.output_queue = queue.Queue()
        
        # 初始化地图
        self.load_map_config()
        
        print(f"[{datetime.now()}] [MAP_VIZ] [INFO] 地图可视化节点已启动")
    
    def find_map_config_files(self):
        """搜索可用的地图配置文件（使用标准路径管理）"""
        # 使用项目路径管理器获取标准搜索路径
        search_paths = ProjectPaths.get_standard_map_search_paths()
        
        found_files = []
        for path_str in search_paths:
            path = Path(path_str)
            if path.exists():
                found_files.append(path)
        
        # 按修改时间排序，最新的在前
        found_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
        
        print(f"[MAP_VIZ] 🔍 搜索到 {len(found_files)} 个地图配置文件")
        for i, file_path in enumerate(found_files, 1):
            mtime = file_path.stat().st_mtime
            mtime_str = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
            print(f"[MAP_VIZ]   {i}. {file_path} (修改时间: {mtime_str})")
        
        return found_files
    
    def load_map_config(self):
        """加载地图配置"""
        try:
            config_files = self.find_map_config_files()
            
            for config_file in config_files:
                print(f"[MAP_VIZ] 尝试加载地图配置: {config_file}")
                
                try:
                    with open(config_file, 'r') as f:
                        self.map_config = yaml.safe_load(f)
                    
                    # 验证必要字段
                    required_fields = ['image', 'resolution']
                    if all(field in self.map_config for field in required_fields):
                        # 使用项目路径管理器查找地图图像
                        map_image_path = ProjectPaths.find_map_image_for_yaml(str(config_file))
                        
                        if map_image_path and Path(map_image_path).exists():
                            self.map_data = self.load_pgm(map_image_path)
                            if self.map_data is not None:
                                self.map_loaded = True
                                self.map_config['config_file'] = str(config_file)
                                self.map_config['map_dir'] = str(config_file.parent)
                                self.map_config['image_file'] = map_image_path
                                
                                print(f"[MAP_VIZ] ✅ 地图加载成功")
                                print(f"[MAP_VIZ]   配置文件: {config_file}")
                                print(f"[MAP_VIZ]   图像文件: {map_image_path}")
                                print(f"[MAP_VIZ]   分辨率: {self.map_config['resolution']} m/pixel")
                                print(f"[MAP_VIZ]   尺寸: {self.map_config.get('width', 'auto')}×{self.map_config.get('height', 'auto')} pixels")
                                
                                # 发布地图配置信息
                                self.publish_map_info()
                                return
                        else:
                            print(f"[MAP_VIZ] ⚠️ 地图图像文件未找到，配置文件: {config_file}")
                            if 'image' in self.map_config:
                                print(f"[MAP_VIZ]   期望的图像文件: {self.map_config['image']}")
                    
                except Exception as e:
                    print(f"[MAP_VIZ] 加载配置文件失败 {config_file}: {e}")
                    continue
            
            print("[MAP_VIZ] ⚠️ 未找到有效的地图配置，创建默认地图")
            self.create_default_map()
            
        except Exception as e:
            print(f"[MAP_VIZ] 地图配置加载错误: {e}")
            self.create_default_map()
    
    def load_pgm(self, filename):
        """加载PGM格式地图文件"""
        try:
            with open(filename, 'rb') as f:
                # 读取头部
                header = f.readline().decode('ascii').strip()
                if header not in ['P2', 'P5']:
                    raise ValueError(f"不支持的PGM格式: {header}")
                
                # 跳过注释行
                line = f.readline().decode('ascii').strip()
                while line.startswith('#'):
                    line = f.readline().decode('ascii').strip()
                
                # 读取宽度和高度
                width, height = map(int, line.split())
                
                # 读取最大值
                max_val = int(f.readline().decode('ascii').strip())
                
                # 读取图像数据
                if header == 'P2':  # ASCII
                    data = []
                    for line in f:
                        data.extend(map(int, line.split()))
                    data = np.array(data, dtype=np.uint8)
                else:  # P5 - 二进制
                    data = np.frombuffer(f.read(), dtype=np.uint8)
                
                # 重塑为图像
                image = data.reshape((height, width))
                
                # 转换为0-1范围，0=障碍物，1=自由空间
                # PGM格式：0=黑色（障碍物），255=白色（自由空间）
                map_data = image / 255.0
                
                print(f"[MAP_VIZ] 加载PGM地图: {width}x{height}, 数据范围: {map_data.min():.3f}-{map_data.max():.3f}")
                print(f"[MAP_VIZ] 障碍物像素数: {np.sum(map_data < 0.5)}")
                print(f"[MAP_VIZ] 自由空间像素数: {np.sum(map_data > 0.5)}")
                
                # 更新配置中的尺寸信息
                self.map_config['width'] = width
                self.map_config['height'] = height
                
                return map_data
                
        except Exception as e:
            print(f"[MAP_VIZ] PGM加载错误: {e}")
            return None
    
    def create_default_map(self):
        """创建默认地图"""
        self.map_config = {
            'image': 'default_map.pgm',
            'resolution': 0.005,  # 5mm/pixel
            'width': 800,
            'height': 800,
            'origin': [0.0, 0.0, 0.0],
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'negate': 0,
            'map_size_meters': 4.0,
            'config_file': 'default',
            'map_dir': '/home/sunny/dora_nav'
        }
        
        # 创建800x800的默认地图
        self.map_data = np.ones((800, 800), dtype=np.float32) * 0.8  # 灰色背景
        
        # 添加边界
        self.map_data[:5, :] = 0.0   # 上边界
        self.map_data[-5:, :] = 0.0  # 下边界
        self.map_data[:, :5] = 0.0   # 左边界
        self.map_data[:, -5:] = 0.0  # 右边界
        
        self.map_loaded = True
        print("[MAP_VIZ] 已创建默认地图")
        
        # 发布地图配置信息
        self.publish_map_info()
    
    def publish_map_info(self):
        """发布地图配置信息给其他节点"""
        try:
            map_info_msg = {
                "header": {
                    "seq": 1,
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "map_file": self.map_config.get('image', ''),
                "resolution": self.map_config.get('resolution', 0.005),
                "width": self.map_config.get('width', 800),
                "height": self.map_config.get('height', 800),
                "origin": self.map_config.get('origin', [0.0, 0.0, 0.0]),
                "occupied_thresh": self.map_config.get('occupied_thresh', 0.65),
                "free_thresh": self.map_config.get('free_thresh', 0.196),
                "negate": self.map_config.get('negate', False),
                "map_size_meters": self.map_config.get('map_size_meters', 4.0),
                "config_source": self.map_config.get('config_file', 'unknown')
            }
            
            map_info_json = json.dumps(map_info_msg)
            # 将消息放入输出队列，由Dora线程发送
            self.output_queue.put(("map_info", map_info_json.encode()))
            
            print(f"[MAP_VIZ] 📡 地图配置信息已发布")
            
        except Exception as e:
            print(f"[MAP_VIZ] 发布地图信息错误: {e}")
    
    def world_to_pixel(self, x, y):
        """世界坐标转换为像素坐标"""
        resolution = self.map_config.get('resolution', 0.005)
        width = self.map_config.get('width', 800)
        height = self.map_config.get('height', 800)
        origin = self.map_config.get('origin', [0.0, 0.0, 0.0])
        
        # 如果原点在地图中心 (0,0,0)
        if origin[0] == 0.0 and origin[1] == 0.0:
            # 地图中心为 (400, 400)
            # 世界坐标 x=0 对应像素 x=400
            # 世界坐标 y=0 对应像素 y=400
            pixel_x = int(width // 2 + x / resolution)
            pixel_y = int(height // 2 + y / resolution)  # Y轴不翻转：世界坐标y+对应像素y+
        else:
            # 传统左下角原点
            pixel_x = int((x - origin[0]) / resolution)
            pixel_y = int((y - origin[1]) / resolution)
        
        return pixel_x, pixel_y
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """像素坐标转换为世界坐标"""
        resolution = self.map_config.get('resolution', 0.005)
        width = self.map_config.get('width', 800)
        height = self.map_config.get('height', 800)
        origin = self.map_config.get('origin', [0.0, 0.0, 0.0])
        
        # 如果原点在地图中心
        if origin[0] == 0.0 and origin[1] == 0.0:
            x = (pixel_x - width // 2) * resolution
            y = (pixel_y - height // 2) * resolution  # Y轴不翻转
        else:
            x = pixel_x * resolution + origin[0]
            y = pixel_y * resolution + origin[1]
        
        return x, y
    
    def send_goal(self, x, y, theta=0.0):
        """发送目标点"""
        try:
            self.goal_counter += 1
            goal_id = f"goal_{self.goal_counter}_{int(time.time())}"
            
            goal_msg = {
                "header": {
                    "seq": self.goal_counter,
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "target_pose": {
                    "x": x,
                    "y": y,
                    "theta": theta
                },
                "goal_id": goal_id
            }
            
            goal_json = json.dumps(goal_msg)
            # 将消息放入输出队列，由Dora线程发送
            self.output_queue.put(("goal", goal_json.encode()))
            
            self.target_pose = {"x": x, "y": y, "theta": theta}
            
            # 立即更新目标点可视化
            target_px, target_py = self.world_to_pixel(x, y)
            self.target_circle.center = (target_px, target_py)
            
            # 更新目标点方向箭头
            arrow_length = 20
            dx = arrow_length * np.cos(theta)
            dy = arrow_length * np.sin(theta)  # Y轴不翻转
            
            self.target_arrow.remove()
            self.target_arrow = patches.FancyArrow(target_px, target_py, dx, dy, width=4, color='red', alpha=0.8)
            self.ax.add_patch(self.target_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            
            print(f"[MAP_VIZ] 🎯 发送目标点: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            print(f"[MAP_VIZ] 🎯 目标点位置已更新到像素坐标: ({target_px}, {target_py})")
            
        except Exception as e:
            print(f"[MAP_VIZ] 发送目标点错误: {e}")
    
    def send_initial_pose(self, x, y, theta=0.0):
        """发送初始位姿"""
        try:
            # 使用与geometry_msgs::Pose2D兼容的格式
            initial_pose_msg = {
                "x": x,
                "y": y,
                "theta": theta
            }
            
            pose_json = json.dumps(initial_pose_msg)
            # 将消息放入输出队列，由Dora线程发送
            self.output_queue.put(("initial_pose", pose_json.encode()))
            
            self.robot_pose = {"x": x, "y": y, "theta": theta}
            
            # 立即更新机器人可视化
            robot_px, robot_py = self.world_to_pixel(x, y)
            self.robot_circle.center = (robot_px, robot_py)
            
            # 更新方向箭头
            arrow_length = 25
            dx = arrow_length * np.cos(theta)
            dy = arrow_length * np.sin(theta)  # Y轴不翻转
            
            self.robot_arrow.remove()
            self.robot_arrow = patches.FancyArrow(robot_px, robot_py, dx, dy, width=6, color='blue', alpha=0.8)
            self.ax.add_patch(self.robot_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            
            print(f"[MAP_VIZ] 🤖 发送初始位姿: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            print(f"[MAP_VIZ] 🤖 机器人位置已更新到像素坐标: ({robot_px}, {robot_py})")
            
        except Exception as e:
            print(f"[MAP_VIZ] 发送初始位姿错误: {e}")
    
    def setup_plot(self):
        """设置matplotlib图形"""
        # 设置中文字体支持
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Microsoft YaHei']
        plt.rcParams['axes.unicode_minus'] = False
        
        self.fig, self.ax = plt.subplots(figsize=(14, 12))
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Navigation - Map Management + Visualization', fontsize=16)
        
        # 显示地图
        if self.map_data is not None:
            self.map_plot = self.ax.imshow(self.map_data, cmap='gray', origin='lower', alpha=0.8)
            
        # 初始化图形元素 (使用英文标签避免字体问题)
        # AMCL估计位姿（蓝色）
        self.robot_circle = plt.Circle((400, 400), 10, color='blue', alpha=0.8, label='Robot (AMCL)')
        self.robot_arrow = patches.FancyArrow(400, 400, 15, 0, width=4, color='blue', alpha=0.8)
        
        # IMU预估位姿（黄色）
        self.imu_circle = plt.Circle((400, 400), 8, color='yellow', alpha=0.7, label='IMU Pose')
        self.imu_arrow = patches.FancyArrow(400, 400, 15, 0, width=4, color='yellow', alpha=0.7)
        
        # 融合位姿（紫色）
        self.fused_circle = plt.Circle((400, 400), 10, color='purple', alpha=0.9, label='Fused Pose')
        self.fused_arrow = patches.FancyArrow(400, 400, 18, 0, width=5, color='purple', alpha=0.9)
        
        # 真实位姿（绿色）
        self.true_robot_circle = plt.Circle((400, 400), 8, color='green', alpha=0.8, label='True Robot')
        self.true_robot_arrow = patches.FancyArrow(400, 400, 12, 0, width=3, color='green', alpha=0.8)
        
        # 目标点（红色）
        self.target_circle = plt.Circle((400, 300), 8, color='red', alpha=0.8, label='Goal')
        self.target_arrow = patches.FancyArrow(400, 300, 12, 0, width=3, color='red', alpha=0.8)
        
        # 路径和激光扫描
        self.path_line, = self.ax.plot([], [], 'g-', linewidth=3, alpha=0.8, label='Global Path')
        self.laser_points = self.ax.scatter([], [], c='cyan', s=3, alpha=0.7, label='Laser Scan')
        
        self.ax.add_patch(self.robot_circle)
        self.ax.add_patch(self.robot_arrow)
        self.ax.add_patch(self.imu_circle)
        self.ax.add_patch(self.imu_arrow)
        self.ax.add_patch(self.fused_circle)
        self.ax.add_patch(self.fused_arrow)
        self.ax.add_patch(self.true_robot_circle)
        self.ax.add_patch(self.true_robot_arrow)
        self.ax.add_patch(self.target_circle)
        self.ax.add_patch(self.target_arrow)
        
        # 地图信息显示 (使用英文避免字体问题)
        map_info_text = f"Map Info:\n"
        map_info_text += f"Resolution: {self.map_config.get('resolution', 0.005)} m/pixel\n"
        map_info_text += f"Size: {self.map_config.get('width', 800)}x{self.map_config.get('height', 800)} pixels\n"
        map_info_text += f"Physical Size: {self.map_config.get('map_size_meters', 4.0)} m\n"
        map_info_text += f"Config Source: {os.path.basename(self.map_config.get('config_file', 'default'))}"
        
        self.ax.text(10, self.map_config.get('height', 800) - 10, map_info_text, 
                    fontsize=10, color='black', verticalalignment='top',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8))
        
        # 操作说明 (使用英文避免字体问题)
        operation_text = "Controls:\n"
        operation_text += "Press 'i' -> Set Initial Pose\n"
        operation_text += "Press 'g' -> Set Goal\n"
        operation_text += "Left Click -> Confirm Position\n"
        operation_text += "Press 'Esc' -> Cancel Setting\n"
        operation_text += "Press 'r' -> Republish Map Info"
        
        self.ax.text(10, 150, operation_text, fontsize=10, color='black',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.8))
        
        # 状态显示
        self.status_text = self.ax.text(10, 50, '', fontsize=11, color='black', weight='bold',
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
        
        # 连接事件
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        
        width = self.map_config.get('width', 800)
        height = self.map_config.get('height', 800)
        self.ax.set_xlim(0, width)
        self.ax.set_ylim(0, height)
        self.ax.legend(loc='upper right')
        
        plt.tight_layout()
    
    def on_key_press(self, event):
        """键盘事件处理"""
        if event.key == 'i':
            self.setting_initial_pose = True
            self.setting_goal = False
            self.initial_pose_step = 0
            self.initial_pose_position = None
            if self.initial_pose_temp_circle:
                self.initial_pose_temp_circle.remove()
                self.initial_pose_temp_circle = None
            self.status_text.set_text("Step 1: Click to set robot position")
            print("[MAP_VIZ] 进入初始位姿设置模式（两步设置：位置+方向）")
            
        elif event.key == 'g':
            self.setting_goal = True
            self.setting_initial_pose = False
            self.status_text.set_text("🎯 点击地图设置目标点")
            print("[MAP_VIZ] 进入目标点设置模式")
            
        elif event.key == 'escape':
            self.setting_initial_pose = False
            self.setting_goal = False
            self.initial_pose_step = 0
            self.initial_pose_position = None
            if self.initial_pose_temp_circle:
                self.initial_pose_temp_circle.remove()
                self.initial_pose_temp_circle = None
            self.status_text.set_text("Ready - Press 'i' for initial pose, 'g' for goal")
            print("[MAP_VIZ] 取消设置模式")
            
        elif event.key == 'r':
            self.publish_map_info()
            self.status_text.set_text("📡 地图信息已重新发布")
        
        self.fig.canvas.draw()
    
    def on_mouse_click(self, event):
        """鼠标点击事件处理"""
        if event.inaxes != self.ax:
            return
            
        if event.button == 1:  # 左键
            pixel_x, pixel_y = int(event.xdata), int(event.ydata)
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
            
            if self.setting_initial_pose:
                if self.initial_pose_step == 0:
                    # 第一步：设置位置
                    self.initial_pose_position = (world_x, world_y)
                    self.initial_pose_step = 1
                    self.status_text.set_text("Step 2: Click to set robot direction")
                    print(f"[MAP_VIZ] 初始位姿位置已设置: ({world_x:.3f}, {world_y:.3f})")
                    
                    # 显示临时位置标记
                    if self.initial_pose_temp_circle:
                        self.initial_pose_temp_circle.remove()
                    px, py = self.world_to_pixel(world_x, world_y)
                    self.initial_pose_temp_circle = patches.Circle((px, py), 8, color='orange', alpha=0.7)
                    self.ax.add_patch(self.initial_pose_temp_circle)
                elif self.initial_pose_step == 1:
                    # 第二步：设置朝向
                    pos_x, pos_y = self.initial_pose_position
                    theta = np.arctan2(world_y - pos_y, world_x - pos_x)
                    self.send_initial_pose(pos_x, pos_y, theta)
                    self.setting_initial_pose = False
                    self.initial_pose_step = 0
                    self.initial_pose_position = None
                    self.status_text.set_text(f"Initial pose set: ({pos_x:.3f}, {pos_y:.3f}, {theta:.3f})")
                    print(f"[MAP_VIZ] 初始位姿已设置: ({pos_x:.3f}, {pos_y:.3f}, {theta:.3f})")
                    
                    # 清除临时标记
                    if self.initial_pose_temp_circle:
                        self.initial_pose_temp_circle.remove()
                        self.initial_pose_temp_circle = None
                
            elif self.setting_goal:
                self.send_goal(world_x, world_y, 0.0)
                self.setting_goal = False
                self.status_text.set_text(f"🎯 目标点已设置: ({world_x:.3f}, {world_y:.3f})")
            
            self.fig.canvas.draw()
    
    def update_robot_pose(self, pose_data):
        """更新机器人位姿"""
        try:
            # 处理PyArrow数组数据
            if hasattr(pose_data, 'to_pylist'):
                # 如果是PyArrow数组，转换为JSON字符串
                try:
                    data_list = pose_data.to_pylist()
                    pose_data = ''.join(chr(x) for x in data_list)
                except Exception as e:
                    print(f"[MAP_VIZ] PyArrow数据转换错误: {e}")
                    pose_data = str(pose_data)
            
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.robot_pose["x"] = pose_data.get("x", 0.0)
            self.robot_pose["y"] = pose_data.get("y", 0.0) 
            self.robot_pose["theta"] = pose_data.get("theta", 0.0)
            
            print(f"[MAP_VIZ] Robot pose updated: x={self.robot_pose['x']:.3f}, y={self.robot_pose['y']:.3f}, theta={self.robot_pose['theta']:.3f}")
            
            # 转换为像素坐标
            px, py = self.world_to_pixel(self.robot_pose["x"], self.robot_pose["y"])
            print(f"[MAP_VIZ] Robot pixel coordinates: px={px}, py={py}")
            
            # 更新机器人图标
            self.robot_circle.center = (px, py)
            
            # 更新方向箭头
            arrow_length = 25
            dx = arrow_length * np.cos(self.robot_pose["theta"])
            dy = arrow_length * np.sin(self.robot_pose["theta"])  # Y轴不翻转
            
            self.robot_arrow.remove()
            self.robot_arrow = patches.FancyArrow(px, py, dx, dy, width=6, color='blue', alpha=0.8)
            self.ax.add_patch(self.robot_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新机器人位姿错误: {e}")
    
    def update_imu_pose(self, pose_data):
        """更新IMU预估位姿"""
        try:
            # 处理PyArrow数组数据
            if hasattr(pose_data, 'to_pylist'):
                try:
                    data_list = pose_data.to_pylist()
                    pose_data = ''.join(chr(x) for x in data_list)
                except Exception as e:
                    print(f"[MAP_VIZ] PyArrow数据转换错误: {e}")
                    pose_data = str(pose_data)
            
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.imu_pose["x"] = pose_data.get("x", 0.0)
            self.imu_pose["y"] = pose_data.get("y", 0.0) 
            self.imu_pose["theta"] = pose_data.get("theta", 0.0)
            
            print(f"[MAP_VIZ] IMU pose updated: x={self.imu_pose['x']:.3f}, y={self.imu_pose['y']:.3f}, theta={self.imu_pose['theta']:.3f}")
            
            # 转换为像素坐标
            px, py = self.world_to_pixel(self.imu_pose["x"], self.imu_pose["y"])
            
            # 更新IMU位姿图标
            self.imu_circle.center = (px, py)
            
            # 更新方向箭头
            arrow_length = 20
            dx = arrow_length * np.cos(self.imu_pose["theta"])
            dy = arrow_length * np.sin(self.imu_pose["theta"])  # Y轴不翻转
            
            self.imu_arrow.remove()
            self.imu_arrow = patches.FancyArrow(px, py, dx, dy, width=4, color='yellow', alpha=0.7)
            self.ax.add_patch(self.imu_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新IMU位姿错误: {e}")
    
    def update_fused_pose(self, pose_data):
        """更新融合位姿"""
        try:
            # 处理PyArrow数组数据
            if hasattr(pose_data, 'to_pylist'):
                try:
                    data_list = pose_data.to_pylist()
                    pose_data = ''.join(chr(x) for x in data_list)
                except Exception as e:
                    print(f"[MAP_VIZ] PyArrow数据转换错误: {e}")
                    pose_data = str(pose_data)
            
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.fused_pose["x"] = pose_data.get("x", 0.0)
            self.fused_pose["y"] = pose_data.get("y", 0.0) 
            self.fused_pose["theta"] = pose_data.get("theta", 0.0)
            self.fused_pose["confidence"] = pose_data.get("confidence", 0.0)
            
            print(f"[MAP_VIZ] Fused pose updated: x={self.fused_pose['x']:.3f}, y={self.fused_pose['y']:.3f}, theta={self.fused_pose['theta']:.3f}, conf={self.fused_pose['confidence']:.3f}")
            
            # 转换为像素坐标
            px, py = self.world_to_pixel(self.fused_pose["x"], self.fused_pose["y"])
            
            # 更新融合位姿图标
            self.fused_circle.center = (px, py)
            
            # 更新方向箭头
            arrow_length = 22
            dx = arrow_length * np.cos(self.fused_pose["theta"])
            dy = arrow_length * np.sin(self.fused_pose["theta"])  # Y轴不翻转
            
            self.fused_arrow.remove()
            self.fused_arrow = patches.FancyArrow(px, py, dx, dy, width=5, color='purple', alpha=0.9)
            self.ax.add_patch(self.fused_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新融合位姿错误: {e}")
    
    def update_true_robot_pose(self, pose_data):
        """更新真实机器人位姿（仿真）"""
        try:
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.true_robot_pose["x"] = pose_data.get("x", 0.0)
            self.true_robot_pose["y"] = pose_data.get("y", 0.0) 
            self.true_robot_pose["theta"] = pose_data.get("theta", 0.0)
            
            print(f"[MAP_VIZ] True robot pose updated: x={self.true_robot_pose['x']:.3f}, y={self.true_robot_pose['y']:.3f}, theta={self.true_robot_pose['theta']:.3f}")
            
            # 转换为像素坐标
            px, py = self.world_to_pixel(self.true_robot_pose["x"], self.true_robot_pose["y"])
            print(f"[MAP_VIZ] True robot pixel coordinates: px={px}, py={py}")
            
            # 更新真实机器人图标
            self.true_robot_circle.center = (px, py)
            
            # 更新方向箭头
            arrow_length = 20
            dx = arrow_length * np.cos(self.true_robot_pose["theta"])
            dy = arrow_length * np.sin(self.true_robot_pose["theta"])  # Y轴不翻转
            
            self.true_robot_arrow.remove()
            self.true_robot_arrow = patches.FancyArrow(px, py, dx, dy, width=4, color='green', alpha=0.8)
            self.ax.add_patch(self.true_robot_arrow)
            
            # 强制重绘
            self.fig.canvas.draw_idle()
            print(f"[MAP_VIZ] 真实机器人位姿可视化已更新")
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新真实机器人位姿错误: {e}")
    
    def update_global_path(self, path_data):
        """更新全局路径"""
        try:
            # 处理PyArrow数组数据
            if hasattr(path_data, 'to_pylist'):
                try:
                    data_list = path_data.to_pylist()
                    path_data = ''.join(chr(x) for x in data_list)
                except Exception as e:
                    print(f"[MAP_VIZ] PyArrow数据转换错误: {e}")
                    path_data = str(path_data)
            
            if isinstance(path_data, str):
                path_data = json.loads(path_data)
            
            path_x = []
            path_y = []
            
            for pose in path_data.get("poses", []):
                x = pose["pose"]["position"]["x"]
                y = pose["pose"]["position"]["y"]
                px, py = self.world_to_pixel(x, y)
                path_x.append(px)
                path_y.append(py)
            
            self.path_line.set_data(path_x, path_y)
            print(f"[MAP_VIZ] 更新全局路径: {len(path_x)}个点")
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新全局路径错误: {e}")
            import traceback
            traceback.print_exc()
    
    def update_laser_scan(self, scan_data):
        """更新激光雷达数据"""
        try:
            if isinstance(scan_data, str):
                scan_data = json.loads(scan_data)
            
            ranges = scan_data.get("ranges", [])
            angle_min = scan_data.get("angle_min", -np.pi)
            angle_increment = scan_data.get("angle_increment", 0.01)
            
            print(f"[MAP_VIZ] 收到激光扫描数据: {len(ranges)} 个点")
            
            # 获取机器人当前位置（使用真实位姿）
            robot_px, robot_py = self.world_to_pixel(self.true_robot_pose["x"], self.true_robot_pose["y"])
            
            laser_x = []
            laser_y = []
            
            angle = angle_min
            valid_points = 0
            for range_val in ranges:
                if not np.isnan(range_val) and 0.1 < range_val < 10.0:  # 合理的距离范围
                    # 激光点在机器人坐标系中的位置（使用真实位姿）
                    local_x = range_val * np.cos(angle + self.true_robot_pose["theta"])
                    local_y = range_val * np.sin(angle + self.true_robot_pose["theta"])
                    
                    # 转换到世界坐标（使用真实位姿）
                    world_x = self.true_robot_pose["x"] + local_x
                    world_y = self.true_robot_pose["y"] + local_y
                    
                    # 转换到像素坐标
                    px, py = self.world_to_pixel(world_x, world_y)
                    
                    # 检查像素坐标是否在地图范围内
                    width = self.map_config.get('width', 800)
                    height = self.map_config.get('height', 800)
                    if 0 <= px < width and 0 <= py < height:
                        laser_x.append(px)
                        laser_y.append(py)
                        valid_points += 1
                
                angle += angle_increment
            
            print(f"[MAP_VIZ] 有效激光点: {valid_points}/{len(ranges)}")
            
            # 更新激光点云显示
            if laser_x and laser_y:
                self.laser_points.set_offsets(np.column_stack([laser_x, laser_y]))
                print(f"[MAP_VIZ] 激光点云已更新，显示 {len(laser_x)} 个点")
            else:
                # 如果没有有效点，清空显示
                self.laser_points.set_offsets(np.empty((0, 2)))
                print(f"[MAP_VIZ] 没有有效激光点，清空显示")
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新激光雷达错误: {e}")
            import traceback
            traceback.print_exc()
    
    def update_planning_status(self, status_data):
        """更新规划状态"""
        try:
            if isinstance(status_data, str):
                status_data = json.loads(status_data)
            
            status_code = status_data.get("status", 0)
            message = status_data.get("message", "")
            goal_id = status_data.get("goal_id", "")
            
            status_names = {0: "空闲", 1: "规划中", 2: "成功", 3: "失败"}
            status_name = status_names.get(status_code, "未知")
            
            self.planning_status = status_name
            status_text = f"🧠 规划状态: {status_name}"
            if message:
                status_text += f" - {message}"
            if goal_id:
                status_text += f" (ID: {goal_id[-8:]})"  # 只显示ID的后8位
            
            self.status_text.set_text(status_text)
            print(f"[MAP_VIZ] 规划状态更新: {status_text}")
            
        except Exception as e:
            print(f"[MAP_VIZ] 更新规划状态错误: {e}")
    
    def dora_loop(self):
        """Dora消息处理循环"""
        print("[MAP_VIZ] Dora消息循环已启动")
        
        # 启动时发布一次地图信息
        time.sleep(1)  # 等待其他节点启动
        self.publish_map_info()
        
        while self.running:
            try:
                # 处理输出队列中的消息
                while not self.output_queue.empty():
                    try:
                        output_id, output_data = self.output_queue.get_nowait()
                        self.node.send_output(output_id, output_data)
                        print(f"[MAP_VIZ] ✅ 已发送消息: {output_id}")
                    except queue.Empty:
                        break
                    except Exception as e:
                        print(f"[MAP_VIZ] 发送消息错误: {e}")
                
                event = self.node.next()
                
                if event is None:
                    continue
                
                if event["type"] == "INPUT":
                    input_id = event["id"]
                    data = event["value"]
                    #if isinstance(data, bytes):
                    #    data = data.decode('utf-8')
                    if input_id == "pose":
                        print(f"[MAP_VIZ] 收到AMCL机器人位姿数据: {data}")
                        self.update_queue.put(("pose", data))
                    elif input_id == "imu_pose":
                        print(f"[MAP_VIZ] 收到IMU位姿数据: {data}")
                        self.update_queue.put(("imu_pose", data))
                    elif input_id == "fused_pose":
                        print(f"[MAP_VIZ] 收到融合位姿数据: {data}")
                        self.update_queue.put(("fused_pose", data))
                    elif input_id == "true_robot_pose":
                        print(f"[MAP_VIZ] 收到真实机器人位姿数据: {data[:100]}")
                        # 处理PyArrow数组数据
                        if hasattr(data, 'to_pylist'):
                            # 如果是PyArrow数组，转换为JSON字符串
                            try:
                                data_list = data.to_pylist()
                                data_str = ''.join(chr(x) for x in data_list)
                                self.update_queue.put(("true_pose", data_str))
                            except Exception as e:
                                print(f"[MAP_VIZ] PyArrow数据转换错误: {e}")
                                self.update_queue.put(("true_pose", str(data)))
                        else:
                            self.update_queue.put(("true_pose", data))
                    elif input_id == "global_path":
                        print(f"[MAP_VIZ] 收到全局路径数据: {data[:100]}")
                        self.update_queue.put(("path", data))
                    elif input_id == "planning_status":
                        print(f"[MAP_VIZ] 收到规划状态数据: {data[:100]}")
                        self.update_queue.put(("status", data))
                    elif input_id == "timer" or input_id == "tick":
                        # 定期重新发布地图信息
                        if int(time.time()) % 10 == 0:  # 每10秒发布一次
                            self.publish_map_info()
                    # 特殊处理激光扫描数据（PyArrow数组格式）
                    elif input_id == "laser_scan" or input_id == "scan":
                         try:
                             # lidar_sim发送的是PyArrow数组，需要解码为JSON字符串
                             if hasattr(data, 'to_pylist'):
                                 # 转换PyArrow数组为字节列表，然后转为字符串
                                 byte_list = data.to_pylist()
                                 scan_json = bytes(byte_list).decode('utf-8')
                                 #print(f"[MAP_VIZ] 收到激光扫描数据，解码后长度: {len(scan_json)}")
                                 self.update_queue.put(("scan", scan_json))
                             else:
                                 print(f"[MAP_VIZ] 激光扫描数据格式: {type(data)}")
                                 # 如果已经是字符串，直接传递
                                 self.update_queue.put(("scan", str(data)))
                         except Exception as e:
                             print(f"[MAP_VIZ] 激光扫描数据解码错误: {e}") 
                    
                    # 解码其他数据（如果不是激光扫描）
                    if input_id not in ["laser_scan", "scan"]:
                        if isinstance(data, bytes):
                            data = data.decode('utf-8')
                    
                    # 根据输入ID处理数据
                    print(f"[MAP_VIZ] 收到输入: {input_id}, 数据长度: {len(data) if data else 0}")
                    
                        
            except Exception as e:
                print(f"[MAP_VIZ] Dora循环错误: {e}")
                time.sleep(0.1)
    
    def update_visualization(self, frame):
        """动画更新函数"""
        # 处理队列中的更新
        try:
            while not self.update_queue.empty():
                update_type, data = self.update_queue.get_nowait()
                
                if update_type == "pose":
                    self.update_robot_pose(data)
                elif update_type == "imu_pose":
                    self.update_imu_pose(data)
                elif update_type == "fused_pose":
                    self.update_fused_pose(data)
                elif update_type == "true_pose":
                    self.update_true_robot_pose(data)
                elif update_type == "path":
                    self.update_global_path(data)
                elif update_type == "status":
                    self.update_planning_status(data)
                elif update_type == "scan":
                    self.update_laser_scan(data)
                    
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[MAP_VIZ] 更新可视化错误: {e}")
        
        return [self.robot_circle, self.robot_arrow, self.true_robot_circle, self.true_robot_arrow, self.target_circle, self.target_arrow, self.path_line, self.laser_points]
    
    def run(self):
        """运行地图可视化节点"""
        try:
            # 启动Dora消息处理线程
            dora_thread = threading.Thread(target=self.dora_loop, daemon=True)
            dora_thread.start()
            
            # 设置matplotlib图形
            self.setup_plot()
            
            # 启动动画
            self.animation = FuncAnimation(
                self.fig, self.update_visualization, 
                interval=100, blit=False, cache_frame_data=False
            )
            
            print("[MAP_VIZ] 🗺️ 地图可视化界面已启动")
            print("[MAP_VIZ] 📡 地图信息已发布给其他节点")
            plt.show()
            
        except KeyboardInterrupt:
            print("[MAP_VIZ] 用户中断")
        except Exception as e:
            print(f"[MAP_VIZ] 运行错误: {e}")
        finally:
            self.running = False

def main():
    import sys
    
    node_id = "map_viz"
    
    try:
        map_viz_node = MapVisualizationNode(node_id)
        map_viz_node.run()
    except Exception as e:
        print(f"[MAP_VIZ] 主程序错误: {e}")

if __name__ == "__main__":
    main()
