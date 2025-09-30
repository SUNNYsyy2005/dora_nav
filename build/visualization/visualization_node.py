#!/usr/bin/env python3
"""
独立的可视化节点
功能：
1. 显示地图、机器人位姿、激光雷达数据
2. 手动设置初始位姿
3. 手动设置目标点
4. 显示全局路径规划结果
5. 实时更新机器人状态
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

# Dora imports
from dora import Node

class VisualizationNode:
    def __init__(self, node_id):
        self.node = Node(node_id)
        self.running = True
        
        # 数据存储
        self.map_data = None
        self.map_info = {}
        self.robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.target_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.laser_scan = None
        self.global_path = []
        self.planning_status = "IDLE"
        
        # 交互状态
        self.setting_initial_pose = False
        self.setting_goal = False
        self.goal_counter = 0
        
        # 数据队列
        self.update_queue = queue.Queue()
        
        # 初始化地图
        self.load_map()
        
        print(f"[{datetime.now()}] [VISUALIZATION] [INFO] 可视化节点已启动")
    
    def load_map(self):
        """加载地图数据"""
        try:
            # 尝试从SLAM生成的地图加载
            map_paths = [
                "/home/sunny/dora_nav/build/slam/laser_data.datslam_map.yaml",
                "/home/sunny/dora_nav/build/nav/laser_data.yaml",
                "/home/sunny/dora_nav/build/simulation_map_800x800.yaml"
            ]
            
            map_loaded = False
            for yaml_path in map_paths:
                if os.path.exists(yaml_path):
                    print(f"[VISUALIZATION] 加载地图配置: {yaml_path}")
                    
                    with open(yaml_path, 'r') as f:
                        self.map_info = yaml.safe_load(f)
                    
                    # 加载地图图像
                    map_dir = os.path.dirname(yaml_path)
                    image_file = self.map_info.get('image', 'map.pgm')
                    map_image_path = os.path.join(map_dir, image_file)
                    
                    if os.path.exists(map_image_path):
                        # 读取PGM文件
                        self.map_data = self.load_pgm(map_image_path)
                        map_loaded = True
                        print(f"[VISUALIZATION] ✅ 地图加载成功: {map_image_path}")
                        print(f"[VISUALIZATION] 地图信息: {self.map_info}")
                        break
                    else:
                        print(f"[VISUALIZATION] 地图图像不存在: {map_image_path}")
            
            if not map_loaded:
                print("[VISUALIZATION] ⚠️ 未找到地图文件，创建默认地图")
                self.create_default_map()
                
        except Exception as e:
            print(f"[VISUALIZATION] 地图加载错误: {e}")
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
                map_data = (255 - image) / 255.0
                
                return map_data
                
        except Exception as e:
            print(f"[VISUALIZATION] PGM加载错误: {e}")
            return None
    
    def create_default_map(self):
        """创建默认地图"""
        self.map_info = {
            'resolution': 0.005,  # 5mm/pixel - 与SLAM生成的地图一致
            'width': 800,
            'height': 800,
            'origin': [0.0, 0.0, 0.0],
            'map_size_meters': 4.0  # 800 * 0.005 = 4.0米
        }
        
        # 创建800x800的默认地图
        self.map_data = np.ones((800, 800), dtype=np.float32) * 0.8  # 灰色背景
        
        # 添加边界
        self.map_data[:5, :] = 0.0   # 上边界
        self.map_data[-5:, :] = 0.0  # 下边界
        self.map_data[:, :5] = 0.0   # 左边界
        self.map_data[:, -5:] = 0.0  # 右边界
        
        print("[VISUALIZATION] 已创建默认地图")
    
    def world_to_pixel(self, x, y):
        """世界坐标转换为像素坐标"""
        resolution = self.map_info.get('resolution', 0.005)
        width = self.map_info.get('width', 800)
        height = self.map_info.get('height', 800)
        
        # 假设地图中心为原点
        pixel_x = int(x / resolution + width / 2)
        pixel_y = int(-y / resolution + height / 2)  # Y轴翻转
        
        return pixel_x, pixel_y
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """像素坐标转换为世界坐标"""
        resolution = self.map_info.get('resolution', 0.005)
        width = self.map_info.get('width', 800)
        height = self.map_info.get('height', 800)
        
        x = (pixel_x - width / 2) * resolution
        y = -(pixel_y - height / 2) * resolution  # Y轴翻转
        
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
            self.node.send_output("goal", goal_json.encode())
            
            self.target_pose = {"x": x, "y": y, "theta": theta}
            print(f"[VISUALIZATION] 发送目标点: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
        except Exception as e:
            print(f"[VISUALIZATION] 发送目标点错误: {e}")
    
    def send_initial_pose(self, x, y, theta=0.0):
        """发送初始位姿"""
        try:
            initial_pose_msg = {
                "header": {
                    "seq": 1,
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "initial_pose": {
                    "x": x,
                    "y": y,
                    "theta": theta
                }
            }
            
            pose_json = json.dumps(initial_pose_msg)
            self.node.send_output("initial_pose", pose_json.encode())
            
            self.robot_pose = {"x": x, "y": y, "theta": theta}
            print(f"[VISUALIZATION] 发送初始位姿: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
        except Exception as e:
            print(f"[VISUALIZATION] 发送初始位姿错误: {e}")
    
    def setup_plot(self):
        """设置matplotlib图形"""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        self.ax.set_title('机器人导航可视化界面')
        
        # 显示地图
        if self.map_data is not None:
            self.map_plot = self.ax.imshow(self.map_data, cmap='gray', origin='upper')
            
        # 初始化图形元素
        self.robot_circle = plt.Circle((400, 400), 10, color='blue', alpha=0.7)
        self.robot_arrow = patches.FancyArrow(400, 400, 15, 0, width=3, color='blue', alpha=0.7)
        self.target_circle = plt.Circle((400, 300), 8, color='red', alpha=0.7)
        self.path_line, = self.ax.plot([], [], 'g-', linewidth=2, alpha=0.7, label='全局路径')
        
        self.ax.add_patch(self.robot_circle)
        self.ax.add_patch(self.robot_arrow)
        self.ax.add_patch(self.target_circle)
        
        # 添加交互说明
        self.ax.text(10, 780, '操作说明:', fontsize=12, color='black', 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        self.ax.text(10, 760, '1. 按 \'i\' 键设置初始位姿', fontsize=10, color='black')
        self.ax.text(10, 740, '2. 按 \'g\' 键设置目标点', fontsize=10, color='black')  
        self.ax.text(10, 720, '3. 鼠标左键点击确认位置', fontsize=10, color='black')
        
        # 状态显示
        self.status_text = self.ax.text(10, 50, '', fontsize=10, color='black',
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
        
        # 连接事件
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        
        self.ax.set_xlim(0, 800)
        self.ax.set_ylim(0, 800)
        self.ax.legend()
        
        plt.tight_layout()
    
    def on_key_press(self, event):
        """键盘事件处理"""
        if event.key == 'i':
            self.setting_initial_pose = True
            self.setting_goal = False
            self.status_text.set_text("点击地图设置初始位姿")
            print("[VISUALIZATION] 进入初始位姿设置模式")
            
        elif event.key == 'g':
            self.setting_goal = True
            self.setting_initial_pose = False
            self.status_text.set_text("点击地图设置目标点")
            print("[VISUALIZATION] 进入目标点设置模式")
            
        elif event.key == 'escape':
            self.setting_initial_pose = False
            self.setting_goal = False
            self.status_text.set_text("取消设置")
            print("[VISUALIZATION] 取消设置模式")
        
        self.fig.canvas.draw()
    
    def on_mouse_click(self, event):
        """鼠标点击事件处理"""
        if event.inaxes != self.ax:
            return
            
        if event.button == 1:  # 左键
            pixel_x, pixel_y = int(event.xdata), int(event.ydata)
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
            
            if self.setting_initial_pose:
                self.send_initial_pose(world_x, world_y, 0.0)
                self.setting_initial_pose = False
                self.status_text.set_text(f"初始位姿已设置: ({world_x:.3f}, {world_y:.3f})")
                
            elif self.setting_goal:
                self.send_goal(world_x, world_y, 0.0)
                self.setting_goal = False
                self.status_text.set_text(f"目标点已设置: ({world_x:.3f}, {world_y:.3f})")
            
            self.fig.canvas.draw()
    
    def update_robot_pose(self, pose_data):
        """更新机器人位姿"""
        try:
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.robot_pose["x"] = pose_data.get("x", 0.0)
            self.robot_pose["y"] = pose_data.get("y", 0.0) 
            self.robot_pose["theta"] = pose_data.get("theta", 0.0)
            
            # 转换为像素坐标
            px, py = self.world_to_pixel(self.robot_pose["x"], self.robot_pose["y"])
            
            # 更新机器人图标
            self.robot_circle.center = (px, py)
            
            # 更新方向箭头
            arrow_length = 20
            dx = arrow_length * np.cos(self.robot_pose["theta"])
            dy = -arrow_length * np.sin(self.robot_pose["theta"])  # Y轴翻转
            
            self.robot_arrow.remove()
            self.robot_arrow = patches.FancyArrow(px, py, dx, dy, width=5, color='blue', alpha=0.7)
            self.ax.add_patch(self.robot_arrow)
            
        except Exception as e:
            print(f"[VISUALIZATION] 更新机器人位姿错误: {e}")
    
    def update_global_path(self, path_data):
        """更新全局路径"""
        try:
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
            print(f"[VISUALIZATION] 更新全局路径: {len(path_x)}个点")
            
        except Exception as e:
            print(f"[VISUALIZATION] 更新全局路径错误: {e}")
    
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
            status_text = f"规划状态: {status_name} - {message}"
            if goal_id:
                status_text += f" (ID: {goal_id})"
            
            self.status_text.set_text(status_text)
            print(f"[VISUALIZATION] 规划状态更新: {status_text}")
            
        except Exception as e:
            print(f"[VISUALIZATION] 更新规划状态错误: {e}")
    
    def dora_loop(self):
        """Dora消息处理循环"""
        print("[VISUALIZATION] Dora消息循环已启动")
        
        while self.running:
            try:
                event = self.node.next()
                
                if event is None:
                    continue
                
                if event["type"] == "INPUT":
                    input_id = event["id"]
                    data = event["data"]
                    
                    # 解码数据
                    if isinstance(data, bytes):
                        data = data.decode('utf-8')
                    
                    # 根据输入ID处理数据
                    if input_id == "pose":
                        self.update_queue.put(("pose", data))
                    elif input_id == "global_path":
                        self.update_queue.put(("path", data))
                    elif input_id == "planning_status":
                        self.update_queue.put(("status", data))
                    elif input_id == "scan":
                        # 激光雷达数据处理（可选）
                        pass
                        
            except Exception as e:
                print(f"[VISUALIZATION] Dora循环错误: {e}")
                time.sleep(0.1)
    
    def update_visualization(self, frame):
        """动画更新函数"""
        # 处理队列中的更新
        try:
            while not self.update_queue.empty():
                update_type, data = self.update_queue.get_nowait()
                
                if update_type == "pose":
                    self.update_robot_pose(data)
                elif update_type == "path":
                    self.update_global_path(data)
                elif update_type == "status":
                    self.update_planning_status(data)
                    
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[VISUALIZATION] 更新可视化错误: {e}")
        
        return [self.robot_circle, self.robot_arrow, self.target_circle, self.path_line]
    
    def run(self):
        """运行可视化节点"""
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
            
            print("[VISUALIZATION] 可视化界面已启动")
            plt.show()
            
        except KeyboardInterrupt:
            print("[VISUALIZATION] 用户中断")
        except Exception as e:
            print(f"[VISUALIZATION] 运行错误: {e}")
        finally:
            self.running = False

def main():
    import sys
    
    if len(sys.argv) != 2:
        print("使用方法: python visualization_node.py <node_id>")
        sys.exit(1)
    
    node_id = sys.argv[1]
    
    try:
        viz_node = VisualizationNode(node_id)
        viz_node.run()
    except Exception as e:
        print(f"[VISUALIZATION] 主程序错误: {e}")

if __name__ == "__main__":
    main()
