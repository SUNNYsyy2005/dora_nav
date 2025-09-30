#!/usr/bin/env python3
"""
基于OpenCV的地图可视化节点
解决matplotlib中文字体和性能问题
"""

import cv2
import numpy as np
import json
import time
import yaml
import os
import math
from pathlib import Path
from dora import Node

# 项目路径管理
from project_paths import ProjectPaths

class MapVisualizationOpenCV:
    def __init__(self, node_id="map_viz"):
        self.node = Node(node_id)
        self.running = True
        
        # 地图相关
        self.map_data = None
        self.map_config = {}
        self.map_loaded = False
        
        # 机器人状态
        self.robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.target_pose = {"x": None, "y": None, "theta": None}
        self.has_target = False
        
        # 可视化数据
        self.global_path = []
        self.laser_points = []
        self.planning_status = "IDLE"
        
        # 用户交互模式
        self.set_initial_pose_mode = False
        self.set_goal_mode = False
        
        # OpenCV窗口设置
        self.window_name = "Robot Navigation - Map Visualization"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1200, 900)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # 初始化地图
        self.load_map_config()
        
        print(f"[MAP_VIZ_CV] OpenCV可视化节点已启动")
    
    def find_map_config_files(self):
        """搜索可用的地图配置文件"""
        search_paths = ProjectPaths.get_standard_map_search_paths()
        
        found_files = []
        for path_str in search_paths:
            path = Path(path_str)
            if path.exists():
                found_files.append(path)
        
        # 按修改时间排序，最新的在前
        found_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
        
        print(f"[MAP_VIZ_CV] Found {len(found_files)} map config files")
        return found_files
    
    def load_map_config(self):
        """加载地图配置"""
        try:
            config_files = self.find_map_config_files()
            
            for config_file in config_files:
                print(f"[MAP_VIZ_CV] Trying to load: {config_file}")
                
                try:
                    with open(config_file, 'r') as f:
                        self.map_config = yaml.safe_load(f)
                    
                    # 验证必要字段
                    required_fields = ['image', 'resolution']
                    if all(field in self.map_config for field in required_fields):
                        # 使用项目路径管理器查找地图图像
                        map_image_path = ProjectPaths.find_map_image_for_yaml(str(config_file))
                        
                        if map_image_path and Path(map_image_path).exists():
                            self.map_data = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
                            if self.map_data is not None:
                                self.map_loaded = True
                                self.map_config['config_file'] = str(config_file)
                                self.map_config['image_file'] = map_image_path
                                
                                # 确保有宽度和高度信息
                                if 'width' not in self.map_config:
                                    self.map_config['width'] = self.map_data.shape[1]
                                if 'height' not in self.map_config:
                                    self.map_config['height'] = self.map_data.shape[0]
                                
                                print(f"[MAP_VIZ_CV] Map loaded successfully!")
                                print(f"[MAP_VIZ_CV]   Config: {config_file}")
                                print(f"[MAP_VIZ_CV]   Image: {map_image_path}")
                                print(f"[MAP_VIZ_CV]   Resolution: {self.map_config['resolution']} m/pixel")
                                print(f"[MAP_VIZ_CV]   Size: {self.map_config['width']}x{self.map_config['height']} pixels")
                                
                                # 发布地图配置信息
                                self.publish_map_info()
                                return
                        else:
                            print(f"[MAP_VIZ_CV] Map image not found for config: {config_file}")
                    
                except Exception as e:
                    print(f"[MAP_VIZ_CV] Failed to load config {config_file}: {e}")
                    continue
            
            # 如果没有找到有效地图，创建默认地图
            print("[MAP_VIZ_CV] No valid map found, creating default map")
            self.create_default_map()
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error loading map config: {e}")
            self.create_default_map()
    
    def create_default_map(self):
        """创建默认地图"""
        self.map_config = {
            'resolution': 0.005,
            'width': 800,
            'height': 800,
            'origin': [0.0, 0.0, 0.0],
            'config_file': 'default',
            'image_file': 'default_map.pgm'
        }
        
        # 创建800x800的默认地图
        self.map_data = np.ones((800, 800), dtype=np.uint8) * 200  # 灰色背景
        
        # 添加边界
        self.map_data[0:5, :] = 0
        self.map_data[-5:, :] = 0
        self.map_data[:, 0:5] = 0
        self.map_data[:, -5:] = 0
        
        self.map_loaded = True
        print("[MAP_VIZ_CV] Default map created")
    
    def world_to_pixel(self, x, y):
        """世界坐标转换为像素坐标"""
        resolution = self.map_config.get('resolution', 0.005)
        width = self.map_config.get('width', 800)
        height = self.map_config.get('height', 800)
        origin = self.map_config.get('origin', [0.0, 0.0, 0.0])
        
        # 考虑地图原点
        pixel_x = int((x - origin[0]) / resolution)
        pixel_y = int((y - origin[1]) / resolution)
        
        # 如果原点在地图中心
        if origin[0] == 0.0 and origin[1] == 0.0:
            pixel_x += width // 2
            pixel_y = height // 2 - pixel_y  # Y轴翻转
        
        return pixel_x, pixel_y
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """像素坐标转换为世界坐标"""
        resolution = self.map_config.get('resolution', 0.005)
        width = self.map_config.get('width', 800)
        height = self.map_config.get('height', 800)
        origin = self.map_config.get('origin', [0.0, 0.0, 0.0])
        
        # 如果原点在地图中心
        if origin[0] == 0.0 and origin[1] == 0.0:
            pixel_x -= width // 2
            pixel_y = height // 2 - pixel_y  # Y轴翻转
        
        x = pixel_x * resolution + origin[0]
        y = pixel_y * resolution + origin[1]
        
        return x, y
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            world_x, world_y = self.pixel_to_world(x, y)
            
            if self.set_initial_pose_mode:
                self.robot_pose['x'] = world_x
                self.robot_pose['y'] = world_y
                self.robot_pose['theta'] = 0.0
                print(f"[MAP_VIZ_CV] Set initial pose: x={world_x:.3f}, y={world_y:.3f}")
                self.publish_initial_pose(world_x, world_y, 0.0)
                self.set_initial_pose_mode = False
            elif self.set_goal_mode:
                self.target_pose['x'] = world_x
                self.target_pose['y'] = world_y
                self.target_pose['theta'] = 0.0
                self.has_target = True
                print(f"[MAP_VIZ_CV] Set goal: x={world_x:.3f}, y={world_y:.3f}")
                self.publish_goal(world_x, world_y, 0.0)
                self.set_goal_mode = False
    
    def publish_initial_pose(self, x, y, theta):
        """发布初始位姿"""
        try:
            initial_pose_msg = {
                "header": {
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "initial_pose": {
                    "x": x,
                    "y": y,
                    "theta": theta
                }
            }
            
            self.node.send_output("initial_pose", json.dumps(initial_pose_msg).encode())
            print(f"[MAP_VIZ_CV] Published initial pose: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error sending initial pose: {e}")
    
    def publish_goal(self, x, y, theta):
        """发布目标点"""
        try:
            goal_msg = {
                "header": {
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "target_pose": {
                    "x": x,
                    "y": y,
                    "theta": theta
                },
                "goal_id": f"goal_{int(time.time())}"
            }
            
            self.node.send_output("goal", json.dumps(goal_msg).encode())
            print(f"[MAP_VIZ_CV] Published goal: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error sending goal: {e}")
    
    def publish_map_info(self):
        """发布地图信息"""
        try:
            map_info_msg = {
                "header": {
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "resolution": self.map_config['resolution'],
                "width": self.map_config['width'],
                "height": self.map_config['height'],
                "origin": self.map_config['origin'],
                "config_source": self.map_config.get('config_file', 'default')
            }
            
            self.node.send_output("map_info", json.dumps(map_info_msg).encode())
            print(f"[MAP_VIZ_CV] Published map info")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error sending map info: {e}")
    
    def draw_robot(self, img, pose, color=(255, 0, 0), size=10):
        """绘制机器人"""
        if pose['x'] is None or pose['y'] is None:
            return
        
        px, py = self.world_to_pixel(pose['x'], pose['y'])
        
        # 绘制圆形表示机器人
        cv2.circle(img, (px, py), size, color, -1)
        
        # 绘制方向箭头
        if pose['theta'] is not None:
            arrow_length = size + 8
            end_x = int(px + arrow_length * math.cos(pose['theta']))
            end_y = int(py - arrow_length * math.sin(pose['theta']))  # Y轴在图片中是反的
            cv2.arrowedLine(img, (px, py), (end_x, end_y), (0, 255, 255), 2)
    
    def draw_path(self, img, path, color=(0, 255, 0), thickness=2):
        """绘制路径"""
        if len(path) < 2:
            return
        
        points = []
        for pose in path:
            if 'position' in pose:
                px, py = self.world_to_pixel(pose['position']['x'], pose['position']['y'])
            else:
                px, py = self.world_to_pixel(pose['x'], pose['y'])
            points.append((px, py))
        
        # 绘制路径线
        for i in range(len(points) - 1):
            cv2.line(img, points[i], points[i + 1], color, thickness)
    
    def draw_laser_points(self, img, laser_points, color=(0, 255, 255)):
        """绘制激光点云"""
        for px, py in laser_points:
            if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
                cv2.circle(img, (int(px), int(py)), 1, color, -1)
    
    def create_display_image(self):
        """创建显示图像"""
        if self.map_data is None:
            return np.zeros((600, 800, 3), dtype=np.uint8)
        
        # 转换为彩色图像
        display_img = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2BGR)
        
        # 绘制激光点云
        self.draw_laser_points(display_img, self.laser_points, (0, 255, 255))
        
        # 绘制全局路径
        self.draw_path(display_img, self.global_path, (0, 255, 0), 2)
        
        # 绘制机器人
        self.draw_robot(display_img, self.robot_pose, (255, 0, 0), 8)
        
        # 绘制目标点
        if self.has_target and self.target_pose['x'] is not None:
            self.draw_robot(display_img, self.target_pose, (0, 0, 255), 8)
            # 绘制目标标记
            px, py = self.world_to_pixel(self.target_pose['x'], self.target_pose['y'])
            cv2.drawMarker(display_img, (px, py), (0, 0, 255), cv2.MARKER_STAR, 15, 2)
        
        # 添加信息文本
        self.add_info_text(display_img)
        
        return display_img
    
    def add_info_text(self, img):
        """添加信息文本"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)
        thickness = 2
        
        # 地图信息
        y_offset = 30
        cv2.putText(img, f"Map Info:", (10, y_offset), font, font_scale, color, thickness)
        y_offset += 25
        cv2.putText(img, f"Resolution: {self.map_config.get('resolution', 0.005)} m/pixel", 
                   (10, y_offset), font, font_scale-0.1, color, 1)
        y_offset += 20
        cv2.putText(img, f"Size: {self.map_config.get('width', 800)}x{self.map_config.get('height', 800)} pixels", 
                   (10, y_offset), font, font_scale-0.1, color, 1)
        
        # 机器人状态
        y_offset += 40
        cv2.putText(img, f"Robot: ({self.robot_pose['x']:.2f}, {self.robot_pose['y']:.2f}, {self.robot_pose['theta']:.2f})", 
                   (10, y_offset), font, font_scale-0.1, color, 1)
        
        # 规划状态
        y_offset += 25
        cv2.putText(img, f"Status: {self.planning_status}", (10, y_offset), font, font_scale-0.1, color, 1)
        
        # 操作说明
        height = img.shape[0]
        y_offset = height - 150
        cv2.putText(img, "Controls:", (10, y_offset), font, font_scale, color, thickness)
        y_offset += 25
        cv2.putText(img, "Press 'i' -> Set Initial Pose", (10, y_offset), font, font_scale-0.1, color, 1)
        y_offset += 20
        cv2.putText(img, "Press 'g' -> Set Goal", (10, y_offset), font, font_scale-0.1, color, 1)
        y_offset += 20
        cv2.putText(img, "Left Click -> Confirm Position", (10, y_offset), font, font_scale-0.1, color, 1)
        y_offset += 20
        cv2.putText(img, "Press 'r' -> Republish Map Info", (10, y_offset), font, font_scale-0.1, color, 1)
        y_offset += 20
        cv2.putText(img, "Press 'q' -> Quit", (10, y_offset), font, font_scale-0.1, color, 1)
        
        # 当前模式
        if self.set_initial_pose_mode:
            cv2.putText(img, "MODE: SET INITIAL POSE", (10, height - 30), font, font_scale, (0, 255, 255), thickness)
        elif self.set_goal_mode:
            cv2.putText(img, "MODE: SET GOAL", (10, height - 30), font, font_scale, (0, 255, 255), thickness)
    
    def update_robot_pose(self, pose_data):
        """更新机器人位姿"""
        try:
            if isinstance(pose_data, str):
                pose_data = json.loads(pose_data)
            
            self.robot_pose['x'] = pose_data.get('x', 0.0)
            self.robot_pose['y'] = pose_data.get('y', 0.0)
            self.robot_pose['theta'] = pose_data.get('theta', 0.0)
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error updating robot pose: {e}")
    
    def update_global_path(self, path_data):
        """更新全局路径"""
        try:
            if isinstance(path_data, str):
                path_data = json.loads(path_data)
            
            self.global_path = path_data.get('poses', [])
            print(f"[MAP_VIZ_CV] Updated global path: {len(self.global_path)} points")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error updating global path: {e}")
    
    def update_laser_scan(self, scan_data):
        """更新激光扫描"""
        try:
            if isinstance(scan_data, str):
                scan_data = json.loads(scan_data)
            
            ranges = scan_data.get("ranges", [])
            angle_min = scan_data.get("angle_min", -math.pi)
            angle_increment = scan_data.get("angle_increment", 0.01)
            
            self.laser_points = []
            angle = angle_min
            
            for range_val in ranges:
                if not math.isnan(range_val) and 0.1 < range_val < 10.0:
                    # 激光点在机器人坐标系中的位置
                    local_x = range_val * math.cos(angle + self.robot_pose["theta"])
                    local_y = range_val * math.sin(angle + self.robot_pose["theta"])
                    
                    # 转换到世界坐标
                    world_x = self.robot_pose["x"] + local_x
                    world_y = self.robot_pose["y"] + local_y
                    
                    # 转换到像素坐标
                    px, py = self.world_to_pixel(world_x, world_y)
                    self.laser_points.append((px, py))
                
                angle += angle_increment
            
            print(f"[MAP_VIZ_CV] Updated laser scan: {len(self.laser_points)} points")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error updating laser scan: {e}")
    
    def update_planning_status(self, status_data):
        """更新规划状态"""
        try:
            if isinstance(status_data, str):
                status_data = json.loads(status_data)
            
            status_code = status_data.get("status", 0)
            status_map = {0: "IDLE", 1: "PLANNING", 2: "SUCCESS", 3: "FAILED"}
            self.planning_status = status_map.get(status_code, "UNKNOWN")
            
        except Exception as e:
            print(f"[MAP_VIZ_CV] Error updating planning status: {e}")
    
    def run(self):
        """主运行循环"""
        print("[MAP_VIZ_CV] Starting main loop...")
        
        # 首次启动时发布地图信息
        self.publish_map_info()
        
        for event in self.node:
            if event.event_type == "STOP":
                print("[MAP_VIZ_CV] Received STOP event.")
                break
            
            if event.event_type == "INPUT":
                input_id = event.id.decode()
                data = event.data.decode()
                
                try:
                    if input_id == "robot_pose":
                        self.update_robot_pose(data)
                    elif input_id == "global_path":
                        self.update_global_path(data)
                    elif input_id == "laser_scan":
                        self.update_laser_scan(data)
                    elif input_id == "planning_status":
                        self.update_planning_status(data)
                    elif input_id == "timer" or input_id == "tick":
                        # 定时更新显示
                        pass
                except Exception as e:
                    print(f"[MAP_VIZ_CV] Error processing input {input_id}: {e}")
            
            # 更新显示
            display_img = self.create_display_image()
            cv2.imshow(self.window_name, display_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[MAP_VIZ_CV] 'q' pressed, exiting.")
                break
            elif key == ord('i'):
                self.set_initial_pose_mode = True
                self.set_goal_mode = False
                print("[MAP_VIZ_CV] Set initial pose mode. Click on map.")
            elif key == ord('g'):
                self.set_goal_mode = True
                self.set_initial_pose_mode = False
                print("[MAP_VIZ_CV] Set goal mode. Click on map.")
            elif key == ord('r'):
                self.publish_map_info()
                print("[MAP_VIZ_CV] Map info republished.")
            elif key == 27:  # Esc key
                self.set_initial_pose_mode = False
                self.set_goal_mode = False
                print("[MAP_VIZ_CV] Mode cancelled.")
        
        cv2.destroyAllWindows()
        print("[MAP_VIZ_CV] Node stopped.")

def main():
    print("[MAP_VIZ_CV] Starting OpenCV Map Visualization Node...")
    node = MapVisualizationOpenCV()
    try:
        node.run()
    except KeyboardInterrupt:
        print("[MAP_VIZ_CV] Interrupted by user")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

