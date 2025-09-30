#!/usr/bin/env python3
"""
简化版机器人仿真器 - 移除Dora依赖，专注于传感器数据生成
"""

import json
import math
import time
import numpy as np

# 4m x 4m 场地配置
FIELD_WIDTH = 4.0  # 米
FIELD_HEIGHT = 4.0  # 米

# 障碍物定义 (x, y, width, height) - 地图中心坐标系
OBSTACLES = [
    (-0.5, -1.0, 0.5, 0.3),  # 矩形障碍物1 (原1.5,1.0)
    (0.2, 0.8, 0.4, 0.4),    # 矩形障碍物2 (原2.2,2.8)
    (-1.2, 1.2, 0.3, 0.6),   # 矩形障碍物3 (原0.8,3.2)
]

class RobotSimulator:
    def __init__(self):
        # 机器人初始位置（地图中心坐标系，安全路径起点）
        self.x = -1.7  # 安全路径起点
        self.y = -1.7  # 安全路径起点
        self.theta = 0.0  # 朝向
    
        # 运动参数（降低速度以便AMCL更好地跟踪）
        self.linear_speed = 0.05  # m/s (降低到原来的1/10)
        self.angular_speed = 0.15  # rad/s
        self.timer_period = 0.1  # 100ms
        
        # 路径参数
        self.path_step = 0.0
        
    def update_position(self, dt):
        """根据时间步长更新机器人位置（顺时针绕圈）"""
        # 计算绕圈的路径
        # 总路径长度 = 2*width + 2*height - 四个转角
        corner_size = 0.5
        safe_width = FIELD_WIDTH - 2 * corner_size
        safe_height = FIELD_HEIGHT - 2 * corner_size
        
        # 顺时针绕圈：
        # 左下 -> 右下 -> 右上 -> 左上 -> 左下
        
        def get_position_and_yaw(path_progress):
            # 避开障碍物的安全路径
            # 障碍物1: (-0.5, -1.0, 0.5, 0.3) - 底部中央
            # 障碍物2: (0.2, 0.8, 0.4, 0.4) - 右上
            # 障碍物3: (-1.2, 1.2, 0.3, 0.6) - 左上
            
            if path_progress <= safe_width:
                # 底部路径：向右移动，避开障碍物1
                x = -1.5 + path_progress  # 从-1.5到1.5
                y = -1.7  # 更靠下，避开障碍物1 (y=-1.0到-0.7)
                theta = 0.0
            elif path_progress <= safe_width + safe_height:
                # 右侧路径：向上移动，避开障碍物2
                x = 1.7  # 更靠右，避开障碍物2 (x=0.0到0.4)
                y = -1.7 + (path_progress - safe_width)  # 从-1.7到1.7
                theta = math.pi / 2
            elif path_progress <= 2 * safe_width + safe_height:
                # 顶部路径：向左移动
                x = 1.7 - (path_progress - safe_width - safe_height)  # 从1.7到-1.7
                y = 1.7  # 更靠上
                theta = math.pi
            elif path_progress <= 2 * safe_width + 2 * safe_height:
                # 左侧路径：向下移动，避开障碍物3
                x = -1.7  # 更靠左，避开障碍物3 (x=-1.35到-1.05)
                y = 1.7 - (path_progress - 2 * safe_width - safe_height)  # 从1.7到-1.7
                theta = -math.pi / 2
            else:
                # 回到起点
                x = -1.7
                y = -1.7
                theta = 0.0
                
            return x, y, theta
        
        # 更新路径进度
        total_path_length = 2 * safe_width + 2 * safe_height
        self.path_step += self.linear_speed * dt
        path_progress = self.path_step % total_path_length
        
        self.x, self.y, self.theta = get_position_and_yaw(path_progress)
        
    def simulate_lidar_scan(self):
        """模拟激光雷达扫描"""
        # 激光雷达参数
        angle_min = -2.3561899662017822  # -135度
        angle_max = 2.3561899662017822    # +135度
        angle_increment = 0.004363314714282751  # 约0.25度
        range_min = 0.1
        range_max = 10.0
        
        # 计算射线数量
        num_rays = int((angle_max - angle_min) / angle_increment) + 1
        
        ranges = []
        intensities = []
        
        for i in range(num_rays):
            # 计算当前射线角度
            angle = angle_min + i * angle_increment
            
            # 计算射线的全局方向
            global_angle = self.theta + angle
            
            # 射线起点
            start_x = self.x
            start_y = self.y
            
            # 射线方向
            dx = math.cos(global_angle)
            dy = math.sin(global_angle)
            
            # 找到最近的距离
            min_distance = range_max
            
            # 检查与场边界的交点（地图中心坐标系：-2m到+2m）
            if dx != 0:
                # 与左右边界
                t_left = (-2.0 - start_x) / dx if (-2.0 - start_x) / dx > 0 else float('inf')
                t_right = (2.0 - start_x) / dx if (2.0 - start_x) / dx > 0 else float('inf')
                if 0 < t_left < min_distance and -2.0 <= start_y + t_left * dy <= 2.0:
                    min_distance = t_left
                if 0 < t_right < min_distance and -2.0 <= start_y + t_right * dy <= 2.0:
                    min_distance = t_right
                    
            if dy != 0:
                # 与上下边界
                t_bottom = (-2.0 - start_y) / dy if (-2.0 - start_y) / dy > 0 else float('inf')
                t_top = (2.0 - start_y) / dy if (2.0 - start_y) / dy > 0 else float('inf')
                if 0 < t_bottom < min_distance and -2.0 <= start_x + t_bottom * dx <= 2.0:
                    min_distance = t_bottom
                if 0 < t_top < min_distance and -2.0 <= start_x + t_top * dx <= 2.0:
                    min_distance = t_top
            
            # 检查与障碍物的交点
            for obs_x, obs_y, obs_w, obs_h in OBSTACLES:
                intersection_dist = self.ray_rectangle_intersection(
                    start_x, start_y, dx, dy, obs_x, obs_y, obs_w, obs_h
                )
                if intersection_dist > 0 and intersection_dist < min_distance:
                    min_distance = intersection_dist
            
            # 限制距离范围
            distance = max(range_min, min(min_distance, range_max))
            
            # 添加一些噪声
            noise = np.random.normal(0, 0.02)
            distance += noise
            distance = max(0, distance)  # 确保非负
            
            ranges.append(distance)
            intensities.append(max(0, 1.0 / (distance + 0.1)))  # 强度随距离减少
        
        # 生成JSON格式的激光雷达数据
        scan_data = {
            "header": {
                "seq": 0,
                "stamp": {
                    "sec": int(time.time()),
                    "nsec": int((time.time() % 1) * 1e9)
                },
                "frame_id": "laser_frame"
            },
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "time_increment": 0.0,
            "scan_time": 0.0,
            "range_min": range_min,
            "range_max": range_max,
            "ranges": ranges,
            "intensities": intensities
        }
        
        return json.dumps(scan_data, separators=(',', ':'))
    
    def ray_rectangle_intersection(self, start_x, start_y, dx, dy, rect_x, rect_y, rect_w, rect_h):
        """计算射线与矩形的交点距离"""
        if dx == 0 and dy == 0:
            return -1
            
        intersections = []
        
        # 检查射线与四条边的交点
        # 左边 x = rect_x
        if dx != 0:
            t = (rect_x - start_x) / dx
            if t > 0:
                hit_y = start_y + t * dy
                if rect_y <= hit_y <= rect_y + rect_h:
                    intersections.append(t)
        
        # 右边 x = rect_x + rect_w
        if dx != 0:
            t = (rect_x + rect_w - start_x) / dx
            if t > 0:
                hit_y = start_y + t * dy
                if rect_y <= hit_y <= rect_y + rect_h:
                    intersections.append(t)
        
        # 下边 y = rect_y
        if dy != 0:
            t = (rect_y - start_y) / dy
            if t > 0:
                hit_x = start_x + t * dx
                if rect_x <= hit_x <= rect_x + rect_w:
                    intersections.append(t)
        
        # 上边 y = rect_y + rect_h
        if dy != 0:
            t = (rect_y + rect_h - start_y) / dy
            if t > 0:
                hit_x = start_x + t * dx
                if rect_x <= hit_x <= rect_x + rect_w:
                    intersections.append(t)
        
        return min(intersections) if intersections else -1
    
    def simulate_imu_data(self):
        """模拟IMU数据"""
        # 基于机器人当前位置和朝向生成IMU数据
        
        # 线性加速度（简化：主要是重力 + 一些运动噪声）
        linear_acceleration = {
            "x": np.random.normal(0, 0.1),  # 前后加速度
            "y": np.random.normal(0, 0.1),  # 左右加速度
            "z": np.random.normal(9.8, 0.05)  # 重力加速度
        }
        
        # 角速度（基于转向）
        angular_velocity = {
            "x": np.random.normal(0, 0.01),  # roll
            "y": np.random.normal(0, 0.01),  # pitch
            "z": np.random.normal(0, 0.02)   # yaw
        }
        
        # 方向（从角度转换为四元数）
        half_yaw = self.theta * 0.5
        w = math.cos(half_yaw)
        z = math.sin(half_yaw)
        x = 0.0
        y = 0.0
        
        orientation = {
            "x": x,
            "y": y,
            "z": z,
            "w": w
        }
        
        # 协方差矩阵（简化为对角阵）
        orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        linear_acceleration_covariance = [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05]
        
        # 生成JSON格式的IMU数据
        imu_data = {
            "header": {
                "seq": 0,
                "stamp": {
                    "sec": int(time.time()),
                    "nsec": int((time.time() % 1) * 1e9)
                },
                "frame_id": "/imu_link"
            },
            "orientation": orientation,
            "orientation_covariance": orientation_covariance,
            "angular_velocity": angular_velocity,
            "angular_velocity_covariance": angular_velocity_covariance,
            "linear_acceleration": linear_acceleration,
            "linear_acceleration_covariance": linear_acceleration_covariance
        }
        
        return json.dumps(imu_data, separators=(',', ':'))
    
    def update_simulation(self, dt):
        """更新仿真状态"""
        self.update_position(dt)
