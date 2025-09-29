import os
import json
import math
import time
import numpy as np
import dora
from dora.builder import DataflowBuilder

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Initialize a new dataflow
dataflow = DataflowBuilder(name="robot-simulation-dataflow")

# 4m x 4m 场地配置
FIELD_WIDTH = 4.0  # 米
FIELD_HEIGHT = 4.0  # 米

# 障碍物定义 (x, y, width, height)
OBSTACLES = [
    (1.5, 1.0, 0.5, 0.3),  # 矩形障碍物1
    (2.2, 2.8, 0.4, 0.4),  # 矩形障碍物2
    (0.8, 3.2, 0.3, 0.6),  # 矩形障碍物3
]

class RobotSimulator:
    def __init__(self):
        # 机器人初始位置（左下角）
        self.x = 0.5
        self.y = 0.5
        self.theta = 0.0  # 朝向
    
        # 运动参数
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.3  # rad/s
        self.timer_period = 0.1  # 100ms
        
        # 计时器
        self.last_scan_time = 0
        self.last_imu_time = 0
        self.scan_period = 0.1  # 10Hz
        self.imu_period = 0.05   # 20Hz
        
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
            if path_progress <= safe_width:
                # 底部路径：向左移动
                x = corner_size + path_progress
                y = corner_size
                theta = 0.0
            elif path_progress <= safe_width + safe_height:
                # 右侧路径：向上移动
                x = FIELD_WIDTH - corner_size
                y = corner_size + (path_progress - safe_width)
                theta = math.pi / 2
            elif path_progress <= 2 * safe_width + safe_height:
                # 顶部路径：向左移动
                x = FIELD_WIDTH - corner_size - (path_progress - safe_width - safe_height)
                y = FIELD_HEIGHT - corner_size
                theta = math.pi
            elif path_progress <= 2 * safe_width + 2 * safe_height:
                # 左侧路径：向下移动
                x = corner_size
                y = FIELD_HEIGHT - corner_size - (path_progress - 2 * safe_width - safe_height)
                theta = -math.pi / 2
            else:
                # 回到起点
                x = corner_size
                y = corner_size
                theta = 0.0
                
            return x, y, theta
        
        # 更新路径进度
        total_path_length = 2 * safe_width + 2 * safe_height
        self.path_step += self.linear_speed * dt
        path_progress = self.path_step % total_path_length
        
        self.x, self.y, self.theta = get_position_and_yaw(path_progress)
        
    def simulate_lidar_scan(self):
        """模拟激光雷达扫描"""
        current_time = time.time()
        if current_time - self.last_scan_time < self.scan_period:
            return None
            
        self.last_scan_time = current_time
        
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
            
            # 检查与场边界的交点
            if dx != 0:
                # 与左右边界
                t_left = (0 - start_x) / dx if (0 - start_x) / dx > 0 else float('inf')
                t_right = (FIELD_WIDTH - start_x) / dx if (FIELD_WIDTH - start_x) / dx > 0 else float('inf')
                if 0 < t_left < min_distance and 0 <= start_y + t_left * dy <= FIELD_HEIGHT:
                    min_distance = t_left
                if 0 < t_right < min_distance and 0 <= start_y + t_right * dy <= FIELD_HEIGHT:
                    min_distance = t_right
                    
            if dy != 0:
                # 与上下边界
                t_bottom = (0 - start_y) / dy if (0 - start_y) / dy > 0 else float('inf')
                t_top = (FIELD_HEIGHT - start_y) / dy if (FIELD_HEIGHT - start_y) / dy > 0 else float('inf')
                if 0 < t_bottom < min_distance and 0 <= start_x + t_bottom * dx <= FIELD_WIDTH:
                    min_distance = t_bottom
                if 0 < t_top < min_distance and 0 <= start_x + t_top * dx <= FIELD_WIDTH:
                    min_distance = t_top
            
            # 检查与障碍物的交点
            for obs_x, obs_y, obs_w, obs_h in OBSTACLES:
                # 检查射线与障碍物矩形的交点
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
            
            ranges.append(distance)
            intensities.append(max(0, 1.0 / (distance + 0.1)))  # 强度随距离减少
        
        # 生成JSON格式的激光雷达数据
        scan_data = {
            "header": {
                "seq": 0,
                "stamp": {
                    "sec": int(current_time),
                    "nsec": int((current_time % 1) * 1e9)
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
        current_time = time.time()
        if current_time - self.last_imu_time < self.imu_period:
            return None
            
        self.last_imu_time = current_time
        
        # 基于机器人当前位置和朝向生成IMU数据
        # 这里简化处理，添加一些噪声和动态效果
        
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
                    "sec": int(current_time),
                    "nsec": int((current_time % 1) * 1e9)
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

# Create simulator instance
simulator = RobotSimulator()

def lidar_sim_program():
    """雷达仿真程序入口点"""
    current_time = time.time()
    dt = 0.1  # 仿真时间步长
    
    # 更新仿真状态
    simulator.update_simulation(dt)
    
    # 获取雷达数据
    lidar_data = simulator.simulate_lidar_scan()
    
    if lidar_data:
        # 这里应该发送数据到Dora数据流
        # 注意：这需要在实际的Python Dora节点中实现
        print(f"Lidar data: {lidar_data[:100]}...")

def imu_sim_program():
    """IMU仿真程序入口点"""
    current_time = time.time()
    dt = 0.05  # IMU更新频率更高
    
    # 获取IMU数据
    imu_data = simulator.simulate_imu_data()
    
    if imu_data:
        # 这里应该发送数据到Dora数据流
        # 注意：这需要在实际的Python Dora节点中实现
        print(f"IMU data: {imu_data[:100]}...")

# ============== Dora 节点定义 ==============

# Lidar simulator node
lidar_node = dataflow.add_node(
    id="lidar_sim",
    path="python",
    build="python -c \"import json, math, time, numpy as np\"",
    env={
        "SIM_TYPE": "lidar",
        "FIELD_SIZE": f"{FIELD_WIDTH}x{FIELD_HEIGHT}",
        "ROBOT_POSITION": "bottom_left",
        "UPDATE_FREQ": "10"
    },
)
lidar_scan_output = lidar_node.add_output("scan")

# IMU simulator node  
imu_node = dataflow.add_node(
    id="imu_sim", 
    path="python",
    build="python -c \"import json, math, time, numpy as np\"",
    env={
        "SIM_TYPE": "imu",
        "UPDATE_FREQ": "20"
    },
)
imu_data_output = imu_node.add_output("data")

# AMCL localization node (modified inputs for simulation)
amcl_node = dataflow.add_node(
    id="amcl_sim",
    source="build/amcl/build/test",
    inputs={
        "tick": "dora/timer/millis/10",
        "scan2": "lidar_sim/scan",
        "imu": "imu_sim/data",
        "twist": "teb_sim/twist"
    },
    outputs=["pose"]
)

# TEB planner node
teb_node = dataflow.add_node(
    id="teb_sim",
    source="build/teb/build/test", 
    inputs={
        "tick": "dora/timer/millis/500",
        "pose": "amcl_sim/pose",
        "scan": "lidar_sim/scan"
    },
    outputs=["twist"]
)

# Control node
control_node = dataflow.add_node(
    id="control_sim",
    path="control/target/release/control",  # 使用release版本
    inputs={
        "message": "teb_sim/twist"
    },
)

# Generate the YAML file
output_file = os.path.join(script_dir, "sim_robot_dataflow.yml")
dataflow.to_yaml(output_file)
print(f"Generated simulation dataflow: {output_file}")

# If env var NO_BUILD is set, skip the build and run steps
if not os.getenv("NO_BUILD"):
    print("Building simulation dataflow...")
    build(output_file, uv=True)
else:
    print("Skipping build due to NO_BUILD env var")

print("Running robot simulation dataflow...")
run(output_file, uv=True)

# 打印仿真信息
print(f"""
===============================================
        机器人仿真配置信息
===============================================
场地尺寸: {FIELD_WIDTH}m x {FIELD_HEIGHT}m
障碍物数量: {len(OBSTACLES)}
起始位置: 左下角 ({simulator.x:.1f}, {simulator.y:.1f})
运动模式: 顺时针绕圈
雷达频率: 10Hz
IMU频率: 20Hz
===============================================
""")

for i, (obs_x, obs_y, obs_w, obs_h) in enumerate(OBSTACLES, 1):
    print(f"障碍物{i}: 位置({obs_x:.1f}, {obs_y:.1f}), 尺寸({obs_w:.1f}x{obs_h:.1f})")

print("===============================================")

if __name__ == "__main__":
    # 测试仿真数据生成
    print("Testing simulation data generation...")
    
    # 测试雷达数据
    laser_data = simulator.simulate_lidar_scan()
    if laser_data:
        data = json.loads(laser_data)
        print(f"✓ Lidar simulation: {len(data['ranges'])} rays, range: {min(data['ranges']):.2f}-{max(data['ranges']):.2f}m")
    
    # 测试IMU数据
    imu_data = simulator.simulate_imu_data()
    if imu_data:
        data = json.loads(imu_data)
        print(f"✓ IMU simulation: orientation=({data['orientation']['w']:.3f}, {data['orientation']['x']:.3f}, {data['orientation']['y']:.3f}, {data['orientation']['z']:.3f})")
    
    print("Simulation test completed!")
