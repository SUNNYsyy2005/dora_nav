#!/usr/bin/env python3
"""
800×800像素仿真环境地图生成器
与SLAM、AMCL、TEB、NAV模块保持一致的地图尺寸
"""

import numpy as np
import os

# 仿真环境配置（与robot_simulator_simple.py一致）
FIELD_WIDTH = 4.0   # 米
FIELD_HEIGHT = 4.0  # 米

# 障碍物定义 (x, y, width, height) - 来自仿真环境
OBSTACLES = [
    (1.5, 1.0, 0.5, 0.3),  # 矩形障碍物1
    (2.2, 2.8, 0.4, 0.4),  # 矩形障碍物2
    (0.8, 3.2, 0.3, 0.6),  # 矩形障碍物3
]

# 地图参数 - 与各模块保持一致
MAP_PIXELS_SIZE = 800     # 像素尺寸（与系统中800×800一致）
MAP_RESOLUTION = MAP_PIXELS_SIZE / FIELD_WIDTH  # 每像素0.2米，与系统scale=0.04不同但保持比例

# 注意：这里我们调整参数以适应800×800像素
# 实际物理环境仍是4m×4m，但分辨率会有所不同

# ROS地图常量
OCCUPIED = 0      # 障碍物（黑色）
FREE_SPACE = 255  # 自由空间（白色）
UNKNOWN = 128     # 未知区域（灰色）

def meters_to_pixels(x_m, y_m):
    """将米制坐标转换为像素坐标"""
    # 将4m×4m环境映射到800×800像素
    x_pixel = int(x_m / FIELD_WIDTH * MAP_PIXELS_SIZE)
    y_pixel = int(y_m / FIELD_HEIGHT * MAP_PIXELS_SIZE)
    return x_pixel, y_pixel

def pixels_to_meters(x_pixel, y_pixel):
    """将像素坐标转换为米制坐标"""
    x_m = x_pixel / MAP_PIXELS_SIZE * FIELD_WIDTH
    y_m = y_pixel / MAP_PIXELS_SIZE * FIELD_HEIGHT
    return x_m, y_m

def add_obstacle_to_map(map_data, x_m, y_m, width_m, height_m):
    """在像素地图中添加矩形障碍物"""
    # 转换为像素坐标
    x_pixel, y_pixel = meters_to_pixels(x_m, y_m)
    width_pixels = int(width_m / FIELD_WIDTH * MAP_PIXELS_SIZE)
    height_pixels = int(height_m / FIELD_HEIGHT * MAP_PIXELS_SIZE)
    
    # 边界检查
    x_start = max(0, x_pixel)
    y_start = max(0, y_pixel)
    x_end = min(MAP_PIXELS_SIZE, x_pixel + width_pixels)
    y_end = min(MAP_PIXELS_SIZE, y_pixel + height_pixels)
    
    # 填充障碍物区域（标记为占用）
    map_data[y_start:y_end, x_start:x_end] = OCCUPIED
    
    print(f"添加障碍物: ({x_m:.2f}, {y_m:.2f}) -> ({x_m+width_m:.2f}, {y_m+height_m:.2f}) 米")
    print(f"          像素: ({x_start}, {y_start}) -> ({x_end}, {y_end})")

def add_robot_start_position(map_data, x_m, y_m, radius_m=0.2):
    """在地图上标记机器人起始位置"""
    x_pixel, y_pixel = meters_to_pixels(x_m, y_m)
    radius_pixels = max(1, int(radius_m / FIELD_WIDTH * MAP_PIXELS_SIZE))
    
    # 绘制机器人起始点（标记为特殊值）
    # 简单的圆形标记算法
    for dy in range(-radius_pixels, radius_pixels + 1):
        for dx in range(-radius_pixels, radius_pixels + 1):
            if dx*dx + dy*dy <= radius_pixels*radius_pixels:
                nx, ny = x_pixel + dx, y_pixel + dy
                if 0 <= nx < MAP_PIXELS_SIZE and 0 <= ny < MAP_PIXELS_SIZE:
                    map_data[ny, nx] = 100  # 特定标记颜色
    
    print(f"机器人起始位置: ({x_m:.2f}, {y_m:.2f}) 米")

def add_waypoints(map_data, waypoints):
    """在地图上标记路径点"""
    for i, (x_m, y_m) in enumerate(waypoints):
        x_pixel, y_pixel = meters_to_pixels(x_m, y_m)
        
        # 标记路径点（不同颜色）
        color = 50 + i * 20  # 渐变颜色
        
        # 简单的点标记
        if 0 <= x_pixel < MAP_PIXELS_SIZE and 0 <= y_pixel < MAP_PIXELS_SIZE:
            map_data[y_pixel, x_pixel] = min(255, color)
        
        print(f"路径点 {i+1}: ({x_m:.2f}, {y_m:.2f}) 米")

def save_pgm_map(map_data, filename):
    """保存地图到PGM文件"""
    # 确保目录存在
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    # 写入PGM格式
    with open(filename, 'w') as f:
        f.write("P2\n")
        f.write(f"# Generated simulation environment map (800x800)\n")
        f.write(f"# Physical Size: {FIELD_WIDTH}m x {FIELD_HEIGHT}m\n")
        f.write(f"# Pixel Size: {MAP_PIXELS_SIZE} x {MAP_PIXELS_SIZE}\n")
        f.write(f"# Resolution: {MAP_RESOLUTION}m/pixel\n")
        f.write(f"# Obsacles: {len(OBSTACLES)}\n")
        f.write(f"{MAP_PIXELS_SIZE} {MAP_PIXELS_SIZE}\n")
        f.write("255\n")
        
        # 写入像素数据（每行最多70个值以提高可读性）
        pixels_per_line = 70
        for y in range(MAP_PIXELS_SIZE):
            pixel_row = []
            for x in range(MAP_PIXELS_SIZE):
                pixel_row.append(str(map_data[y, x]))
                
                # 每行写入固定数量的像素值
                if len(pixel_row) >= pixels_per_line or x == MAP_PIXELS_SIZE - 1:
                    f.write(" ".join(pixel_row))
                    f.write("\n")
                    pixel_row = []
    
    print(f"地图已保存到: {filename}")

def save_yaml_map(filename):
    """保存YAML地图元数据文件"""
    
    # 注意：使用0.005作为分辨率（800像素/4米 = 每像素0.005米）
    actual_resolution = FIELD_WIDTH / MAP_PIXELS_SIZE
    
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    with open(filename, 'w') as f:
        f.write("image: simulation_map_800x800.pgm\n")
        f.write(f"resolution: {actual_resolution}\n")
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    
    print(f"地图元数据已保存到: {filename}")

def save_ascii_map(map_data, filename):
    """保存ASCII艺术地图（用于调试查看）"""
    filename_dir = os.path.dirname(filename)
    if filename_dir:
        os.makedirs(filename_dir, exist_ok=True)
    
    with open(filename, 'w') as f:
        f.write(f"Simulation Environment Map ({FIELD_WIDTH}m x {FIELD_HEIGHT}m)\n")
        f.write(f"Pixel Size: {MAP_PIXELS_SIZE} x {MAP_PIXELS_SIZE}\n")
        f.write(f"Resolution: {MAP_RESOLUTION} m/pixel\n")
        f.write(f"Obstacles: {len(OBSTACLES)}\n\n")
        
        # 绘制ASCII地图（采样显示以提高可读性）
        sample_ratio = max(1, MAP_PIXELS_SIZE // 100)  # 采样比例
        
        for y in range(0, MAP_PIXELS_SIZE, sample_ratio):
            line = ""
            for x in range(0, MAP_PIXELS_SIZE, sample_ratio):
                pixel_value = map_data[MAP_PIXELS_SIZE-1-y, x]  # 翻转Y轴
                
                if pixel_value == OCCUPIED:
                    line += "#"  # 障碍物
                elif pixel_value == FREE_SPACE:
                    line += " "  # 自由空间
                elif pixel_value == 100:
                    line += "R"  # 机器人起始位置
                elif pixel_value == UNKNOWN:
                    line += "?"  # 未知区域
                elif 50 <= pixel_value < 100:
                    line += "+"  # 路径点
                else:
                    line += "."  # 其他标记
            f.write(line + "\n")
    
    print(f"ASCII地图已保存到: {filename}")

def main():
    """主函数"""
    print("生成800×800像素仿真环境地图\n")
    
    print(f"地图尺寸: {MAP_PIXELS_SIZE} x {MAP_PIXELS_SIZE} 像素")
    print(f"地图分辨率: {MAP_RESOLUTION} m/pixel")
    print(f"实际尺寸: {FIELD_WIDTH} x {FIELD_HEIGHT} 米")
    
    # 创建空地图 (初始为未知区域)
    map_data = np.full((MAP_PIXELS_SIZE, MAP_PIXELS_SIZE), UNKNOWN, dtype=np.uint8)
    
    print("\n=== 开始生成地图 ===")
    
    # 1. 设置自由空间
    map_data[:] = FREE_SPACE
    print("设置自由空间区域")
    
    # 2. 添加边界墙
    border_thickness = 16  # 增加边界厚度以适应800像素
    map_data[0:border_thickness, :] = OCCUPIED  # 上边界
    map_data[MAP_PIXELS_SIZE-border_thickness:MAP_PIXELS_SIZE, :] = OCCUPIED  # 下边界
    map_data[:, 0:border_thickness] = OCCUPIED  # 左边界
    map_data[:, MAP_PIXELS_SIZE-border_thickness:MAP_PIXELS_SIZE] = OCCUPIED  # 右边界
    print("添加场地边界墙")
    
    # 3. 添加障碍物
    for obstacle in OBSTACLES:
        x_m, y_m, width_m, height_m = obstacle


        add_obstacle_to_map(map_data, x_m, y_m, width_m, height_m)
    
    # 4. 添加机器人起始位置
    add_robot_start_position(map_data, 0.5, 0.5)
    
    # 5. 添加示例路径点（机器人会经过的路径）
    waypoints = [
        (0.5, 0.5),   # 起始点
        (1.0, 0.5),   # 右侧中间
        (2.0, 1.5),   # 右上
        (3.5, 2.0),   # 上右
        (3.5, 3.5),   # 右上角
        (2.0, 3.5),   # 左上
        (0.5, 3.0),   # 左上
        (0.5, 1.5),  # 左下
        (0.5, 0.5),   # 回到起点
    ]
    add_waypoints(map_data, waypoints)
    
    print("\n=== 地图生成完成 ===")
    
    # 统计像素类型
    occupied_pixels = np.sum(map_data == OCCUPIED)
    free_pixels = np.sum(map_data == FREE_SPACE)
    unknown_pixels = np.sum(map_data == UNKNOWN)
    total_pixels = MAP_PIXELS_SIZE * MAP_PIXELS_SIZE
    
    print(f"\n=== 地图信息 ===")
    print(f"尺寸: {FIELD_WIDTH}m x {FIELD_HEIGHT}m")
    print(f"像素尺寸: {MAP_PIXELS_SIZE} x {MAP_PIXELS_SIZE}")
    print(f"分辨率: {MAP_RESOLUTION} m/pixel")
    print(f"障碍物数量: {len(OBSTACLES)}")
    print(f"\n像素统计:")
    print(f"占用区域: {occupied_pixels} ({occupied_pixels/total_pixels*100:.1f}%)")
    print(f"自由空间: {free_pixels} ({free_pixels/total_pixels*100:.1f}%)")
    print(f"未知区域: {unknown_pixels} ({unknown_pixels/total_pixels*100:.1f}%)")
    
    # 保存各种格式的地图
    save_pgm_map(map_data, "build/simulation_map_800x800.pgm")
    save_yaml_map("build/simulation_map_800x800.yaml")
    save_ascii_map(map_data, "simulation_map_800x800.txt")
    
    print("\n=== 地图生成完成！ ===")
    print("可用文件:")
    print("- build/simulation_map_800x800.pgm (用于导航算法)")
    print("- build/simulation_map_800x800.yaml (地图元数据)")
    print("- simulation_map_800x800.txt (ASCII查看)")
    
    print("\n地图特征:")
    print("- 场地边界: 4m×4m有墙壁围护")
    print("- 障碍物1: 位置(1.5,1.0) 尺寸(0.5×0.3)m")
    print("- 障碍物2: 位置(2.2,2.8) 尺寸(0.4×0.4)m") 
    print("- 障碍物3: 位置(0.8,3.2) 尺寸(0.3×0.6)m")
    print("- 机器人起始: (0.5,0.5)m")
    
    print(f"\n兼容性说明:")
    print(f"- 与TEB模块的map_width=800保持一致")
    print(f"- 与NAV模块的800×800网格保持一致")
    print(f"- 与SLAM生成的地图尺寸匹配")

if __name__ == "__main__":

    main()
