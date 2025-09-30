#!/usr/bin/env python3
"""
机器人仿真节点 - 提供真实机器人位姿
用于可视化节点显示真实位姿与AMCL估计位姿的对比
"""

import numpy as np
import json
import time
import math
from dora import Node

class RobotSimulator:
    def __init__(self, node_id):
        self.node = Node(node_id)
        
        # 机器人状态
        self.robot_x = -1.7  # 安全路径起点
        self.robot_y = -1.7
        self.robot_theta = 0.0
        
        # 运动参数
        self.linear_velocity = 0.1  # m/s
        self.angular_velocity = 0.0  # rad/s
        self.dt = 0.1  # 时间步长
        
        # 仿真环境参数
        self.field_width = 4.0  # 4m x 4m 环境
        self.field_height = 4.0
        
        print(f"[ROBOT_SIM] 机器人仿真器初始化完成")
        print(f"[ROBOT_SIM] 初始位置: ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_theta:.3f})")
    
    def update_robot_pose(self):
        """更新机器人位姿（矩形轨迹运动）"""
        # 矩形轨迹运动（顺时针绕场地边界）
        t = time.time()
        
        # 场地参数（地图中心为原点，范围 -2m 到 +2m）
        corner_size = 0.5
        safe_width = 3.0  # 4m - 2*0.5m
        safe_height = 3.0  # 4m - 2*0.5m
        
        # 计算路径进度（降低速度：每60秒一圈，速度约0.2m/s）
        path_progress = (t * 0.05) % (2 * safe_width + 2 * safe_height)
        
        # 根据路径进度计算位置和朝向（避开障碍物）
        if path_progress <= safe_width:
            # 底部路径：向右移动，避开障碍物1
            self.robot_x = -1.7 + path_progress
            self.robot_y = -1.7  # 更靠下，避开障碍物1
            self.robot_theta = 0.0
        elif path_progress <= safe_width + safe_height:
            # 右侧路径：向上移动，避开障碍物2
            self.robot_x = 1.7  # 更靠右，避开障碍物2
            self.robot_y = -1.7 + (path_progress - safe_width)
            self.robot_theta = math.pi / 2
        elif path_progress <= 2 * safe_width + safe_height:
            # 顶部路径：向左移动
            self.robot_x = 1.7 - (path_progress - safe_width - safe_height)
            self.robot_y = 1.7  # 更靠上
            self.robot_theta = math.pi
        else:
            # 左侧路径：向下移动，避开障碍物3
            self.robot_x = -1.7  # 更靠左，避开障碍物3
            self.robot_y = 1.7 - (path_progress - 2 * safe_width - safe_height)
            self.robot_theta = -math.pi / 2
        
        # 确保在边界内
        self.robot_x = max(-1.9, min(1.9, self.robot_x))
        self.robot_y = max(-1.9, min(1.9, self.robot_y))
    
    def publish_robot_pose(self):
        """发布机器人位姿"""
        try:
            pose_msg = {
                "header": {
                    "seq": 0,
                    "stamp": {"sec": int(time.time()), "nsec": 0},
                    "frame_id": "map"
                },
                "x": self.robot_x,
                "y": self.robot_y,
                "theta": self.robot_theta
            }
            
            pose_json = json.dumps(pose_msg)
            self.node.send_output("true_robot_pose", pose_json.encode())
            
            print(f"[ROBOT_SIM] 发布真实位姿: x={self.robot_x:.3f}, y={self.robot_y:.3f}, theta={self.robot_theta:.3f}")
            
        except Exception as e:
            print(f"[ROBOT_SIM] 发布位姿错误: {e}")
    
    def run(self):
        """运行仿真器"""
        print("[ROBOT_SIM] 开始运行机器人仿真器")
        
        while True:
            event = self.node.next()
            
            if event["type"] == "INPUT":
                input_id = event["id"]
                
                if input_id == "tick" or input_id == "timer":
                    # 更新机器人位姿
                    self.update_robot_pose()
                    
                    # 发布位姿
                    self.publish_robot_pose()
                    
                elif input_id == "twist":
                    # 接收速度命令（可选）
                    try:
                        data = event["value"]
                        if isinstance(data, bytes):
                            data = data.decode('utf-8')
                        
                        twist_data = json.loads(data)
                        linear_vel = twist_data.get("linear", {}).get("x", 0.0)
                        angular_vel = twist_data.get("angular", {}).get("z", 0.0)
                        
                        # 更新运动参数
                        self.linear_velocity = linear_vel
                        self.angular_velocity = angular_vel
                        
                        print(f"[ROBOT_SIM] 收到速度命令: linear={linear_vel:.3f}, angular={angular_vel:.3f}")
                        
                    except Exception as e:
                        print(f"[ROBOT_SIM] 处理速度命令错误: {e}")
            
            elif event["type"] == "STOP":
                print("[ROBOT_SIM] 收到停止信号")
                break

if __name__ == "__main__":
    simulator = RobotSimulator("robot_sim")
    simulator.run()
