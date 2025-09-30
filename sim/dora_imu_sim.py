#!/usr/bin/env python3
"""
Dora IMU仿真节点 - 使用正确的Dora Node API
"""

import os
import sys
import json
import math
import time
import numpy as np
import logging

# Dora imports
from dora import Node
import pyarrow as pa

# 添加当前目录到路径以便导入仿真器
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot_simulator_simple import RobotSimulator

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [IMU_SIM] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def main():
    """Dora IMU仿真节点主函数"""
    logger.info("Starting Dora IMU Simulation Node...")
    
    # Get node ID from environment variable
    node_id = os.getenv("DORA_NODE_ID", "imu_sim")
    node = Node(node_id)
    
    # 存储真实机器人位姿（从robot_sim接收）
    true_robot_pose = {"x": -1.7, "y": -1.7, "theta": 0.0}
    last_theta = 0.0
    last_time = time.time()
    
    tick_count = 0
    imu_count = 0
    last_log_time = time.time()
    
    try:
        while True:
            event = node.next()
            
            if event is None:
                continue
                
            if event["type"] == "STOP":
                logger.info("🛑 Received STOP signal")
                break
            
            # 接收真实机器人位姿
            if event["type"] == "INPUT" and event["id"] == "true_robot_pose":
                try:
                    # 解码PyArrow数组
                    data = event["value"]
                    if hasattr(data, 'to_pylist'):
                        data_list = data.to_pylist()
                        json_str = ''.join(chr(x) for x in data_list)
                    else:
                        json_str = str(data)
                    
                    pose_data = json.loads(json_str)
                    true_robot_pose["x"] = pose_data.get("x", 0.0)
                    true_robot_pose["y"] = pose_data.get("y", 0.0)
                    true_robot_pose["theta"] = pose_data.get("theta", 0.0)
                    
                except Exception as e:
                    logger.error(f"❌ Failed to parse true_robot_pose: {e}")
                
            if event["type"] == "INPUT" and event["id"] == "tick":
                tick_count += 1
                
                # 定期输出日志状态
                current_time = time.time()
                should_log = (tick_count % 100 == 0 or 
                            current_time - last_log_time >= 5 or 
                            tick_count <= 3)
                
                if should_log:
                    logger.info(f"Processing tick #{tick_count}")
                    last_log_time = current_time
                
                # 计算角速度（基于真实机器人角度变化）
                dt = current_time - last_time
                if dt > 0:
                    angular_velocity_z = (true_robot_pose["theta"] - last_theta) / dt
                else:
                    angular_velocity_z = 0.0
                
                last_theta = true_robot_pose["theta"]
                last_time = current_time
                
                # 生成IMU数据（基于真实机器人角度）
                try:
                    # 将欧拉角转换为四元数
                    theta = true_robot_pose["theta"]
                    qw = math.cos(theta / 2)
                    qx = 0.0
                    qy = 0.0
                    qz = math.sin(theta / 2)
                    
                    # 构建IMU消息
                    imu_data = {
                        "header": {
                            "seq": imu_count,
                            "stamp": {
                                "sec": int(current_time),
                                "nsec": int((current_time % 1) * 1e9)
                            },
                            "frame_id": "/imu_link"
                        },
                        "orientation": {
                            "w": qw,
                            "x": qx,
                            "y": qy,
                            "z": qz
                        },
                        "orientation_covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
                        "angular_velocity": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": angular_velocity_z
                        },
                        "angular_velocity_covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
                        "linear_acceleration": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 9.81
                        },
                        "linear_acceleration_covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
                    }
                    
                    imu_json = json.dumps(imu_data)
                    imu_count += 1
                    
                    # 发送JSON字符串的字节数组（转换为UInt8）
                    imu_bytes = imu_json.encode('utf-8')
                    imu_array = pa.array(list(imu_bytes), type=pa.uint8())
                    
                    # 发送数据
                    node.send_output("data", imu_array, {"timestamp": current_time})
                    
                    if should_log:
                        logger.info(f"✅ Generated IMU #{imu_count}: "
                                  f"theta={theta:.3f}, orient=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}) "
                                  f"ωz={angular_velocity_z:.3f}")
                    
                except Exception as e:
                    logger.error(f"❌ Failed to generate IMU data: {e}")
                    
    except KeyboardInterrupt:
        logger.info("🛑 IMU simulation interrupted")
    except Exception as e:
        logger.error(f"❌ Fatal error in IMU simulation: {e}")
    finally:
        logger.info(f"⏹️ IMU simulation stopped (processed {tick_count} ticks, generated {imu_count} measurements)")
    
    print("IMU simulation node stopped.", flush=True)

if __name__ == "__main__":
    main()