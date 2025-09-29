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
    
    # 初始化IMU仿真器
    simulator = RobotSimulator()
    
    # Get node ID from environment variable
    node_id = os.getenv("DORA_NODE_ID", "imu_sim")
    node = Node(node_id)
    
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
                
                # 更新仿真状态
                dt = 0.02  # 20ms对应50Hz
                simulator.update_simulation(dt)
                
                # 生成IMU数据
                try:
                    imu_json = simulator.simulate_imu_data()
                    
                    imu_count += 1
                    
                    # 发送JSON字符串的字节数组（转换为UInt8）
                    imu_bytes = imu_json.encode('utf-8')
                    imu_array = pa.array(list(imu_bytes), type=pa.uint8())
                    
                    # 发送数据
                    node.send_output("data", imu_array, {"timestamp": current_time})
                    
                    if should_log:
                        # 解析JSON以获取IMU数据用于日志
                        imu_data = json.loads(imu_json)
                        orientation = imu_data.get('orientation', {})
                        angular_vel = imu_data.get('angular_velocity', {})
                        logger.info(f"✅ Generated IMU #{imu_count}: "
                                  f"orient=({orientation.get('w',0):.3f},{orientation.get('x',0):.3f},{orientation.get('y',0):.3f},{orientation.get('z',0):.3f}) "
                                  f"ωz={angular_vel.get('z',0):.3f}")
                    
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