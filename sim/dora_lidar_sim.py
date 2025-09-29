#!/usr/bin/env python3
"""
Dora雷达仿真节点 - 使用正确的Dora Node API
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
    format='[%(asctime)s] [LIDAR_SIM] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def main():
    """Dora Lidar仿真节点主函数"""
    logger.info("Starting Dora Lidar Simulation Node...")
    
    # 初始化雷达仿真器
    simulator = RobotSimulator()
    
    # Get node ID from environment variable
    node_id = os.getenv("DORA_NODE_ID", "lidar_sim")
    node = Node(node_id)
    
    tick_count = 0
    scan_count = 0
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
                should_log = (tick_count % 200 == 0 or 
                            current_time - last_log_time >= 5 or 
                            tick_count <= 3)
                
                if should_log:
                    logger.info(f"Processing tick #{tick_count}")
                    last_log_time = current_time
                
                # 更新仿真状态
                dt = 0.01  # 10ms对应100Hz
                simulator.update_simulation(dt)
                
                # 生成雷达数据
                try:
                    lidar_json = simulator.simulate_lidar_scan()
                    
                    scan_count += 1
                    
                    # 发送JSON字符串的字节数组（转换为UInt8）
                    lidar_bytes = lidar_json.encode('utf-8')
                    scan_array = pa.array(list(lidar_bytes), type=pa.uint8())
                    
                    # 发送数据
                    node.send_output("scan", scan_array, {"timestamp": current_time})
                    
                    if should_log:
                        # 解析JSON以获取有效射线数量用于日志
                        lidar_data = json.loads(lidar_json)
                        ranges = lidar_data.get('ranges', [])
                        valid_ranges = [r for r in ranges if isinstance(r, (int, float)) and not math.isnan(r) and r != "NaN"]
                        logger.info(f"✅ Generated scan #{scan_count} - {len(valid_ranges)} valid rays, "
                                  f"robot pos:({simulator.x:.2f},{simulator.y:.2f},{simulator.theta:.2f})")
                    
                except Exception as e:
                    logger.error(f"❌ Failed to generate lidar scan: {e}")
                    
    except KeyboardInterrupt:
        logger.info("🛑 Lidar simulation interrupted")
    except Exception as e:
        logger.error(f"❌ Fatal error in lidar simulation: {e}")
    finally:
        logger.info(f"⏹️ Lidar simulation stopped (processed {tick_count} ticks, generated {scan_count} scans)")
    
    print("Lidar simulation node stopped.", flush=True)

if __name__ == "__main__":
    main()