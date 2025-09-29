#!/usr/bin/env python3
"""
符合Dora规范的Python测试节点
"""

import sys
import json
import time

def main():
    """
    Dora节点的主函数应该保持运行并等待事件
    根据Dora官方文档，Python节点通过stdin/stdout与Dora通信
    """
    print("Dora test node starting...", flush=True)
    
    try:
        # 发送准备完成的信号给Dora
        ready_msg = {
            "type": "ready"
        }
        print(json.dumps(ready_msg), flush=True)
        
        # 持续读取stdin
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
                
            print(f"Received from Dora: {line}", flush=True)
            
            # 解析Dora输入消息
            try:
                dora_msg = json.loads(line)
                
                if dora_msg.get("type") == "message":
                    # 发送输出消息
                    output_msg = {
                        "type": "output",
                        "output": "test_output",
                        "data": f"Echo: {dora_msg.get('message', 'no message')}"
                    }
                    print(json.dumps(output_msg), flush=True)
                    
                elif dora_msg.get("type") == "stop":
                    print("Received stop signal", flush=True)
                    break
                    
            except json.JSONDecodeError:
                print(f"Invalid JSON: {line}", flush=True)
                continue
                
    except KeyboardInterrupt:
        print("Interrupted by user", flush=True)
    except Exception as e:
        print(f"Error: {e}", flush=True)
    finally:
        print("Test node shutting down", flush=True)

if __name__ == "__main__":
    main()
