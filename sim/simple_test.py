#!/usr/bin/env python3
"""
简单的Dora连接测试
"""

import sys
import json

def main():
    print("Simple Dora test node starting...", flush=True)
    
    try:
        # 读取stdin直到结束
        line = sys.stdin.readline()
        count = 0
        
        while line and line.strip():
            count += 1
            print(f"Received line {count}: {line.strip()}", flush=True)
            
            # 简单响应
            response = {"type": "output", "output": "test", "data": f"echo_{count}"}
            print(json.dumps(response), flush=True)
            
            line = sys.stdin.readline()
            
    except Exception as e:
        print(f"Error: {e}", flush=True)
    
    print("Simple test completed", flush=True)

if __name__ == "__main__":
    main()
