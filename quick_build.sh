#!/bin/bash

# DoraNav 快速编译脚本
# 专门用于开发时的快速编译和测试

set -e  # 遇到错误立即退出

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# 快速编译函数
quick_build() {
    local module=$1
    echo -e "${GREEN}快速编译: $module${NC}"
    
    cd "/home/sunny/dora_nav/build/$module"
    
    # 检查是否需要CMake配置
    if [[ ! -d "build" ]] || [[ ! -f "build/Makefile" ]]; then
        echo "CMake配置..."
        mkdir -p build
        cd build
        cmake .. >/dev/null 2>&1
    else
        cd build
    fi
    
    echo "编译中..."
    make -j$(nproc) >/dev/null 2>&1
    
    echo -e "${GREEN}✓ $module 编译完成${NC}"
    cd /home/sunny/dora_nav
}

# 主函数
if [[ $# -eq 0 ]]; then
    echo "快速编译模式 - 编译所有模块"
    for module in nav slam amcl lidar imu teb; do
        quick_build $module
    done
    echo -e "${GREEN}🎉 所有模块快速编译完成！${NC}"
else
    for module in "$@"; do
        quick_build $module
    done
fi
