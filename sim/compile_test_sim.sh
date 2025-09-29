#!/bin/bash

# 编译测试仿真数据的C++节点

set -e

PROJECT_ROOT="/home/sunny/dora_nav"
SIM_DIR="$PROJECT_ROOT/sim"

echo "=== 编译仿真数据测试节点 ==="

# 进入sim目录
cd "$SIM_DIR"

# 创建build目录
mkdir -p build
cd build

# 配置CMake
echo "配置CMake..."
cmake ..

# 编译
echo "编译..."
make -j$(nproc)

echo "编译完成！"

# 测试可执行文件
if [ -f "./test_sim_data_node" ]; then
    echo "✓ 可执行文件创建成功"
    ls -la ./test_sim_data_node
else
    echo "✗ 编译失败"
    exit 1
fi

echo "=== 使用说明 ==="
echo "1. 运行仿真数据流:"
echo "   cd $PROJECT_ROOT"
echo "   dora dataflow sim/test_sim_dataflow.yml"
echo ""
echo "2. 或者手动测试节点:"
echo "   cd $SIM_DIR/build"
echo "   ./test_sim_data_node"
echo ""
echo "节点将接收并分析Python仿真节点的数据"
