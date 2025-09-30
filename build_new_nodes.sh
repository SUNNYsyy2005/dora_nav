#!/bin/bash

# 构建新的机器人导航节点脚本
# 使用方法: ./build_new_nodes.sh

set -e

echo "=========================================="
echo "构建位姿融合和TF管理节点"
echo "=========================================="

# 检查必要的环境和依赖
source setup_env.sh

# 创建构建目录
mkdir -p build/pose_fusion/build
mkdir -p build/tf_manager/build

cd build

echo "开始构建位姿融合节点..."
cd pose_fusion
mkdir -p build
cd build
cmake ..
make -j$(nproc)
echo "位姿融合节点构建完成"

cd ..

echo "开始构建TF管理节点..."
cd ../tf_manager
mkdir -p build
cd build
cmake ..
make -j$(nproc)
echo "TF管理节点构建完成"

cd ../..

echo "=========================================="
echo "所有新节点构建完成！"
echo "=========================================="

echo "生成的二进制文件："
echo "  - build/pose_fusion/build/test"
echo "  - build/tf_manager/build/test"

echo ""
echo "测试建议："
echo "1. 可以单独测试位姿融合节点："
echo "   cd build/pose_fusion && dora up dataflow.yml"
echo ""
echo "2. 可以使用完整的数据流："
echo "   dora up enhanced_dataflow.yml"

