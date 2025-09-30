#!/bin/bash

# TF集成版节点构建脚本
# 使用方法: ./build_tf_integrated_nodes.sh

set -e

echo "=========================================="
echo "构建TF集成版机器人导航节点"
echo "=========================================="

# 检查必要的环境和依赖
source setup_env.sh

# 创建构建目录
mkdir -p build/slam/build
mkdir -p build/amcl/build
mkdir -p build/teb/build
mkdir -p build/pose_fusion/build
mkdir -p build/tf_manager/build

cd build

echo "开始构建地图生成器..."
cd ../sim
python3 map_generator.py
echo "地图生成完成"

cd ../build

echo ""
echo "1. 构建SLAM模块（TF集成版）..."
cd slam
# 复制TF集成文件并重新命名
cp ../slam/tf_integration.h .
cp ../slam/main_tf_integrated.cc main_tf.cc

cd build
cmake ..
make -j$(nproc)
# 输出为test_tf_integrated
mv test test_tf_integrated
echo "SLAM TF集成版构建完成"

cd ../..

echo ""
echo "2. 构建AMCL模块（TF集成版）..."
cd amcl
# 复制TF集成文件并重新命名  
cp ../amcl/tf_integration.h .
cp ../amcl/main_tf_integrated.cc main_tf.cc

cd build
cmake ..
make -j$(nproc)
# 输出为test_tf_integrated
mv test test_tf_integrated
echo "AMCL TF集成版构建完成"

cd ../..

echo ""
echo "3. 构建TEB模块（TF集成版）..."
cd teb
# 复制TF集成文件并重新命名
cp ../teb/tf_integration.h .
cp ../teb/main_tf_integrated.cc main_tf.cc

cd build
cmake ..
make -j$(nproc)
# 输出为test_tf_integrated
mv test test_tf_integrated
echo "TEB TF集成版构建完成"

cd ../..

echo ""
echo "4. 构建位姿融合模块..."
cd pose_fusion
cd build
cmake ..
make -j$(nproc)
echo "位姿融合模块构建完成"

cd ../..

echo ""
echo "5. 构建TF管理模块..."
cd tf_manager
cd build
cmake ..
make -j$(nproc)
echo "TF管理模块构建完成"

cd ../..

echo "=========================================="
echo "所有TF集成节点构建完成！"
echo "=========================================="

echo "生成的二进制文件："
echo "  - build/slam/build/test_tf_integrated"
echo "  - build/amcl/build/test_tf_integrated"
echo "  - build/teb/build/test_tf_integrated"
echo "  - build/pose_fusion/build/test"
echo "  - build/tf_manager/build/test"

echo ""
echo "生成的地图文件："
echo "  - build/simulation_map.pgm（PGM地图）"
echo "  - build/simulation_map.yaml（地图元数据）"
echo "  - simulation_map_visualization.png（可视化）"

echo ""
echo "使用方法："
echo "1. 运行TF集成版数据流："
echo "   dora up enhanced_dataflow_tf_integrated.yml"
echo ""
echo "2. 运行原始数据流："
echo "   dora up enhanced_dataflow.yml"
echo ""
echo "3. 测试单个模块："
echo "   cd build/pose_fusion && dora up dataflow.yml"

