#!/bin/bash

# 完整的TF集成版节点构建脚本
# 使用方法: ./build_tf_integrated_complete.sh

set -e

echo "=========================================="
echo "构建完整的TF集成版机器人导航节点"
echo "=========================================="

# 检查必要的环境和依赖
source setup_env.sh

# 创建构建目录
mkdir -p build/common
mkdir -p build/slam/build
mkdir -p build/amcl/build
mkdir -p build/teb/build
mkdir -p build/nav/build
mkdir -p build/pose_fusion/build
mkdir -p build/tf_manager/build

cd build

echo ""
echo "1. 生成800×800像素地图..."
cd ../sim
python3 map_generator_800x800.py
echo "地图生成完成"

cd ../build

echo ""
echo "2. 构建SLAM模块（TF集成版）..."
cd slam
# 复制TF集成文件
cp ../slam/tf_integration.h .
cp ../slam/main_tf_integrated.cc .
cp ../slam/main_with_yaml.cc .

cd build
cmake -f ../CMakeLists_tf.txt ..
make -j$(nproc)
echo "SLAM TF集成版构建完成"

cd ../..

echo ""
echo "3. 构建AMCL模块（TF集成版）..."
cd amcl
# 复制TF集成文件
cp ../amcl/tf_integration.h .
cp ../amcl/main_tf_integrated.cc .

cd build
cmake -f ../CMakeLists_tf.txt ..
make -j$(nproc)
echo "AMCL TF集成版构建完成"

cd ../..

echo ""
echo "4. 构建TEB模块（TF集成版）..."
cd teb
# 复制TF集成文件
cp ../teb/tf_integration.h .
cp ../teb/main_tf_integrated.cc .

cd build
cmake -f ../CMakeLists_tf.txt ..
make -j$(nproc)
echo "TEB TF集成版构建完成"

cd ../..

echo ""
echo "5. 构建NAV模块（TF集成版）..."
cd nav
# 复制TF集成文件
cp ../nav/tf_integration.h .
cp ../nav/main_tf_integrated.cc .

cd build
cmake -f ../CMakeLists_tf.txt ..
make -j$(nproc)
echo "NAV TF集成版构建完成"

cd ../..

echo ""
echo "6. 构建位姿融合模块..."
cd pose_fusion
cd build
cmake ..
make -j$(nproc)
echo "位姿融合模块构建完成"

cd ../..

echo ""
echo "7. 构建TF管理模块..."
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
echo "  - build/slam/build/test_with_yaml"
echo "  - build/amcl/build/test_tf_integrated"
echo "  - build/teb/build/test_tf_integrated"
echo "  - build/nav/build/test_tf_integrated"
echo "  - build/pose_fusion/build/test"
echo "  - build/tf_manager/build/test"

echo ""
echo "生成的地图文件："
echo "  - build/simulation_map_800x800.pgm（PGM地图）"
echo "  - build/simulation_map_800x800.yaml（地图元数据）"
echo "  - simulation_map_800x800.txt（ASCII查看）"

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

