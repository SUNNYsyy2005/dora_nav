#!/bin/bash
# DoraNav 项目环境设置脚本
# 使用方法: source setup_env.sh 或 ./setup_env.sh

# 自动检测项目根目录（当前脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

echo "=== DoraNav 环境设置 ==="
echo "项目根目录: $PROJECT_ROOT"

# 设置环境变量
export DORA_NAV_ROOT="$PROJECT_ROOT"

# 添加到环境变量中以供下次启动时使用
echo "" >> ~/.bashrc
echo "# DoraNav Project Environment" >> ~/.bashrc
echo "export DORA_NAV_ROOT=\"$PROJECT_ROOT\"" >> ~/.bashrc

echo "环境变量 DORA_NAV_ROOT 已设置为: $DORA_NAV_ROOT"
echo "环境变量已添加到 ~/.bashrc，重新打开终端即可自动加载"

# 验证一些关键路径
echo ""
echo "=== 路径验证 ==="
echo "CMake配置文件: $PROJECT_ROOT/cmake_config.cmake"
echo "TEB模块: $PROJECT_ROOT/build/teb/"
echo "NAV模块: $PROJECT_ROOT/build/nav/"
echo "AMCL模块: $PROJECT_ROOT/build/amcl/"

echo ""
echo "=== 使用方法 ==="
echo "1. 首次运行: source setup_env.sh"
echo "2. 重新启动终端后环境变量会自动加载"
echo "3. 运行程序时会自动使用正确的项目路径"
echo ""
echo "项目路径管理系统已经配置完成！"
