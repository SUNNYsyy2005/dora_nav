#!/bin/bash

echo "=== Dora Nav 依赖检查 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查基础工具
echo -e "\n${YELLOW}🔧 检查基础工具...${NC}"
mkdir -p /tmp/dora_check_result

for tool in gcc g++ cmake python3 pip3 cargo gdb; do
    if command -v $tool &> /dev/null; then
        echo -e "✅ ${GREEN}$tool${NC}: $(which $tool)"
        echo "$tool: $(which $tool)" >> /tmp/dora_check_result/tools.txt
    else
        echo -e "❌ ${RED}$tool${NC}: 未安装"
        echo "安装命令: sudo apt-get install $tool"
    fi
done

# 检查版本信息
echo -e "\n${YELLOW}📋 版本信息:${NC}"
if command -v gcc &> /dev/null; then
    echo "   GCC: $(gcc --version | head -n1)"
fi
if command -v cmake &> /dev/null; then
    echo "   CMake: $(cmake --version | head -n1)"
fi
if command -v python3 &> /dev/null; then
    echo "   Python: $(python3 --version)"
fi
if command -v cargo &> /dev/null; then
    echo "   Rust: $(rustc --version)"
fi

# 检查C++库
echo -e "\n${YELLOW}📚 检查C++库文件...${NC}"
for lib in "opencv4" "eigen3" "boost_system" "yaml-cpp" "suitesparse" "flann" "lz4" "spdlog" "serial"; do
    found=false
    case $lib in
        "opencv4")
            if pkg-config --exists opencv4; then
                echo -e "✅ ${GREEN}OpenCV${NC}: $(pkg-config --modversion opencv4)"
                found=true
            fi
            ;;
        "eigen3")
            if find /usr/include -name "Eigen" -type d 2>/dev/null | grep -q Eigen; then
                echo -e "✅ ${GREEN}Eigen3${NC}: $(find /usr/include -name "Eigen" -type d | head -1)"
                found=true
            fi
            ;;
        *)
            if ldconfig -p 2>/dev/null | grep -q $lib; then
                echo -e "✅ ${GREEN}$lib${NC}: 已安装"
                found=true
            fi
            ;;
    esac
    
    if [ "$found" = false ]; then
        echo -e "❌ ${RED}$lib${NC}: 未安装"
        case $lib in
            "boost_system") echo "   安装命令: sudo apt-get install libboost-all-dev" ;;
            "yaml-cpp") echo "   安装命令: sudo apt-get install libyaml-cpp-dev" ;;
            "suitesparse") echo "   安装命令: sudo apt-get install libsuitesparse-dev" ;;
            "flann") echo "   安装命令: sudo apt-get install libflann-dev" ;;
            "lz4") echo "   安装命令: sudo apt-get install liblz4-dev" ;;
            "spdlog") echo "   安装命令: sudo apt-get install libspdlog-dev" ;;
            "serial") echo "   安装命令: sudo apt-get install libserial-dev" ;;
        esac
    fi
done

# 检查手动编译的库
echo -e "\n${YELLOW}📦 检查手动编译的库...${NC}"

# 检查nlohmann/json
if [ -f "third_party/nlohmann_json/include/nlohmann/json.hpp" ]; then
    echo -e "✅ ${GREEN}nlohmann/json${NC}: 已安装"
else
    echo -e "❌ ${RED}nlohmann/json${NC}: 未安装"
    echo "   安装命令:"
    echo "   mkdir -p third_party/nlohmann_json/include"
    echo "   cd third_party/nlohmann_json"
    echo "   curl -L https://github.com/nlohmann/json/releases/latest/download/json.hpp -o include/nlohmann/json.hpp"
fi

# 检查G2O
if [ -f "third_party/g2o/build/lib/libg2o_core.so" ] || [ -f "third_party/g2o/lib/libg2o_core.so" ]; then
    echo -e "✅ ${GREEN}G2O${NC}: 已安装"
else
    echo -e "❌ ${RED}G2O${NC}: 未安装"
    echo "   安装命令:"
    echo "   cd third_party"
    echo "   git clone https://github.com/RainerKuemmerle/g2o.git"
    echo "   cd g2o"
    echo "   git checkout 74517458bd1e0c2a0bda8b07ec91b1b97b2e4b4d"
    echo "   mkdir build && cd build"
    echo "   cmake -DCMAKE_BUILD_TYPE=Release .."
    echo "   make -j\$(nproc)"
fi

# 检查Dora框架
echo -e "\n${YELLOW}🛠️  检查Dora框架...${NC}"
if [ -f "$HOME/dora/target/release/dora" ]; then
    echo -e "✅ ${GREEN}Dora${NC}: 已安装"
    if command -v dora &> /dev/null; then
        echo "   版本: $(dora --version 2>/dev/null || echo '未知')"
    fi
else
    echo -e "❌ ${RED}Dora${NC}: 未安装"
    echo "   安装命令:"
    echo "   git clone https://github.com/dora-rs/dora.git ~/dora"
    echo "   cd ~/dora"
    echo "   cargo build --release"
fi

# 检查环境变量
echo -e "\n${YELLOW}🌍 检查环境变量...${NC}"
if [ ! -z "$DORA_NAV_ROOT" ]; then
    echo -e "✅ ${GREEN}DORA_NAV_ROOT${NC}: $DORA_NAV_ROOT"
else
    echo -e "❌ ${RED}DORA_NAV_ROOT${NC}: 未设置"
    echo "   设置命令: export DORA_NAV_ROOT=/home/sunny/dora_nav"
fi

if echo $PATH | grep -q "dora/target/release"; then
    echo -e "✅ ${GREEN}Dora路径${NC}: 已在PATH中"
else
    echo -e "❌ ${RED}Dora路径${NC}: 未在PATH中"
    echo "   设置命令: export PATH=\"\$HOME/dora/target/release:\$PATH\""
fi

# 检查Python依赖
echo -e "\n${YELLOW}🐍 检查Python依赖...${NC}"
python3 -c "
import sys
required_packages = ['pyarrow', 'numpy', 'matplotlib']
missing = []
for pkg in required_packages:
    try:
        __import__(pkg)
        print(f'✅ ${pkg}: 已安装')
    except ImportError:
        missing.append(pkg)
        print(f'❌ ${pkg}: 未安装')

if missing:
    print(f'安装命令: pip3 install {\" \".join(missing)}')
"

# 总结
echo -e "\n${YELLOW}📊 部署状态总览:${NC}"

# 统计部分
tools_count=$(grep -c "✅" /tmp/dora_check_result/tools.txt 2>/dev/null || echo "0")
total_tools=6  # gcc, g++, cmake, python3, pip3, cargo

echo "基础工具: $tools_count/$total_tools"

# 清理临时文件
rm -rf /tmp/dora_check_result

echo -e "\n${YELLOW}=== 检查完成 ===${NC}"
echo "💡 提示: 完整的安装指南请查看 doc/DEVELOPMENT_SETUP.md"
