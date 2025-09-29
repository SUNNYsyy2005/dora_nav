# dora_nav 项目开发环境搭建指南

这份文档详细说明如何搭建dora_nav项目的完整开发环境，包括所有必需的依赖库的安装。

## 📋 目录

- [系统要求](#系统要求)
- [核心依赖](#核心依赖)
- [开发工具](#开发工具)
- [语言环境](#语言环境)
- [第三方库](#第三方库)
- [Dora框架](#dora框架)
- [验证安装](#验证安装)
- [常见问题](#常见问题)

## 🖥️ 系统要求

- **操作系统**: Ubuntu 20.04 或更高版本
- **架构**: x86_64
- **内存**: 至少 8GB RAM
- **存储**: 至少 10GB 可用空间

## 🔧 核心依赖

### 1. 基础构建工具

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    tar \
    unzip
```

### 2. 开发库

```bash
sudo apt-get install -y \
    libpthread-stubs0-dev \
    libdl-dev \
    librt-dev
```

## 🔬 开发工具

### 1. C/C++ 开发工具

```bash
sudo apt-get install -y \
    gcc \
    g++ \
    gdb \
    clang \
    clang-tools \
    valgrind \
    cppcheck
```

### 2. Python 开发环境

```bash
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv
```

### 3. Rust 环境 (用于control模块)

```bash
# 安装Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# 验证安装
rustc --version
cargo --version
```

## 🌐 语言环境

### 1. Python包管理

```bash
# 升级pip
python3 -m pip install --upgrade pip

# 安装常用Python包
pip3 install \
    pyarrow \
    numpy \
    matplotlib
```

## 📚 第三方库

### 2. OpenCV

```bash
sudo apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv
```

### 3. Eigen3

```bash
sudo apt-get install -y \
    libeigen3-dev \
    libeigen3-doc
```

### 4. Boost

```bash
sudo apt-get install -y \
    libboost-all-dev \
    libboost-system1.71.0 \
    libboost-thread1.71.0
```

### 5. YAML-CPP

```bash
sudo apt-get install -y \
    libyaml-cpp-dev
```

### 6. SuiteSparse

```bash
sudo apt-get install -y \
    libsuitesparse-dev \
    libcamd2 \
    libcolamd2 \
    libcholmod3 \
    libccolamd2
```

### 7. FLANN

```bash
sudo apt-get install -y \
    libflann-dev \
    libflann1.9
```

### 8. LZ4

```bash
sudo apt-get install -y \
    liblz4-dev \
    liblz4-1
```

### 9. libpcap

```bash
sudo apt-get install -y \
    libpcap-dev \
    libpcap0.8
```

### 10. libserial

```bash
sudo apt-get install -y \
    libserial-dev \
    libserial1
```

### 11. spdlog

```bash
sudo apt-get install -y \
    libspdlog-dev \
    libspdlog1
```

## 📦 手动编译的第三方库

某些库需要手动编译到`third_party`目录：

### 1. nlohmann/json

```bash
cd /home/sunny/dora_nav
mkdir -p third_party/nlohmann_json/include
cd third_party/nlohmann_json
git clone https://github.com/nlohmann/json.git src
cp src/single_include/nlohmann/json.hpp include/
```

### 2. G2O (TEB模块专用)

```bash
cd /home/sunny/dora_nav/third_party
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 74517458bd1e0c2a0bda8b07ec91b1b97b2e4b4d  # 2023-07-30的稳定版本
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

## 🛠️ Dora框架

### 1. 安装Dora

```bash
# 克隆Dora仓库
git clone https://github.com/dora-rs/dora.git ~/dora
cd ~/dora

# 编译Dora
cargo build --release

# 设置环境变量
export PATH=$HOME/dora/target/release:$PATH
echo 'export PATH="$HOME/dora/target/release:$PATH"' >> ~/.bashrc

# 验证Dora安装
dora --version
```

## ✅ 验证安装

### 1. 运行完整性检查

```bash
cd /home/sunny/dora_nav

# 设置环境变量
export DORA_NAV_ROOT=/home/sunny/dora_nav

# 检查所有模块编译
./compile.sh --all -v
```

### 2. 测试各个模块

```bash
# 测试python仿真节点
python3 sim/dora_lidar_sim.py --test
python3 sim/dora_imu_sim.py --test

# 测试Dora数据流
dora start sim_dataflow.yml
```

### 3. 依赖检查脚本

创建`check_dependencies.sh`来验证所有依赖：

```bash
#!/bin/bash

echo "=== Dora Nav 依赖检查 ==="

# 检查基础工具
echo "检查基础工具..."
for tool in gcc g++ cmake python3 pip3 cargo; do
    if command -v $tool &> /dev/null; then
        echo "✅ $tool: $(which $tool)"
    else
        echo "❌ $tool: 未安装"
    fi
done

# 检查库文件
echo -e "\n检查C++库..."
for lib in libopencv_core.so libeigen3 libboost_system libyaml-cpp libcholmod g2o libflann liblz4 libspdlog; do
    if ldconfig -p | grep -q $lib; then
        echo "✅ $lib: 已安装"
    else
        echo "❌ $lib: 未安装"
        echo "   安装命令: sudo apt-get install lib$(basename $lib .so)-dev"
    fi
done

# 检查手动编译的库
echo -e "\n检查手动编译库..."
if [ -f "third_party/nlohmann_json/include/json.hpp" ]; then
    echo "✅ nlohmann/json: 已安装"
else
    echo "❌ nlohmann/json: 未安装"
fi

if [ -f "third_party/g2o/lib/libg2o_core.so" ]; then
    echo "✅ G2O: 已安装"
else
    echo "❌ G2O: 未安装"
fi

echo -e "\n=== 检查完成 ==="
```

## 🐛 常见问题

### 问题1: CMake找不到OpenCV

```bash
# 解决方案
sudo apt-get install libopencv-dev
pkg-config --modversion opencv4
```

### 问题2: Eigen3头文件找不到

```bash
# 解决方案
sudo apt-get install libeigen3-dev
find /usr/include -name "Eigen" -type d
```

### 问题3: G2O编译错误

```bash
# 确保使用正确的G2O版本
cd third_party/g2o
git checkout 74517458bd1e0c2a0bda8b07ec91b1b97b2e4b4d

# 重新编译
mkdir -p build && cd build
rm -rf *
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 问题4: Dora运行错误

```bash
# 检查Dora环境变量
export PATH=$HOME/dora/target/release:$PATH
export DORA_NAV_ROOT=/home/sunny/dora_nav

# 验证Dora安装
which dora
dora --version
```

### 问题5: Python节点无法导入

```bash
# 安装Python依赖
pip3 install pyarrow numpy matplotlib

# 检查Python环境
python3 -c "import dora; print('Dora OK')"
```

## 📄 完整的安装脚本

您还可以使用一键安装脚本：

```bash
#!/bin/bash
# 保存为 install_dependencies.sh

set -e

echo "开始安装dora_nav项目依赖..."

# 更新包管理器
sudo apt-get update

# 安装基础工具和库
sudo apt-get install -y \
    build-essential cmake pkg-config git \
    gcc g++ gdb clang \
    python3 python3-pip python3-dev \
    libopencv-dev libeigen3-dev libboost-all-dev \
    libyaml-cpp-dev libsuitesparse-dev libflann-dev \
    liblz4-dev libpcap-dev libserial-dev libspdlog-dev

# 安装Rust
if ! command -v cargo &> /dev/null; then
    echo "安装Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source ~/.cargo/env
fi

# 安装nlohmann/json
echo "安装nlohmann/json..."
mkdir -p ~/dora_nav/third_party/nlohmann_json/include
cd ~/dora_nav/third_party/nlohmann_json
curl -L https://github.com/nlohmann/json/releases/latest/download/json.hpp -o include/json.hpp

# 设置环境变量
echo 'export DORA_NAV_ROOT=~/dora_nav' >> ~/.bashrc
echo 'export PATH="$HOME/dora/target/release:$PATH"' >> ~/.bashrc

echo "✅ 依赖安装完成!"
echo "请注意: 请手动安装G2O和Dora框架"
echo "重启终端后运行: source ~/.bashrc"
```

## 📞 获取帮助

如果在安装过程中遇到问题：

1. 查看项目README.md
2. 检查doc/BUILD_GUIDE.md
3. 查看具体的错误日志
4. 验证系统版本和架构匹配

---

*更新时间: 2024年9月*
*版本: v1.0*
