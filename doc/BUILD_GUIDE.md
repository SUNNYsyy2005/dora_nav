# DoraNav 编译指南

## 📦 编译脚本

本项目提供了两个编译脚本来简化编译过程：

### 1. 完整功能编译脚本 (`compile.sh`)

提供完整的编译功能，支持多种编译选项。

#### 使用方法

```bash
# 编译所有模块
./compile.sh --all  # 明确编译所有模块
./compile.sh        # 编译所有模块（默认行为）

# 编译指定模块
./compile.sh nav
./compile.sh amcl nav slam

# 设置并行作业数
./compile.sh -j 8 nav

# 显示详细输出
./compile.sh -v nav

# 清理编译文件
./compile.sh -c

# 显示帮助信息
./compile.sh -h

# 列出所有可用模块
./compile.sh -l

# 编译后运行测试
./compile.sh --test nav
```

#### 可用模块

| 模块名 | 描述 | 状态 |
|--------|------|------|
| `amcl` | AMCL - 自适应蒙特卡洛定位 | ✅ |
| `nav` | NAV - 路径规划 | ✅ |
| `slam` | SLAM - 同步定位与建图 | ✅ |
| `lidar` | LIDAR - 激光雷达驱动 | ✅ |
| `imu` | IMU - 惯性测量单元 | ✅ |
| `teb` | TEB - 时序弹性带局部规划器 | ✅ |
| `control` | CONTROL - 运动控制 | ✅ |

### 📋 支持的编译选项

| 选项 | 功能 | 示例 |
|------|------|------|
| `-h, --help` | 显示帮助信息 | `./compile.sh -h` |
| `-l, --list` | 列出可用模块 | `./compile.sh -l` |
| `-c, --clean` | 清理编译文件 | `./compile.sh -c` |
| `-a, --all` | 明确编译所有模块 | `./compile.sh --all` |
| `-j, --jobs N` | 设置并行作业数 | `./compile.sh -j 8 nav` |
| `-v, --verbose` | 详细输出 | `./compile.sh -v nav` |
| `--test` | 编译后测试 | `./compile.sh --test nav` |

#### `--all` 参数说明

- **`--all`**: 明确指定编译所有模块，会显示"使用 --all 参数编译所有模块"的提示信息
- **默认行为**: 当没有指定任何模块时，也会编译所有模块，但显示的提示是"未指定模块，将编译所有模块"
- **组合使用**: `--all` 可以与其他选项组合，如 `./compile.sh --all --test` 编译所有模块后运行测试

### 2. 快速编译脚本 (`quick_build.sh`)

适用于开发时的快速编译，输出简洁，专注于编译速度。

#### 使用方法

```bash
# 快速编译所有模块
./quick_build.sh

# 快速编译指定模块
./quick_build.sh nav slam
./quick_build.sh teb
```

## 🚀 快速开始

### 环境设置

```bash
# 1. 设置环境变量
source setup_env.sh

# 2. 编译所有模块
./compile.sh

# 或者快速编译
./quick_build.sh
```

### 常见使用场景

#### 开发调试单个模块

```bash
# 编译并查看详细输出
./compile.sh -v nav

# 快速重新编译
./quick_build.sh nav
```

#### 完整项目构建

```bash
# 清理之前的编译文件
./compile.sh -c

# 编译所有模块
./compile.sh
```

#### 并行编译优化

```bash
# 使用更多CPU核心
./compile.sh -j $(nproc) nav slam

# 限制CPU使用
./compile.sh -j 4 nav
```

#### 问题排查

```bash
# 显示详细编译信息
./compile.sh -v nav

# 清理后重新编译
./compile.sh -c
./compile.sh nav
```

## 📋 手动编译

如果你想手动编译模块，可以参考以下步骤：

### AMCL模块

```bash
cd build/amcl
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### NAV模块

```bash
cd build/nav
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### SLAM模块

```bash
cd build/slam
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### TEB模块

```bash
cd build/teb
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### LIDAR模块

```bash
cd build/lidar
make -j$(nproc)
```

### IMU模块

```bash
cd build/imu
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### CONTROL模块

```bash
cd control
cargo build --release
```

## 🔧 配置说明

### 依赖检查

编译脚本会自动检查以下依赖：
- CMake
- Make  
- 项目配置文件
- 环境变量设置

### 并行编译

默认使用系统所有CPU核心进行并行编译：
- 脚本会自动检测CPU核心数
- 可通过 `-j N` 参数手动设置
- 建议设置为CPU核心数的75%-100%

### 错误处理

- 编译失败时会显示具体错误信息
- 支持继续编译其他模块（即使部分模块失败）
- 提供清理和重新编译选项

## 🛠️ 故障排除

### 常见问题

1. **CMake配置失败**
   ```bash
   # 清理build目录后重新配置
   rm -rf build/build
   ./compile.sh nav
   ```

2. **环境变量未设置**
   ```bash
   # 设置环境变量
   source setup_env.sh
   ```

3. **权限问题**
   ```bash
   # 添加执行权限
   chmod +x compile.sh
   chmod +x quick_build.sh
   ```

4. **依赖包缺失**
   ```bash
   # 参考README.md安装系统依赖
   sudo apt-get install cmake make g++ ...
   ```

### 编译输出

- `✓` 表示编译成功
- `✗` 表示编译失败  
- 详细的错误信息会显示在输出中

## 📊 性能优化

### 编译速度优化

1. **使用SSD存储**
2. **增加RAM**
3. **设置合理的并行作业数**
4. **定期清理编译缓存**

### 推荐配置

- **开发环境**: `./quick_build.sh -j $(nproc)`
- **生产环境**: `./compile.sh -j $(($(nproc) * 75 / 100))`
- **调试模式**: `./compile.sh -v module_name`

编译脚本使项目构建过程更加简化和自动化，大大提高了开发效率！
