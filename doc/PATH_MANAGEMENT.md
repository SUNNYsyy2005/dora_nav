# DoraNav 项目路径管理系統

## 概述

本项目实现了统一的路径管理系统，所有源代码中的硬编码绝对路径都已替换为基于项目根目录的动态路径方案。

## 系统架构

### 核心文件

1. **`include/project_paths.h`** - C++项目的统一路径管理头文件
2. **`include/project_paths_c.h`** - C项目的路径管理头文件
3. **`setup_env.sh`** - 环境变量设置脚本

### 使用方法

#### 方法1：环境变量设置（推荐）

1. 运行环境设置脚本：
   ```bash
   source setup_env.sh
   ```

2. 环境变量会自动添加到 ~/.bashrc，重新启动终端后会自动加载

#### 方法2：手动设置环境变量

```bash
export DORA_NAV_ROOT="/path/to/your/dora_nav"
```

#### 方法3：程序自动检测

程序会自动检测项目根目录，查找包含 "/dora_nav" 的路径。

## C++ 代码使用方法

在 C++ 文件中包含头文件：

```cpp
#include "../../include/project_paths.h"
```

使用路径函数：

```cpp
// 获取特定路径
std::string laser_data = ProjectPaths::build_nav_laser_data();
std::string path_csv = ProjectPaths::build_teb_path_csv();

// 或者构建自定义路径
std::string custom_path = build_path("build/custom/file.txt");
```

## C 代码使用方法

在 C 文件中包含头文件：

```c
#include "../../../include/project_paths_c.h"
```

使用路径函数：

```c
const char* output_path = get_amcl_output_path_c();
```

## 可用的路径函数

### C++ 路径函数（ProjectPaths 命名空间）

- `build_nav_laser_data()` - build/nav/laser_data.pgm
- `build_teb_path_csv()` - build/teb/path.csv  
- `build_slam_data()` - build/slam/laser_data.dat
- `amcl_txt()` - amcl.txt
- `teb_txt()` - teb.txt
- `nav_output_pgm()` - build/nav/output.pgm
- `nav_output2_pgm()` - build/nav/output2.pgm
- `nav_data_pgm(dataset)` - build/nav/{dataset}.pgm
- `amcl_output_pgm()` - build/amcl/output.pgm

### C 路径函数

- `get_amcl_output_path_c()` - build/amcl/output.pgm

## 已修复的文件

以下文件中的硬编码路径已被修复：

### C++ 文件
- `build/amcl/main.cc`
- `build/teb/main.cc` 
- `build/slam/main.cc`
- `build/slam/log2pgm.cc`
- `build/nav/A_star_dwa.cc`

### C 文件
- `build/amcl/map/map_store.c`

## project_paths.h vs project_paths_c.h

- **project_paths.h**: 用于C++项目，提供std::string类型的路径函数
- **project_paths_c.h**: 用于C项目，提供const char*类型的路径函数

## 优点

1. **可移植性** - 程序可以在不同路径下运行
2. **统一管理** - 所有路径配置集中在一处
3. **灵活配置** - 支持环境变量和自动检测
4. **类型安全** - C++使用std::string，C使用const char*
5. **向后兼容** - 如果检测失败，会回退到默认路径

## 编译测试

以下模块已验证可以正常编译：

- ✅ AMCL
- ✅ NAV  
- ✅ TEB (使用老版本g2o)
- ✅ IMU (移除snapshot后)
- ✅ LIDAR
- ✅ SLAM
- ✅ 所有模块统一使用项目路径系统

## 错误排查

如果编译时出现路径相关错误：

1. 检查环境变量设置：`echo $DORA_NAV_ROOT`
2. 确认头文件路径正确：检查 `#include` 语句的路径
3. 验证项目根目录确实包含预期文件
4. 检查 CMakeLists.txt 中的 include_directories 设置

项目路径管理系统已成功实现并测试完成！
