# 机器人仿真系统 (sim/)

这个目录包含完整的机器人仿真系统，用于模拟机器人在4m×4m矩形场地中的激光雷达和IMU传感器数据。

## 📁 文件结构

```
sim/
├── README.md                    # 本说明文档
├── robot_simulator.py           # 核心仿真器类
├── dora_lidar_sim.py           # Dora激光雷达仿真节点
├── dora_imu_sim.py             # Dora IMU仿真节点
├── test_sim_data_node.cc       # C++测试节点源码
├── CMakeLists_test_sim.txt     # C++测试节点CMake配置
├── compile_test_sim.sh         # C++测试节点编译脚本
└── build/                      # C++测试节点编译输出
    └── test_sim_data_node      # 编译后的可执行文件
```

## 🚀 快速开始

### 1. 测试Python仿真器
```bash
cd /home/sunny/dora_nav/sim
python3 robot_simulator.py
```

### 2. 编译C++测试节点
```bash
cd /home/sunny/dora_nav/sim
bash compile_test_sim.sh
```

### 3. 运行完整仿真测试
```bash
cd /home/sunny/dora_nav
dora dataflow sim_dataflow.yml
```

### 4. 运行简化测试
```bash
cd /home/sunny/dora_nav
dora dataflow test_sim_dataflow.yml
```

## 🔧 文件说明

### Core Files

#### `robot_simulator.py`
- **功能**: 核心仿真器类，包含所有物理和环境仿真逻辑
- **特性**: 
  - 4m×4m矩形场地仿真
  - 3个不对称障碍物
  - 顺时针绕圈运动模式
  - 激光雷达和IMU数据生成
  - 精确的射线-障碍物碰撞检测

#### `dora_lidar_sim.py`
- **功能**: 激光雷达仿真的Dora节点包装器
- **输入**: Dora事件流
- **输出**: JSON格式的激光雷达数据
- **更新频率**: 10Hz

#### `dora_imu_sim.py`
- **功能**: IMU仿真的Dora节点包装器
- **输入**: Dora事件流
- **输出**: JSON格式的IMU数据
- **更新频率**: 20Hz

### Test Files

#### `test_sim_data_node.cc`
- **功能**: C++测试节点，验证Python仿真数据
- **特性**:
  - 接收并解析激光雷达和IMU JSON数据
  - 数据格式验证和统计
  - 实时数据分析和展示
  - 兼容性测试

#### `CMakeLists_test_sim.txt` & `compile_test_sim.sh`
- **功能**: C++测试节点的编译配置和脚本
- **依赖**: nlohmann/json, OpenCV, Dora API

### Configuration Files

#### `sim_dataflow.yml`
- **功能**: 完整的仿真数据流配置
- **节点**: lidar_sim, imu_sim, amcl_sim, teb_sim, control_sim
- **用途**: 完整的导航栈仿真测试

#### `test_sim_dataflow.yml`
- **功能**: 简化的测试数据流配置
- **节点**: lidar_sim, imu_sim, test_sim_data
- **用途**: 验证仿真数据格式和内容

## 📊 数据格式

### 激光雷达数据
```json
{
  "header": {"seq": 0, "stamp": {...}, "frame_id": "laser_frame"},
  "angle_min": -2.356, "angle_max": 2.356,
  "angle_increment": 0.0044,
  "range_min": 0.1, "range_max": 10.0,
  "ranges": [1081个距离值],
  "intensities": [1081个强度值]
}
```

### IMU数据
```json
{
  "header": {"seq": 0, "stamp": {...}, "frame_id": "/imu_link"},
  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
  "angular_velocity": {"x": 0, "y": 0, "z": 0.01},
  "linear_acceleration": {"x": 0, "y": 0, "z": 9.81}
}
```

## 🎯 使用场景

### 开发测试
```bash
# 快速验证仿真逻辑
python3 robot_simulator.py

# 验证数据格式兼容性
bash compile_test_sim.sh
cd build && ./test_sim_data_node
```

### 集成测试
```bash
# 完整导航栈仿真
dora dataflow sim/sim_dataflow.yml

# 简化数据流测试
dora dataflow sim/test_sim_dataflow.yml
```

### 调试分析
- **C++测试节点**: 提供详细的传感器数据统计和分析
- **实时监控**: 显示数据包数量、时间间隔、数据有效性
- **格式验证**: 检查JSON格式、数据范围和物理合理性

## 🔧 自定义配置

### 修改仿真环境
编辑 `robot_simulator.py`:
```python
# 场地尺寸
FIELD_WIDTH = 4.0   # 米
FIELD_HEIGHT = 4.0  # 米

# 障碍物位置和尺寸 (x, y, width, height)
OBSTACLES = [
    (1.5, 1.0, 0.5, 0.3),  # 可以添加新障碍物
]

# 机器人运动参数
linear_speed = 0.5  # m/s 移动速度
```

### 修改传感器参数
编辑 `robot_simulator.py` 中的传感器函数:
```python
# 激光雷达参数
range_min = 0.1     # 最小距离
range_max = 10.0    # 最大距离
angle_resolution = 0.25  # 角度分辨率

# IMU噪声参数
orientation_noise = 0.005   # 方向噪声
angular_vel_noise = 0.01    # 角速度噪声
accel_noise = 0.02         # 加速度噪声
```

## 📈 性能特性

- **仿真精度**: 精确的物理射线-障碍物碰撞检测
- **计算效率**: 优化的算法，实时仿真无延迟
- **数据真实性**: 包含噪声、边界条件等真实传感器特性
- **格式兼容**: 完全兼容现有C++节点数据格式

## 🚨 故障排除

### 常见问题

1. **Python依赖缺失**
   ```bash
   pip install numpy
   ```

2. **C++编译失败**
   ```bash
   # 确保dora已经编译
   cd /home/sunny/dora
   cargo build --release
   ```

3. **数据流运行失败**
   ```bash
   # 检查文件权限
   chmod +x sim/*.py
   
   # 检查路径配置
   ls -la sim/dora_*.py
   ```

### 调试模式
启用详细输出:
```bash
# Python仿真器调试
python3 robot_simulator.py  # 显示详细测试信息

# C++测试节点调试
cd sim/build
./test_sim_data_node  # 显示数据分析和统计
```

---

## 📝 总结

这个仿真系统提供了：

✅ **完整的机器人传感器仿真** (激光雷达 + IMU)
✅ **真实的物理环境** (4m×4m场地 + 障碍物)
✅ **标准数据格式** (完全兼容C++节点)
✅ **C++测试验证** (数据格式和内容验证)
✅ **灵活的配置** (易于修改和扩展)
✅ **良好的文档** (详细的使用说明)

可以用于导航算法的开发、测试和验证，无需真实硬件环境！
