# dora_nav
## 前置安装
### dora安装
```sh
git clone https://github.com/dora-rs/dora/
...
```
### imu节点
```sh
sudo chmod 777 /dev/ttyUSB0
```
### lidar节点
lidar为镭神16线激光雷达，官方驱动网址为https://github.com/Lslidar/Lslidar_ROS1_driver.git
设置有线连接为手动
地址为192.168.1.102
子网掩码为255.255.255.0
网关为1.1.1.1

#### 调试雷达？
##### 方案一
可以参考官网的ros驱动版本
##### 方案二


## slam建图
#### 运行
```sh
dora start slamflow.yml
```
#### 代码说明
slam建图代码主要在build/slam目录下

##### 雷达里程计日志生成 
main.cc文件

该文件主要是接收lidar节点的消息，结合imu对机器人位姿估计的消息(暂无)，输出laser_data.dat日志文件

##### 建图  
log2pgm.cc文件

该文件主要是根据laser_data.dat日志文件信息，采用slam方法建图

注：该代码改自https://github.com/simondlevy/BreezySLAM/

## 机器人导航
```sh
dora start dataflow.yml
```
### 定位
机器人定位的代码主要在amcl目录下，main.cc文件为起始文件

该部分代码根据lidar节点信息和imu节点提供的机器人朝向估计，采用amcl算法估计机器人在地图的实时坐标

注：该代码改自https://github.com/ysuga/navigation_amcl/tree/master


### 导航

#### 全局路径规划
机器人全局路径规划的代码主要在nav目录下，main.cc文件为起始文件

该部分代码对起始点和目标点之间路径采用A*算法进行全局规划，生成path.csv文件

注：该代码改自https://github.com/wql9/Navigation-planning-in-dynamic-and-static-environment/

#### 局部路径规划 main.cc文件
机器人局部路径规划的代码主要在teb目录下，main.cc文件为起始文件

该部分代码根据lidar节点提供障碍物信息和amcl节点提供的实时坐标，采用teb算法对全局路径中过程点之间路径进行实时规划

注：该代码改自https://github.com/wushichatong/teb_local_planner_no_ros