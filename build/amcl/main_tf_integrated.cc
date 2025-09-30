extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>
#include "../../include/ros.h"
#include "../../include/project_paths.h"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include "include/map/map.h"
#include "include/sensors/amcl_laser.h"
#include "include/sensors/amcl_odom.h"
#include "tf_integration.h"

// 全局变量
std::unique_ptr<AMCLTfInterface> tf_interface;
pf_t *pf;
map_t *map;
amcl::AMCLLaser laser_sensor(10, NULL);
amcl::AMCLOdom odom_sensor;
amcl::AMCLLaserData laser_data;
amcl::AMCLOdomData odom_data;

std::mutex slam_mutex, scan_mutex, imu_mutex;
pf_vector_t last_odom_pose = {0, 0, 0};

void replace_null_with_null(std::string& json_str) {
    std::string null_str = "null";
    std::string nan_str = "-1";
    size_t pos = 0;
    while ((pos = json_str.find(null_str, pos)) != std::string::npos) {
        json_str.replace(pos, null_str.length(), nan_str);
        pos += nan_str.length();
    }
}

// SLAM位姿回调（使用TF统一格式）
void slamPoseCallback(const geometry_msgs::Pose2D* slam_pose) {
    if (!slam_pose || !tf_interface || !pf) return;
    
    std::lock_guard<std::mutex> lock(slam_mutex);
    
    printf("AMCL: 接收到SLAM位姿\n");
    
    // 使用SLAM位姿重新初始化粒子滤波器
    pf_vector_t slam_vector;
    slam_vector.v[0] = slam_pose->x;  // x坐标 (米)
    slam_vector.v[1] = slam_pose->y;  // y坐标 (米) 
    slam_vector.v[2] = slam_pose->theta;  // theta角度 (弧度)
    
    // 初始化粒子滤波器到SLAM位置
    pf_vector_t mean = slam_vector;
    pf_matrix_t cov = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.01}; // 较小的初始协方差
    
    pf_init(pf, mean, cov);
    
    printf("AMCL: 基于SLAM位姿重新初始化，位置: (%.3f, %.3f, %.3f)\n", 
           slam_vector.v[0], slam_vector.v[1], slam_vector.v[2]);
}

// TF变换回调
void tfCallback(const tf2_msgs::TFMessage* tf_msg) {
    if (!tf_msg || !tf_interface) {
        printf("AMCL: TF数据无效\n");
        return;
    }
    
    printf("AMCL: 接收到TF变换数据\n");
    
    // 更新TF接口
    tf_interface->updateTfCache(*tf_msg);
    
    printf("AMCL: TF变换更新完成\n");
}

// 激光雷达回调
void scanCallback(const sensor_msgs::LaserScan* scan_data) {
    if (!scan_data || !tf_interface || !pf) return;
    
    std::lock_guard<std::mutex> lock(scan_mutex);
    
    printf("AMCL: 接收到激光扫描数据\n");
    
    // 更新里程计数据
    pf_vector_t current_odom_pose = tf_interface->getOdometryPose();
    
    // 计算里程计增量
    pf_vector_t odom_delta = tf_interface->calculateOdomDelta(last_odom_pose, current_odom_pose);
    
    // 设置里程计数据
    odom_data.pose = current_odom_pose;
    odom_data.delta = odom_delta;
    
    // 更新里程计传感器
    odom_sensor.UpdateAction(pf, &odom_data);
    
    // 转换激光雷达距离数据
    std::vector<double> ranges;
    for (const auto& range : scan_data->ranges) {
        if (!std::isnan(range) && range < 50.0 && range > 0.01) {
            ranges.push_back(range); // 保持米制单位
        } else {
            ranges.push_back(100.0); // 无效距离设为100米
        }
    }
    
    // 设置激光雷达数据
    laser_data.laser = ranges.data();
    laser_data.count = ranges.size();
    
    // 更新激光雷达传感器
    laser_sensor.UpdateSensor(pf, &laser_data);
    
    // 更新上一次里程计位姿
    last_odom_pose = current_odom_pose;
    
    printf("AMCL: 里程计增量 (%.6f, %.6f, %.6f)\n", 
           odom_delta.v[0], odom_delta.v[1], odom_delta.v[2]);
}

// IMU回调
void imuCallback(const sensor_msgs::Imu* imu_data) {
    if (!imu_data || !tf_interface) return;
    
    std::lock_guard<std::mutex> lock(imu_mutex);
    
    printf("AMCL: 接收到IMU数据\n");
    
    // IMU数据用于辅助里程计估计
    // 可以用于检测机器人是否静止等状态
    double linear_accel = sqrt(imu_data->linear_acceleration.x * imu_data->linear_acceleration.x +
                              imu_data->linear_acceleration.y * imu_data->linear_acceleration.y +
                              imu_data->linear_acceleration.z * imu_data->linear_acceleration.z);
    
    if (linear_accel < 0.1) {
        printf("AMCL: 检测到静止状态\n");
        // 可以将里程计噪声参数调整得更小
    }
}

// 定时任务：发布位姿估计
void tickCallback(const char* id, size_t id_len, const char* data, size_t data_len) {
    if (!tf_interface || !pf) return;
    
    std::lock_guard<std::mutex> slam_lock(slam_mutex);
    std::lock_guard<std::mutex> scan_lock(scan_mutex);
    
    printf("AMCL: 定时发布位姿\n");
    
    // 获取当前粒子滤波器的位姿估计
    pf_sample_set_t* set = pf->sets + pf->current_set;
    
    if (set->sample_count <= 0) {
        printf("AMCL: 没有有效的粒子样本\n");
        return;
    }
    
    // 计算加权平均位姿
    pf_vector_t mean_pose = pf_vector_zero();
    double weight_sum = 0.0;
    
    for (int i = 0; i < set->sample_count; i++) {
        pf_sample_t* sample = set->samples + i;
        mean_pose.v[0] += sample->pose.v[0] * sample->weight;
        mean_pose.v[1] += sample->pose.v[1] * sample->weight;
        mean_pose.v[2] += sample->pose.v[2] * sample->weight;
        weight_sum += sample->weight;
    }
    
    if (weight_sum > 0) {
        mean_pose.v[0] /= weight_sum;
        mean_pose.v[1] /= weight_sum;
        mean_pose.v[2] /= weight_sum;
    }
    
    // 标准化角度
    mean_pose.v[2] = AMCLTfInterface::normalizeAngle(mean_pose.v[2]);
    
    // 转换为输出格式
    geometry_msgs::Pose2D amcl_pose = tf_interface->slamPoseToAMCL(mean_pose);
    
    // 发布位姿估计
    std::string output_id = "pose";
    nlohmann::json json_data = amcl_pose.to_json();
    std::string json_str = json_data.dump();
    replace_null_with_null(json_str);
    
    int result = dora_send_output(
        context_, 
        output_id.c_str(), 
        output_id.length(), 
        json_str.c_str(), 
        json_str.length()
    );
    
    if (result != 0) {
        std::cerr << "AMCL: 发送位姿失败" << std::endl;
    }
    
    printf("AMCL位姿估计: x=%.3f, y=%.3f, theta=%.3f\n", 
           amcl_pose.x, amcl_pose.y, amcl_pose.theta);
}

// 初始化和运行函数
extern "C" void* init_amcl_tf(void* context) {
    context_ = context;
    
    // 初始化TF接口
    tf_interface = std::make_unique<AMCLTfInterface>();
    
    // 初始化地图
    map = map_alloc();
    map_load_occ(map, ProjectPaths::build_nav_laser_data().c_str(), 0.04, 1);
    
    // 设置AMCL激光雷达传感器模型
    laser_sensor = amcl::AMCLLaser(2000, map);
    
    pf_vector_t laser_pose = {0, 0, 0}; // 激光雷达在base_link中的位置
    laser_sensor.SetLaserPose(laser_pose);
    laser_sensor.SetModelLikelihoodField(0.99, 0.01, 0.1, 200);
    laser_data.sensor = &laser_sensor;
    
    // 创建粒子滤波器
    int min_samples = 500;
    int max_samples = 2000;
    double alpha_slow = 0.001;
    double alpha_fast = 0.1;
    
    pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                  random_pose_init, &laser_data);
    pf->selective_resampling = 1;
    
    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return nullptr;
    }
    
    // 初始位姿（等待SLAM提供）
    pf_vector_t initial_mean = {400 * 0.04, 400 * 0.04, M_PI/2}; // 假设从地图中心开始
    pf_matrix_t initial_cov = {1.0, 0, 0, 0, 1.0, 0, 0, 0, M_PI/2};
    pf_init(pf, initial_mean, initial_cov);
    
    printf("AMCL模块初始化完成（TF集成版）\n");
    
    return context;
}

extern "C" void run(void* context) {
    if (!context) {
        printf("AMCL: 无效的上下文\n");
        return;
    }
    
    // 初始化
    init_amcl_tf(context);
    
    printf("AMCL: 开始运行（TF集成版）\n");
    
    // 主运行循环
    while (true) {
        void* event = dora_next_event(context);
        
        if (event == NULL) {
            printf("AMCL: 等待事件\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        std::string event_type_name = dora_event_type_name(event);
        std::cout << "AMCL事件类型: " << event_type_name << std::endl;
        
        if (event_type_name == "message") {
            std::string input_id_name = dora_input_id(event);
            std::vector<char> data_vector = *dora_data_event_asBytes(event);
            const char* buffer_data = data_vector.data();
            
            printf("AMCL输入ID: %s\n", input_id_name.c_str());
            
            try {
                nlohmann::json json_data = nlohmann::json::parse(std::string(buffer_data, data_vector.size()));
                
                if (input_id_name == "slam_pose") {
                    geometry_msgs::Pose2D slam_pose = json_data;
                    slamPoseCallback(&slam_pose);
                } else if (input_id_name == "scan2") {
                    sensor_msgs::LaserScan scan_data = json_data;
                    scanCallback(&scan_data);
                } else if (input_id_name == "tf_transforms") {
                    tf2_msgs::TFMessage tf_data = json_data;
                    tfCallback(&tf_data);
                } else if (input_id_name == "imu") {
                    sensor_msgs::Imu imu_data = json_data;
                    imuCallback(&imu_data);
                } else if (input_id_name == "timer") {
                    tickCallback(input_id_name.c_str(), input_id_name.length(), 
                               buffer_data, data_vector.size());
                }
            } catch (const std::exception& e) {
                std::cerr << "AMCL数据处理错误: " << e.what() << std::endl;
            }
        }
        
        dora_free_event(event);
    }
}

// 全局变量
void* context_ = nullptr;
