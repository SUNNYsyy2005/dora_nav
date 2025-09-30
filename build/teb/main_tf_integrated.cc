extern "C"
{
#include "node_api.h"
}

#include <thread>
#include <mutex>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include "tf_integration.h"
#include "../../include/ros.h"
#include "../../include/project_paths.h"

using namespace teb_local_planner;

// 全局变量
std::mutex scan_mutex, pose_mutex, tf_mutex;
geometry_msgs::Pose2D robot_pose;
sensor_msgs::LaserScan current_scan;
std::vector<std::pair<float, float>> pathh;
bool path_initialized = false;
std::unique_ptr<TEBTfInterface> tf_interface;
TebOptimalPlanner* planner = nullptr;
std::vector<ObstaclePtr> obst_vector;
TebConfig config;

const int step = 20;
const double PI = 3.1415926;

// TF回调函数
void tfCallback(const tf2_msgs::TFMessage* tf_msg) {
    if (!tf_msg || !tf_interface) {
        printf("TEB: TF数据无效\n");
        return;
    }
    
    std::lock_guard<std::mutex> lock(tf_mutex);
    
    printf("TEB: 接收到TF变换数据\n");
    
    // 更新TF接口
    tf_interface->updateTfCache(*tf_msg);
    
    printf("TEB: TF变换更新完成\n");
}

// 激光雷达回调函数（TF集成版）
void scanCallback(const sensor_msgs::LaserScan* scan_data) {
    if (!scan_data || !tf_interface || !planner) return;
    
    std::lock_guard<std::mutex> lock(scan_mutex);
    
    printf("TEB: 接收到激光扫描数据\n");
    
    // 保存当前扫描数据
    current_scan = *scan_data;
    
    try {
        // 使用TF接口变换激光数据到map坐标系
        std::vector<Eigen::Vector2d> obstacles_in_map = tf_interface->transformLaserScanToMap(*scan_data);
        
        // 清空障碍物列表
        obst_vector.clear();
        
        // 创建新的障碍物
        for (const auto& obstacle_point : obstacles_in_map) {
            PointObstacle* obstacle = new PointObstacle();
            obstacle->position() = obstacle_point;
            obst_vector.push_back(boost::shared_ptr<obstacles::Obstacle>(obstacle));
        }
        
        printf("TEB: 激光数据转换为%zu个障碍物点\n", obst_vector.size());
        
    } catch (const std::exception& e) {
        std::cerr << "TEB激光数据处理错误: " << e.what() << std::endl;
    }
}

// 位姿回调函数（TF集成版位姿融合）
void poseCallback(const geometry_msgs::Pose2D* fused_pose) {
    if (!fused_pose || !tf_interface) {
        printf("TEB: 位姿数据无效\n");
        return;
    }
    
    std::lock_guard<std::mutex> lock(pose_mutex);
    
    printf("TEB: 接收到融合位姿\n");
    
    // 更新机器人当前位姿
    robot_pose = *fused_pose;
}

// 初始化路径
void initializePath() {
    if (path_initialized) return;
    
    std::string line;
    std::ifstream file(ProjectPaths::build_teb_path_csv());
    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream linestream(line);
            std::string value;
            float x, y;
            std::getline(linestream, value, ',');
            x = std::stof(value);
            std::getline(linestream, value, ',');
            y = std::stof(value);
            pathh.push_back(std::make_pair(x, y));
        }
        file.close();
        std::reverse(pathh.begin(), pathh.end());
        path_initialized = true;
        printf("TEB: 成功加载%zu个路径点\n", pathh.size());
    } else {
        printf("TEB: 无法找到路径文件: %s\n", ProjectPaths::build_teb_path_csv().c_str());
        
        // 创建默认路径点
        pathh.push_back(std::make_pair(200, 250));
        path_initialized = true;
    }
}

// 定时任务：路径规划
void tickCallback(const char* id, size_t id_len, const char* data, size_t data_len) {
    if (!tf_interface || !planner || !path_initialized) return;
    
    std::lock_guard<std::mutex> pose_lock(pose_mutex);
    std::lock_guard<std::mutex> scan_lock(scan_mutex);
    std::lock_guard<std::mutex> tf_lock(tf_mutex);
    
    try {
        printf("TEB: 执行路径规划\n");
        
        // 获取机器人当前位置（从TF中获取或使用fused_pose）
        PoseSE2 start_pose = tf_interface->getRobotPoseInMap();
        
        // 如果TF中没有数据，使用fused_pose
        if (start_pose.x() == 0 && start_pose.y() == 0 && start_pose.theta() == 0) {
            start_pose = PoseSE2(robot_pose.x, robot_pose.y, robot_pose.theta);
        }
        
        // 目标点位姿（使用路径中的下一个点）
        static size_t reach_num = 0;
        if (reach_num >= pathh.size()) {
            reach_num = 0; // 循环导航
        }
        
        PoseSE2 goal_pose(pathh[reach_num].second * 0.04, // 像素转米
                         pathh[reach_num].first * 0.04,  // 像素转米
                         PI / 2); // 目标方向朝北
        
        printf("TEB: 起点 (%.3f, %.3f, %.3f) 目标 (%.3f, %.3f, %.3f)\n",
               start_pose.x(), start_pose.y(), start_pose.theta(),
               goal_pose.x(), goal_pose.y(), goal_pose.theta());
        
        // 执行TEB规划
        planner->plan(start_pose, goal_pose);
        
        // 获取速度指令
        float vx, vy, w;
        bool planning_result = planner->getVelocityCommand(vx, vy, w, step);
        
        if (planning_result) {
            // 创建速度指令
            geometry_msgs::Twist twist;
            twist.linear.x = vx;
            twist.linear.y = vy;
            twist.angular.z = w;
            
            printf("TEB: 生成速度指令 vx=%.3f vy=%.3f w=%.3f\n", vx, vy, w);
            
            // 发送速度指令
            std::string output_id = "twist";
            nlohmann::json json_obj = twist.to_json();
            std::string json_str = json_obj.dump();
            
            int result = dora_send_output(
                context_, 
                output_id.c_str(), 
                output_id.length(), 
                json_str.c_str(), 
                json_str.length()
            );
            
            if (result != 0) {
                std::cerr << "TEB: 发送速度指令失败" << std::endl;
            } else {
                printf("TEB: 速度指令发送成功\n");
                
                // 检查是否到达目标
                double distance_to_goal = sqrt(pow(start_pose.x() - goal_pose.x(), 2) + 
                                             pow(start_pose.y() - goal_pose.y(), 2));
                
                if (distance_to_goal < 0.5) { // 0.5米范围内认为到达
                    reach_num++;
                    printf("TEB: 到达目标点 %zu\n", reach_num);
                }
            }
        } else {
            printf("TEB: 路径规划失败\n");
            
            // 发送停止指令
            geometry_msgs::Twist stop_twist;
            std::string output_id = "twist";
            nlohmann::json json_obj = stop_twist.to_json();
            std::string json_str = json_obj.dump();
            
            dora_send_output(context_, output_id.c_str(), output_id.length(), 
                           json_str.c_str(), json_str.length());
        }
        
    } catch (const std::exception& e) {
        std::cerr << "TEB路径规划异常: " << e.what() << std::endl;
    }
}

// 初始化和运行函数
extern "C" void* init_teb_tf(void* context) {
    context_ = context;
    
    // 初始化TF接口
    tf_interface = std::make_unique<TEBTfInterface>();
    
    // 初始化路径
    initializePath();
    
    // 初始化TEB配置
    config.max_vel_x = 1.0;
    config.max_vel_theta = 1.0;
    config.acc_lim_x = 0.5;
    config.acc_lim_theta = 0.5;
    config.min_turning_radius = 0.1;
    
    // 创建机器人体型模型
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    
    // 创建TEB规划器
    planner = new TebOptimalPlanner(config, &obst_vector, robot_model, nullptr, nullptr);
    
    printf("TEB模块初始化完成（TF集成版）\n");
    
    return context;
}

extern "C" void run(void* context) {
    if (!context) {
        printf("TEB: 无效的上下文\n");
        return;
    }
    
    // 初始化
    init_teb_tf(context);
    
    printf("TEB: 开始运行（TF集成版）\n");
    
    // 主运行循环
    while (true) {
        void* event = dora_next_event(context);
        
        if (event == NULL) {
            printf("TEB: 等待事件\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        std::string event_type_name = dora_event_type_name(event);

        if (event_type_name == "message") {
            std::string input_id_name = dora_input_id(event);
            std::vector<char> data_vector = *dora_data_event_asBytes(event);
            const char* buffer_data = data_vector.data();
            
            printf("TEB输入: %s\n", input_id_name.c_str());
            
            try {
                nlohmann::json json_data = nlohmann::json::parse(std::string(buffer_data, data_vector.size()));
                
                if (input_id_name == "fused_pose") {
                    geometry_msgs::Pose2D pose_data = json_data;
                    poseCallback(&pose_data);
                } else if (input_id_name == "scan") {
                    sensor_msgs::LaserScan scan_data = json_data;
                    scanCallback(&scan_data);
                } else if (input_id_name == "tf_transforms") {
                    tf2_msgs::TFMessage tf_data = json_data;
                    tfCallback(&tf_data);
                } else if (input_id_name == "timer") {
                    tickCallback(input_id_name.c_str(), input_id_name.length(), 
                               buffer_data, data_vector.size());
                }
            } catch (const std::exception& e) {
                std::cerr << "TEB数据处理错误: " << e.what() << std::endl;
            }
        }
        
        dora_free_event(event);
    }
}

// 全局变量
void* context_ = nullptr;

