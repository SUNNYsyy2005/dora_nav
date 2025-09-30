extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cmath>
#include <unistd.h>
#include "A_star_dwa.cc"
#include "tf_integration.h"
#include "../common/map_config.h"
#include "../../include/ros.h"
#include "../../include/project_paths.h"
#include <memory>
#include <chrono>
#include <thread>

// 全局变量
std::unique_ptr<NavTfInterface> tf_interface;
std::unique_ptr<MapConfig> map_config;
Astar_DWA* planner = nullptr;

// 路径规划参数
int start_x = 400, start_y = 400;
double start_angle = M_PI / 2;
int goal_x = 200, goal_y = 250;

double getAngle(int x1, int y1, int x2, int y2) {
    double angle = atan2(y1 - y2, x1 - x2);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

// TF变换回调函数
void tfCallback(const tf2_msgs::TFMessage* tf_msg) {
    if (!tf_msg || !tf_interface) {
        printf("NAV: TF数据无效\n");
        return;
    }
    
    printf("NAV: 接收到TF变换数据\n");
    
    // 更新TF接口
    tf_interface->updateTfCache(*tf_msg);
    
    printf("NAV: TF变换更新完成\n");
}

// 融合位姿回调函数
void fusedPoseCallback(const geometry_msgs::Pose2D* fused_pose) {
    if (!fused_pose || !tf_interface || !map_config) {
        printf("NAV: 位姿数据无效\n");
        return;
    }
    
    printf("NAV: 接收到融合位姿\n");
    
    // 将融合位姿转换为像素坐标
    auto pixel_coords = tf_interface->fusePoseToPixels(*fused_pose);
    int x_pixel = std::get<0>(pixel_coords);
    int y_pixel = std::get<1>(pixel_coords);
    double theta = std::get<2>(pixel_coords);
    
    // 更新起始位置
    start_x = x_pixel;
    start_y = y_pixel;
    start_angle = theta;
    
    printf("NAV: 更新起始位置到像素坐标 (%d, %d, %.3f)\n", start_x, start_y, start_angle);
}

// 激光雷达回调函数
void scanCallback(const sensor_msgs::LaserScan* scan_data) {
    if (!scan_data || !tf_interface) {
        printf("NAV: 激光数据无效\n");
        return;
    }
    
    printf("NAV: 接收到激光扫描数据\n");
    
    // 使用TF接口获取障碍物像素坐标
    auto obstacle_pixels = tf_interface->getObstaclePixelsFromScan(*scan_data);
    
    printf("NAV: 检测到 %zu 个障碍物点\n", obstacle_pixels.size());
    
    // 这里可以更新路径规划器的障碍物信息
    // 由于A_star_dwa.cc使用PGM文件，我们主要依赖地图文件
}

// 定时任务：执行路径规划
void tickCallback(const char* id, size_t id_len, const char* data, size_t data_len) {
    if (!planner || !map_config) {
        printf("NAV: 路径规划器未初始化\n");
        return;
    }
    
    printf("NAV: 执行路径规划\n");
    printf("起始点: (%d, %d, %.3f)\n", start_x, start_y, start_angle);
    printf("目标点: (%d, %d)\n", goal_x, goal_y);
    
    try {
        // 执行A*路径规划
        auto path_result = planner->plan(start_x, start_y, start_angle, goal_x, goal_y, 0.0);
        auto path_x = std::get<0>(path_result);
        auto path_y = std::get<1>(path_result);
        int flag = std::get<2>(path_result);
        
        if (flag == 1) {
            printf("NAV: 路径规划成功，找到 %zu 个路径点\n", path_x.size());
            
            // 创建路径规划结果
            nlohmann::json path_result;
            path_result["success"] = true;
            path_result["path_x"] = path_x;
            path_result["path_y"] = path_y;
            path_result["start"] = {{"x", start_x}, {"y", start_y}, {"theta", start_angle}};
            path_result["goal"] = {{"x", goal_x}, {"y", goal_y}};
            path_result["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
            
            // 发布路径规划结果
            std::string output_id = "path_plan";
            std::string json_str = path_result.dump();
            
            int result = dora_send_output(
                context_, 
                output_id.c_str(), 
                output_id.length(), 
                json_str.c_str(), 
                json_str.length()
            );
            
            if (result != 0) {
                std::cerr << "NAV: 发送路径规划结果失败" << std::endl;
            } else {
                printf("NAV: 路径规划结果发送成功\n");
            }
            
        } else {
            printf("NAV: 路径规划失败\n");
            
            // 发送失败结果
            nlohmann::json path_result;
            path_result["success"] = false;
            path_result["error"] = "No path found";
            path_result["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
            
            std::string output_id = "path_plan";
            std::string json_str = path_result.dump();
            
            dora_send_output(context_, output_id.c_str(), output_id.length(), 
                           json_str.c_str(), json_str.length());
        }
        
    } catch (const std::exception& e) {
        std::cerr << "NAV路径规划异常: " << e.what() << std::endl;
    }
}

// 初始化和运行函数
extern "C" void* init_nav_tf(void* context) {
    context_ = context;
    
    // 初始化TF接口
    tf_interface = std::make_unique<NavTfInterface>();
    
    // 初始化地图配置
    map_config = std::make_unique<MapConfig>();
    
    // 尝试从YAML文件加载地图配置
    std::string yaml_path = "build/simulation_map_800x800.yaml";  // 使用生成的地图配置
    
    if (map_config->loadFromFile(yaml_path)) {
        printf("NAV: 成功加载地图配置: %s\n", yaml_path.c_str());
    } else {
        printf("NAV: 使用默认地图配置\n");
        // 设置默认参数
        map_config->setResolution(0.04);
        map_config->setOrigin(0.0, 0.0, 0.0);
    }
    
    // 初始化路径规划器
    planner = new Astar_DWA();
    
    printf("NAV模块初始化完成（TF集成版）\n");
    printf("地图分辨率: %.4f m/pixel\n", map_config->getResolution());
    
    return context;
}

extern "C" void run(void* context) {
    if (!context) {
        printf("NAV: 无效的上下文\n");
        return;
    }
    
    // 初始化
    init_nav_tf(context);
    
    printf("NAV: 开始运行（TF集成版）\n");
    
    // 主运行循环
    while (true) {
        void* event = dora_next_event(context);
        
        if (event == NULL) {
            printf("NAV: 等待事件\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        std::string event_type_name = dora_event_type_name(event);
        
        if (event_type_name == "message") {
            std::string input_id_name = dora_input_id(event);
            std::vector<char> data_vector = *dora_data_event_asBytes(event);
            const char* buffer_data = data_vector.data();
            
            printf("NAV输入: %s\n", input_id_name.c_str());
            
            try {
                nlohmann::json json_data = nlohmann::json::parse(std::string(buffer_data, data_vector.size()));
                
                if (input_id_name == "fused_pose") {
                    geometry_msgs::Pose2D pose_data = json_data;
                    fusedPoseCallback(&pose_data);
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
                std::cerr << "NAV数据处理错误: " << e.what() << std::endl;
            }
        }
        
        dora_free_event(event);
    }
}

// 全局变量
void* context_ = nullptr;
