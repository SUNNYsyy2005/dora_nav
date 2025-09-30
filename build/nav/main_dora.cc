extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include "A_star_dwa.cc"
#include "../../include/ros.h"
#include "../../include/project_paths.h"

// 全局变量
Astar_DWA* planner = nullptr;
int start_x = 400, start_y = 400;
double start_angle = M_PI / 2;
int goal_x = 200, goal_y = 250;
bool has_goal = false;
void* dora_context = nullptr;

double getAngle(int x1, int y1, int x2, int y2) {
    double angle = atan2(y1 - y2, x1 - x2);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

// 世界坐标到像素坐标转换
std::pair<int, int> world_to_pixel(double x, double y, double resolution, int map_width, int map_height) {
    int px = static_cast<int>(x / resolution + map_width / 2);
    int py = static_cast<int>(y / resolution + map_height / 2);
    return {px, py};
}

// 像素坐标到世界坐标转换
std::pair<double, double> pixel_to_world(int px, int py, double resolution, int map_width, int map_height) {
    double x = (px - map_width / 2) * resolution;
    double y = (py - map_height / 2) * resolution;
    return {x, y};
}

int main() {
    printf("NAV: 初始化导航节点\n");
    
    // 初始化Dora上下文
    dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "NAV: 初始化Dora上下文失败" << std::endl;
        return 1;
    }
    
    // 从YAML加载地图参数
    double resolution = 0.005;
    int map_width = 800;
    int map_height = 800;
    
    std::string yaml_path = ProjectPaths::nav_data_yaml("laser_data");
    std::ifstream yaml_file(yaml_path);
    if (yaml_file.is_open()) {
        std::string line;
        while (std::getline(yaml_file, line)) {
            if (line.find("resolution:") != std::string::npos) {
                sscanf(line.c_str(), "resolution: %lf", &resolution);
            }
        }
        yaml_file.close();
    }
    
    printf("NAV: 地图参数 - 分辨率: %.4f, 尺寸: %dx%d\n", resolution, map_width, map_height);
    
    // 初始化路径规划器 (使用默认地图，A_star_dwa内部会加载laser_data.pgm)
    planner = new Astar_DWA();
    if (!planner) {
        std::cerr << "NAV: 初始化路径规划器失败" << std::endl;
        free_dora_context(dora_context);
        return 1;
    }
    
    printf("NAV: 路径规划器初始化成功\n");
    printf("NAV: 开始主循环\n");
    
    // 主循环
    while (true) {
        void* event = dora_next_event(dora_context);
        
        if (event == NULL) {
            usleep(10000);
            continue;
        }
        
        enum DoraEventType ty = read_dora_event_type(event);
        
        if (ty == DoraEventType_Stop) {
            printf("NAV: 接收到停止信号\n");
            free_dora_event(event);
            break;
        }
        
        if (ty == DoraEventType_Input) {
            char* id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            
            char* data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            
            try {
                if (id == "pose" && data_len > 0) {
                    // 接收机器人位姿
                    std::string json_str(data_ptr, data_len);
                    nlohmann::json json_data = nlohmann::json::parse(json_str);
                    
                    double x = json_data["x"];
                    double y = json_data["y"];
                    double theta = json_data["theta"];
                    
                    // 转换为像素坐标
                    auto pixel_coords = world_to_pixel(x, y, resolution, map_width, map_height);
                    start_x = pixel_coords.first;
                    start_y = pixel_coords.second;
                    start_angle = theta;
                    
                    printf("NAV: 更新机器人位姿 - 世界坐标(%.3f, %.3f, %.3f) -> 像素坐标(%d, %d)\n", 
                           x, y, theta, start_x, start_y);
                }
                else if (id == "goal" && data_len > 0) {
                    // 接收目标点
                    std::string json_str(data_ptr, data_len);
                    nlohmann::json json_data = nlohmann::json::parse(json_str);
                    
                    // 支持两种格式：简单格式 {"x": x, "y": y} 或复杂格式 {"target_pose": {"x": x, "y": y}}
                    double goal_world_x, goal_world_y;
                    if (json_data.contains("target_pose")) {
                        goal_world_x = json_data["target_pose"]["x"];
                        goal_world_y = json_data["target_pose"]["y"];
                    } else {
                        goal_world_x = json_data["x"];
                        goal_world_y = json_data["y"];
                    }
                    
                    // 转换为像素坐标
                    auto pixel_coords = world_to_pixel(goal_world_x, goal_world_y, resolution, map_width, map_height);
                    goal_x = pixel_coords.first;
                    goal_y = pixel_coords.second;
                    has_goal = true;
                    
                    printf("NAV: 接收到目标点 - 世界坐标(%.3f, %.3f) -> 像素坐标(%d, %d)\n", 
                           goal_world_x, goal_world_y, goal_x, goal_y);
                }
                else if (id == "tick") {
                    // 定时规划路径
                    if (has_goal && planner) {
                        printf("NAV: 执行路径规划 从(%d,%d) 到(%d,%d)\n", start_x, start_y, goal_x, goal_y);
                        
                        // 计算目标角度
                        double goal_angle = getAngle(start_x, start_y, goal_x, goal_y);
                        
                        // 执行A*规划
                        auto [path_x, path_y, flag] = planner->plan(start_x, start_y, start_angle, goal_x, goal_y, goal_angle);
                        
                        if (flag != 1) {
                            printf("NAV: 路径规划失败\n");
                            free_dora_event(event);
                            continue;
                        }
                        
                        // 构建路径消息
                        if (path_x.size() > 0) {
                            nlohmann::json path_json;
                            path_json["header"]["seq"] = 0;
                            path_json["header"]["stamp"]["sec"] = 0;
                            path_json["header"]["stamp"]["nsec"] = 0;
                            path_json["header"]["frame_id"] = "map";
                            
                            std::vector<nlohmann::json> poses;
                            
                            for (size_t i = 0; i < path_x.size(); i++) {
                                int px = path_x[i];
                                int py = path_y[i];
                                
                                // 转换为世界坐标
                                auto world_coords = pixel_to_world(px, py, resolution, map_width, map_height);
                                
                                // 调试输出前几个点
                                if (i < 5 || i >= path_x.size() - 2) {
                                    printf("NAV: 路径点[%zu]: 像素(%d,%d) -> 世界(%.3f,%.3f)\n", 
                                           i, px, py, world_coords.first, world_coords.second);
                                }
                                
                                // 计算角度
                                double angle = (i + 1 < path_x.size()) ? 
                                    getAngle(px, py, path_x[i+1], path_y[i+1]) : goal_angle;
                                
                                nlohmann::json pose;
                                pose["pose"]["position"]["x"] = world_coords.first;
                                pose["pose"]["position"]["y"] = world_coords.second;
                                pose["pose"]["position"]["z"] = 0.0;
                                pose["pose"]["orientation"]["x"] = 0.0;
                                pose["pose"]["orientation"]["y"] = 0.0;
                                pose["pose"]["orientation"]["z"] = sin(angle / 2.0);
                                pose["pose"]["orientation"]["w"] = cos(angle / 2.0);
                                
                                poses.push_back(pose);
                            }
                            
                            path_json["poses"] = poses;
                            
                            // 发送全局路径
                            std::string path_json_str = path_json.dump();
                            std::string output_id = "global_path";
                            dora_send_output(dora_context, 
                                           const_cast<char*>(output_id.c_str()), output_id.length(),
                                           const_cast<char*>(path_json_str.c_str()), path_json_str.length());
                            
                            printf("NAV: 发布全局路径 - %zu 个路径点\n", path_x.size());
                            
                            // 发送规划状态
                            nlohmann::json status_json;
                            status_json["status"] = "SUCCESS";
                            status_json["num_points"] = (int)path_x.size();
                            std::string status_str = status_json.dump();
                            std::string status_id = "planning_status";
                            dora_send_output(dora_context,
                                           const_cast<char*>(status_id.c_str()), status_id.length(),
                                           const_cast<char*>(status_str.c_str()), status_str.length());
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "NAV: 处理输入错误 (" << id << "): " << e.what() << std::endl;
            }
        }
        
        free_dora_event(event);
    }
    
    // 清理
    delete planner;
    free_dora_context(dora_context);
    
    printf("NAV: 节点已退出\n");
    return 0;
}
