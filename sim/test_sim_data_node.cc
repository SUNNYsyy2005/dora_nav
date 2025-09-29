#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include "nlohmann/json.hpp"

// Dora相关头文件
extern "C" {
#include "../include/dora/node_api.h"
}

// ROS消息结构体定义（与include/ros.h一致）
struct Header {
    uint32_t seq;
    struct {
        int32_t sec;
        uint32_t nsec;
    } stamp;
    std::string frame_id;
};

struct Vector3 {
    double x, y, z;
};

struct Quaternion {
    double x, y, z, w;
};

// 激光雷达数据结构
struct LaserScan {
    Header header;
    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    std::vector<double> ranges;
    std::vector<double> intensities;
    
    // 从JSON解析
    static LaserScan from_json(const nlohmann::json& j) {
        LaserScan scan;
        scan.header.seq = j["header"]["seq"];
        scan.header.stamp.sec = j["header"]["stamp"]["sec"];
        scan.header.stamp.nsec = j["header"]["stamp"]["nsec"];
        scan.header.frame_id = j["header"]["frame_id"];
        scan.angle_min = j["angle_min"];
        scan.angle_max = j["angle_max"];
        scan.angle_increment = j["angle_increment"];
        scan.time_increment = j["time_increment"];
        scan.scan_time = j["scan_time"];
        scan.range_min = j["range_min"];
        scan.range_max = j["range_max"];
        
        // 解析ranges数组，处理可能的NaN值
        if (j["ranges"].is_array()) {
            for (const auto& val : j["ranges"]) {
                if (val.is_string() && val.get<std::string>() == "NaN") {
                    scan.ranges.push_back(NAN);
                } else {
                    scan.ranges.push_back(val.get<double>());
                }
            }
        }
        
        // 解析intensities数组
        if (j["intensities"].is_array()) {
            for (const auto& val : j["intensities"]) {
                scan.intensities.push_back(val.get<double>());
            }
        }
        
        return scan;
    }
    
    // 打印数据信息
    void print_info() const {
        std::cout << "=== 激光雷达数据 ===" << std::endl;
        std::cout << "序列号: " << header.seq << std::endl;
        std::cout << "时间戳: " << header.stamp.sec << "." << header.stamp.nsec << std::endl;
        std::cout << "坐标系: " << header.frame_id << std::endl;
        std::cout << "角度范围: " << angle_min << " 到 " << angle_max << " rad" << std::endl;
        std::cout << "角度增量: " << angle_increment << " rad" << std::endl;
        std::cout << "距离范围: " << range_min << " 到 " << range_max << " m" << std::endl;
        std::cout << "射线数量: " << ranges.size() << std::endl;
        
        // 统计有效距离
        int valid_ranges = 0;
        double min_range = INFINITY, max_range = -INFINITY;
        double sum_range = 0.0;
        
        for (double range : ranges) {
            if (!std::isnan(range) && range >= range_min && range <= range_max) {
                valid_ranges++;
                min_range = std::min(min_range, range);
                max_range = std::max(max_range, range);
                sum_range += range;
            }
        }
        
        std::cout << "有效射线: " << valid_ranges << std::endl;
        if (valid_ranges > 0) {
            std::cout << "距离统计: 最小=" << min_range << "m, 最大=" << max_range << "m, 平均=" << (sum_range/valid_ranges) << "m" << std::endl;
        }
        
        // 显示前10个距离值
        std::cout << "前10个距离值: ";
        for (int i = 0; i < std::min(10, (int)ranges.size()); ++i) {
            if (std::isnan(ranges[i])) {
                std::cout << "NaN ";
            } else {
                std::cout << ranges[i] << " ";
            }
        }
        std::cout << std::endl;
    }
};

// IMU数据结构
struct Imu {
    Header header;
    Quaternion orientation;
    std::array<double, 9> orientation_covariance;
    Vector3 angular_velocity;
    std::array<double, 9> angular_velocity_covariance;
    Vector3 linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance;
    
    // 从JSON解析
    static Imu from_json(const nlohmann::json& j) {
        Imu imu;
        imu.header.seq = j["header"]["seq"];
        imu.header.stamp.sec = j["header"]["stamp"]["sec"];
        imu.header.stamp.nsec = j["header"]["stamp"]["nsec"];
        imu.header.frame_id = j["header"]["frame_id"];
        
        imu.orientation.x = j["orientation"]["x"];
        imu.orientation.y = j["orientation"]["y"];
        imu.orientation.z = j["orientation"]["z"];
        imu.orientation.w = j["orientation"]["w"];
        
        // 解析协方差矩阵
        auto cov_vec = j["orientation_covariance"].get<std::vector<double>>();
        std::copy(cov_vec.begin(), cov_vec.end(), imu.orientation_covariance.begin());
        
        imu.angular_velocity.x = j["angular_velocity"]["x"];
        imu.angular_velocity.y = j["angular_velocity"]["y"];
        imu.angular_velocity.z = j["angular_velocity"]["z"];
        
        cov_vec = j["angular_velocity_covariance"].get<std::vector<double>>();
        std::copy(cov_vec.begin(), cov_vec.end(), imu.angular_velocity_covariance.begin());
        
        imu.linear_acceleration.x = j["linear_acceleration"]["x"];
        imu.linear_acceleration.y = j["linear_acceleration"]["y"];
        imu.linear_acceleration.z = j["linear_acceleration"]["z"];
        
        cov_vec = j["linear_acceleration_covariance"].get<std::vector<double>>();
        std::copy(cov_vec.begin(), cov_vec.end(), imu.linear_acceleration_covariance.begin());
        
        return imu;
    }
    
    // 从四元数计算偏航角
    double get_yaw() const {
        return std::atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    }
    
    // 打印数据信息
    void print_info() const {
        std::cout << "=== IMU数据 ===" << std::endl;
        std::cout << "序列号: " << header.seq << std::endl;
        std::cout << "时间戳: " << header.stamp.sec << "." << header.stamp.nsec << std::endl;
        std::cout << "坐标系: " << header.frame_id << std::endl;
        std::cout << "四元数: w=" << orientation.w << ", x=" << orientation.x 
                  << ", y=" << orientation.y << ", z=" << orientation.z << std::endl;
        std::cout << "偏航角: " << (get_yaw() * 180.0 / M_PI) << "°" << std::endl;
        std::cout << "角速度: x=" << angular_velocity.x << ", y=" << angular_velocity.y 
                  << ", z=" << angular_velocity.z << " rad/s" << std::endl;
        std::cout << "线性加速度: x=" << linear_acceleration.x << ", y=" << linear_acceleration.y 
                  << ", z=" << linear_acceleration.z << " m/s²" << std::endl;
        
        // 验证重力加速度
        double gravity_magnitude = sqrt(linear_acceleration.x * linear_acceleration.x +
                                      linear_acceleration.y * linear_acceleration.y +
                                      linear_acceleration.z * linear_acceleration.z);
        std::cout << "重力加速度大小: " << gravity_magnitude << " m/s²" << std::endl;
    }
};

// 数据统计器
class DataStatistics {
private:
    int lidar_count = 0;
    int imu_count = 0;
    double total_lidar_time = 0.0;
    double total_imu_time = 0.0;
    double last_lidar_time = 0.0;
    double last_imu_time = 0.0;
    
public:
    void update_lidar_stats(double timestamp) {
        if (last_lidar_time > 0) {
            total_lidar_time += (timestamp - last_lidar_time);
        }
        last_lidar_time = timestamp;
        lidar_count++;
    }
    
    void update_imu_stats(double timestamp) {
        if (last_imu_time > 0) {
            total_imu_time += (timestamp - last_imu_time);
        }
        last_imu_time = timestamp;
        imu_count++;
    }
    
    void print_statistics() {
        std::cout << "\n=== 数据统计 ===" << std::endl;
        std::cout << "激光雷达数据包数: " << lidar_count << std::endl;
        std::cout << "IMU数据包数: " << imu_count << std::endl;
        
        if (lidar_count > 1) {
            double avg_lidar_interval = total_lidar_time / (lidar_count - 1);
            std::cout << "激光雷达平均间隔: " << avg_lidar_interval << "s (期望: 0.1s)" << std::endl;
        }
        
        if (imu_count > 1) {
            double avg_imu_interval = total_imu_time / (imu_count - 1);
            std::cout << "IMU平均间隔: " << avg_imu_interval << "s (期望: 0.05s)" << std::endl;
        }
    }
};

// 全局统计器
DataStatistics g_stats;

// 处理激光雷达数据
void process_lidar_data(const std::vector<unsigned char>& data) {
    try {
        // 解析JSON
        std::string json_str(data.begin(), data.end());
        nlohmann::json json_obj = nlohmann::json::parse(json_str);
        
        // 转换为LaserScan结构
        LaserScan scan = LaserScan::from_json(json_obj);
        
        // 更新时间戳统计
        double timestamp = scan.header.stamp.sec + scan.header.stamp.nsec * 1e-9;
        g_stats.update_lidar_stats(timestamp);
        
        // 打印数据信息
        scan.print_info();
        
        // 数据验证
        bool valid = true;
        if (scan.ranges.size() != scan.intensities.size()) {
            std::cout << "警告: ranges和intensities数组大小不匹配!" << std::endl;
            valid = false;
        }
        
        if (scan.angle_min >= scan.angle_max) {
            std::cout << "警告: 角度范围无效!" << std::endl;
            valid = false;
        }
        
        if (scan.range_min >= scan.range_max) {
            std::cout << "警告: 距离范围无效!" << std::endl;
            valid = false;
        }
        
        if (valid) {
            std::cout << "✓ 激光雷达数据验证通过" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "激光雷达数据解析错误: " << e.what() << std::endl;
    }
}

// 处理IMU数据
void process_imu_data(const std::vector<unsigned char>& data) {
    try {
        // 解析JSON
        std::string json_str(data.begin(), data.end());
        nlohmann::json json_obj = nlohmann::json::parse(json_str);
        
        // 转换为Imu结构
        Imu imu = Imu::from_json(json_obj);
        
        // 更新时间戳统计
        double timestamp = imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9;
        g_stats.update_imu_stats(timestamp);
        
        // 打印数据信息
        imu.print_info();
        
        // 数据验证
        bool valid = true;
        
        // 检查四元数归一化
        double quat_norm = sqrt(imu.orientation.w * imu.orientation.w +
                               imu.orientation.x * imu.orientation.x +
                               imu.orientation.y * imu.orientation.y +
                               imu.orientation.z * imu.orientation.z);
        if (std::abs(quat_norm - 1.0) > 0.01) {
            std::cout << "警告: 四元数未归一化! 大小=" << quat_norm << std::endl;
            valid = false;
        }
        
        // 检查重力加速度范围
        if (imu.linear_acceleration.z < 8.0 || imu.linear_acceleration.z > 11.0) {
            std::cout << "警告: 重力加速度异常! z=" << imu.linear_acceleration.z << std::endl;
            valid = false;
        }
        
        if (valid) {
            std::cout << "✓ IMU数据验证通过" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "IMU数据解析错误: " << e.what() << std::endl;
    }
}

// 主程序入口
int main(int argc, char* argv[]) {
    std::cout << "=== Dora仿真数据测试节点 ===" << std::endl;
    std::cout << "用于测试Python仿真数据的接收和处理" << std::endl;
    std::cout << "支持的数据类型: 激光雷达(scan), IMU(data)" << std::endl;
    std::cout << "按Ctrl+C停止测试" << std::endl << std::endl;
    
    // 初始化Dora上下文
    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "初始化Dora上下文失败" << std::endl;
        return 1;
    }
    
    // 主循环
    int data_count = 0;
    try {
        while (true) {
            // 等待下一个事件
            void* event = dora_next_event(dora_context);
            if (!event) {
                std::cerr << "获取事件失败" << std::endl;
                break;
            }
            
            // 获取事件类型
            DoraEventType event_type = read_dora_event_type(event);
            
            if (event_type == DoraEventType_Stop) {
                std::cout << "收到停止信号" << std::endl;
                free_dora_event(event);
                break;
                
            } else if (event_type == DoraEventType_Error) {
                std::cerr << "发生错误事件" << std::endl;
                free_dora_event(event);
                break;
                
            } else if (event_type == DoraEventType_Input) {
                // 获取输入ID
                char* input_id_ptr;
                size_t input_id_len;
                read_dora_input_id(event, &input_id_ptr, &input_id_len);
                std::string input_id(input_id_ptr, input_id_len);
                
                if (input_id == "scan") {
                    std::cout << "\n--- 接收激光雷达数据 ---" << std::endl;
                    
                    // 读取数据
                    char* data_ptr;
                    size_t data_len;
                    read_dora_input_data(event, &data_ptr, &data_len);
                    
                    // 转换为vector
                    std::vector<unsigned char> data(data_ptr, data_ptr + data_len);
                    
                    // 处理数据
                    process_lidar_data(data);
                    data_count++;
                    
                } else if (input_id == "data") {
                    std::cout << "\n--- 接收IMU数据 ---" << std::endl;
                    
                    // 读取数据
                    char* data_ptr;
                    size_t data_len;
                    read_dora_input_data(event, &data_ptr, &data_len);
                    
                    // 转换为vector
                    std::vector<unsigned char> data(data_ptr, data_ptr + data_len);
                    
                    // 处理数据
                    process_imu_data(data);
                    data_count++;
                    
                } else if (input_id == "tick") {
                    // 定时器事件，可以用于定期输出统计信息
                    if (data_count % 10 == 0 && data_count > 0) {
                        std::cout << "\n--- 定时统计 ---" << std::endl;
                        g_stats.print_statistics();
                    }
                } else {
                    std::cout << "收到未知输入: " << input_id << std::endl;
                }
            }
            
            // 清理事件
            free_dora_event(event);
            
            // 限制输出频率，避免刷屏
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
    } catch (const std::exception& e) {
        std::cout << "程序异常: " << e.what() << std::endl;
    }
    
    // 输出最终统计
    std::cout << "\n=== 最终统计 ===" << std::endl;
    g_stats.print_statistics();
    
    // 清理上下文
    free_dora_context(dora_context);
    
    return 0;
}
