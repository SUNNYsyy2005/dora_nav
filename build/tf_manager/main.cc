extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include "../../include/ros.h"
#include "../../include/project_paths.h"
#include "../common/map_config.h"

// TF变换数据结构
struct TfTransform {
    double x, y, z;
    double qx, qy, qz, qw;
    double timestamp;
    bool valid;
    
    TfTransform() : x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(1), timestamp(0), valid(false) {}
    
    TfTransform(double x_, double y_, double z_, double qx_, double qy_, double qz_, double qw_)
        : x(x_), y(y_), z(z_), qx(qx_), qy(qy_), qz(qz_), qw(qw_), timestamp(0), valid(true) {}
};

// TF树管理器
class TfTreeManager {
private:
    std::map<std::string, std::map<std::string, TfTransform>> transforms_;
    std::mutex tf_mutex_;
    std::map<std::string, bool> static_transforms_;
    std::unique_ptr<MapConfig> map_config_;
    
public:
    TfTreeManager() {
        // 初始化地图配置
        map_config_ = std::make_unique<MapConfig>();
        
        // 初始化静态变换
        setupStaticTransforms();
    }
    
    // 设置静态变换关系
    void setupStaticTransforms() {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        // 尝试从YAML文件加载地图配置
        loadMapConfig();
        
        // base_link到laser的静态变换（假设激光雷达安装在机器人前方0.2米处）
        transforms_["base_link"]["laser"] = TfTransform(0.2, 0.0, 0.3, 0, 0, 0, 1);
        static_transforms_["base_link->laser"] = true;
        
        // base_link到odom的初始变换（起始位置）
        transforms_["odom"]["base_link"] = TfTransform(0, 0, 0, 0, 0, 0, 1);
        static_transforms_["odom->base_link"] = false; // 动态变换
        
        // map到odom的初始变换（从YAML配置读取原点信息）
        double origin_x = map_config_->getOriginX();
        double origin_y = map_config_->getOriginY();
        double origin_theta = map_config_->getOriginTheta();
        
        // 将原点角度转换为四元数
        double qz = sin(origin_theta / 2.0);
        double qw = cos(origin_theta / 2.0);
        
        transforms_["map"]["odom"] = TfTransform(origin_x, origin_y, 0.0, 0, 0, qz, qw);
        static_transforms_["map->odom"] = false; // 动态变换
        
        printf("TF Manager: 从YAML配置初始化map->odom变换\n");
        printf("  原点: (%.3f, %.3f, %.3f)\n", origin_x, origin_y, origin_theta);
        printf("  四元数: (0, 0, %.3f, %.3f)\n", qz, qw);
    }
    
    // 从YAML文件加载地图配置
    void loadMapConfig() {
        // 尝试多个可能的YAML文件路径
        std::vector<std::string> yaml_paths = {
            "build/nav/laser_data.yaml",
            "build/simulation_map_800x800.yaml",
            "build/simulation_map.yaml"
        };
        
        bool config_loaded = false;
        for (const auto& yaml_path : yaml_paths) {
            if (map_config_->loadFromFile(yaml_path)) {
                printf("TF Manager: 成功加载地图配置: %s\n", yaml_path.c_str());
                config_loaded = true;
                break;
            }
        }
        
        if (!config_loaded) {
            printf("TF Manager: 未找到YAML配置文件，使用默认配置\n");
            // 设置默认参数
            map_config_->setResolution(0.04);
            map_config_->setOrigin(0.0, 0.0, 0.0);
        }
    }
    
    // 更新动态变换
    bool updateTransform(const std::string& parent_frame, 
                        const std::string& child_frame,
                        double x, double y, double z,
                        double qx, double qy, double qz, double qw,
                        double timestamp) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        std::string tf_key = parent_frame + "->" + child_frame;
        
        // 如果是静态变换，不允许更新
        if (static_transforms_.find(tf_key) != static_transforms_.end() && 
            static_transforms_[tf_key]) {
            return false;
        }
        
        transforms_[parent_frame][child_frame] = TfTransform(x, y, z, qx, qy, qz, qw);
        transforms_[parent_frame][child_frame].timestamp = timestamp;
        
        return true;
    }
    
    // 从位姿更新base_link在odom坐标系中的位置
    bool updateBaseLinkPose(const geometry_msgs::Pose2D& pose, double timestamp) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        // 将Pose2D转换为TfTransform（四元数）
        double yaw = pose.theta;
        double qz = sin(yaw / 2.0);
        double qw = cos(yaw / 2.0);
        
        transforms_["odom"]["base_link"] = TfTransform(pose.x, pose.y, 0.0, 0, 0, qz, qw);
        transforms_["odom"]["base_link"].timestamp = timestamp;
        
        return true;
    }
    
    // 更新map到odom的变换（用于闭环修正）
    bool updateMapToOdom(const geometry_msgs::Pose2D& correction, double timestamp) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        double yaw = correction.theta;
        double qz = sin(yaw / 2.0);
        double qw = cos(yaw / 2.0);
        
        transforms_["map"]["odom"] = TfTransform(correction.x, correction.y, 0.0, 0, 0, qz, qw);
        transforms_["map"]["odom"].timestamp = timestamp;
        
        return true;
    }
    
    // 获取变换
    bool getTransform(const std::string& parent_frame, const std::string& child_frame,
                     TfTransform& transform) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        auto parent_it = transforms_.find(parent_frame);
        if (parent_it == transforms_.end()) {
            return false;
        }
        
        auto child_it = parent_it->second.find(child_frame);
        if (child_it == parent_it->second.end()) {
            return false;
        }
        
        transform = child_it->second;
        return transform.valid;
    }
    
    // 计算复合变换（从source_frame到target_frame）
    bool lookUpTransform(const std::string& source_frame, const std::string& target_frame,
                        TfTransform& transform) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        // 简化版：只处理直接变换和通过base_link的中转
        TfTransform transform1, transform2;
        
        if (source_frame == target_frame) {
            // 同一坐标系，单位变换
            transform = TfTransform(0, 0, 0, 0, 0, 0, 1);
            return true;
        }
        
        // 尝试直接变换
        if (getTransform(source_frame, target_frame, transform)) {
            return true;
        }
        
        // 尝试通过base_link中转
        if (source_frame != "base_link" && target_frame != "base_link") {
            if (getTransform(source_frame, "base_link", transform1) &&
                getTransform("base_link", target_frame, transform2)) {
                // 复合变换计算
                transform = multiplyTransforms(transform1, transform2);
                return true;
            }
        }
        
        return false;
    }
    
    // 获取所有当前变换信息（用于调试和监控）
    std::string getAllTransformsInfo() {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        
        std::stringstream ss;
        for (const auto& parent_pair : transforms_) {
            for (const auto& child_pair : parent_pair.second) {
                const TfTransform& tf = child_pair.second;
                ss << parent_pair.first << " -> " << child_pair.first 
                   << ": [" << tf.x << ", " << tf.y << ", " << tf.z << "] "
                   << "[" << tf.qx << ", " << tf.qy << ", " << tf.qz << ", " << tf.qw << "] "
                   << "t=" << tf.timestamp << std::endl;
            }
        }
        return ss.str();
    }
    
private:
    // 变换复合计算
    TfTransform multiplyTransforms(const TfTransform& t1, const TfTransform& t2) const {
        TfTransform result;
        
        // 平移部分
        result.x = t1.x + t2.x;
        result.y = t1.y + t2.y;
        result.z = t1.z + t2.z;
        
        // 四元数复合（简化为只处理绕Z轴旋转）
        double yaw1 = atan2(2.0 * (t1.qw * t1.qz + t1.qx * t1.qy), 
                            1.0 - 2.0 * (t1.qy * t1.qy + t1.qz * t1.qz));
        double yaw2 = atan2(2.0 * (t2.qw * t2.qz + t2.qx * t2.qy), 
                            1.0 - 2.0 * (t2.qy * t2.qy + t2.qz * t2.qz));
        
        double yaw_sum = yaw1 + yaw2;
        result.qz = sin(yaw_sum / 2.0);
        result.qw = cos(yaw_sum / 2.0);
        result.qx = 0;
        result.qy = 0;
        
        result.timestamp = std::max(t1.timestamp, t2.timestamp);
        result.valid = true;
        
        return result;
    }
};

// TF管理器
class TfManagerInterface {
private:
    std::unique_ptr<TfTreeManager> tf_manager_;
    
public:
    TfManagerInterface() : tf_manager_(std::make_unique<TfTreeManager>()) {}
    
    // 处理位姿更新
    void pose_callback(const geometry_msgs::Pose2D* fused_pose) {
        if (!fused_pose || !tf_manager_) return;
        
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        double timestamp = seconds.count() + std::chrono::duration<double>(now.time_since_epoch() - std::chrono::seconds(seconds.count())).count();
        
        tf_manager_->updateBaseLinkPose(*fused_pose, timestamp);
        
        // 发布变换信息
        publishTransformUpdates(timestamp, nullptr);
    }
    
    // 处理SLAM闭环修正
    void slam_correction_callback(const geometry_msgs::Pose2D* slam_correction) {
        if (!slam_correction || !tf_manager_) return;
        
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        double timestamp = seconds.count() + std::chrono::duration<double>(now.time_since_epoch() - std::chrono::seconds(seconds.count())).count();
        
        tf_manager_->updateMapToOdom(*slam_correction, timestamp);
    }
    
public:
    void publishTransformUpdates(double timestamp, void* context) {
        try {
            // 获取所有关键变换
            TfTransform odom_to_base, base_to_laser, map_to_odom;
            
            if (tf_manager_->getTransform("odom", "base_link", odom_to_base) &&
                tf_manager_->getTransform("base_link", "laser", base_to_laser) &&
                tf_manager_->getTransform("map", "odom", map_to_odom)) {
                
                // 创建简化的TF变换信息
                nlohmann::json tf_info;
                tf_info["timestamp"] = timestamp;
                tf_info["transforms"] = nlohmann::json::array();
                
                // odom -> base_link
                nlohmann::json transform1;
                transform1["parent_frame"] = "odom";
                transform1["child_frame"] = "base_link";
                transform1["translation"] = {{"x", odom_to_base.x}, {"y", odom_to_base.y}, {"z", odom_to_base.z}};
                transform1["rotation"] = {{"x", odom_to_base.qx}, {"y", odom_to_base.qy}, {"z", odom_to_base.qz}, {"w", odom_to_base.qw}};
                tf_info["transforms"].push_back(transform1);
                
                // base_link -> laser
                nlohmann::json transform2;
                transform2["parent_frame"] = "base_link";
                transform2["child_frame"] = "laser";
                transform2["translation"] = {{"x", base_to_laser.x}, {"y", base_to_laser.y}, {"z", base_to_laser.z}};
                transform2["rotation"] = {{"x", base_to_laser.qx}, {"y", base_to_laser.qy}, {"z", base_to_laser.qz}, {"w", base_to_laser.qw}};
                tf_info["transforms"].push_back(transform2);
                
                // map -> odom
                nlohmann::json transform3;
                transform3["parent_frame"] = "map";
                transform3["child_frame"] = "odom";
                transform3["translation"] = {{"x", map_to_odom.x}, {"y", map_to_odom.y}, {"z", map_to_odom.z}};
                transform3["rotation"] = {{"x", map_to_odom.qx}, {"y", map_to_odom.qy}, {"z", map_to_odom.qz}, {"w", map_to_odom.qw}};
                tf_info["transforms"].push_back(transform3);
                
                std::string output_id = "tf_transforms";
                std::string json_str = tf_info.dump();
                
                int result = dora_send_output(
                    context, 
                    const_cast<char*>(output_id.c_str()), 
                    output_id.length(), 
                    const_cast<char*>(json_str.c_str()), 
                    json_str.length()
                );
                
                if (result != 0) {
                    std::cerr << "failed to send tf transforms output" << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "TF update error: " << e.what() << std::endl;
        }
    }
};

// 全局实例
std::shared_ptr<TfManagerInterface> tf_manager;

// 全局变量
void* context_ = nullptr;
// 回调函数已集成到run函数中

extern "C" void* init_tf_manager(void* context) {
    context_ = context;
    tf_manager = std::make_shared<TfManagerInterface>();
    
    printf("TF Manager初始化完成\n");
    
    return context;
}

extern "C" void run(void* context) {
    // 初始化TF管理器
    if (!tf_manager) {
        init_tf_manager(context);
    }
    
    printf("TF Manager: 开始运行\n");
    
    // 主运行循环
    while (true) {
        void* event = dora_next_event(context);
        
        if (event == NULL) {
            printf("TF Manager: 等待事件\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        enum DoraEventType ty = read_dora_event_type(event);
        
        if (ty == DoraEventType_Input) {
            char* id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            
            char* data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            
            printf("TF Manager输入: %s\n", id.c_str());
            
            try {
                if (id == "timer" || id == "tick") {
                    // Timer事件通常没有数据内容，直接处理
                    if (tf_manager) {
                        auto current_time = std::chrono::system_clock::now().time_since_epoch().count() / 1000000.0;
                        tf_manager->publishTransformUpdates(current_time, context);
                    }
                } else if (data_len > 0) {
                    // 只有当数据长度大于0时才解析JSON
                    nlohmann::json json_data = nlohmann::json::parse(std::string(data_ptr, data_len));
                    
                    if (id == "fused_pose") {
                        geometry_msgs::Pose2D fused_pose;
                        fused_pose.x = json_data["x"];
                        fused_pose.y = json_data["y"];
                        fused_pose.theta = json_data["theta"];
                        
                        if (tf_manager) {
                            tf_manager->pose_callback(&fused_pose);
                        }
                    } else if (id == "slam_correction") {
                        geometry_msgs::Pose2D slam_correction;
                        slam_correction.x = json_data["x"];
                        slam_correction.y = json_data["y"];
                        slam_correction.theta = json_data["theta"];
                        
                        if (tf_manager) {
                            tf_manager->slam_correction_callback(&slam_correction);
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "TF Manager数据处理错误: " << e.what() << " (输入ID: " << id << ", 数据长度: " << data_len << ")" << std::endl;
            }
        }
        
        free_dora_event(event);
    }
}


int main() { auto dora_context = init_dora_context_from_env(); run(dora_context); free_dora_context(dora_context); return 0; }
