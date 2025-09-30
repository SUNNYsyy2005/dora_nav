#ifndef NAV_TF_INTEGRATION_H
#define NAV_TF_INTEGRATION_H

#include "../../include/ros.h"
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>

// NAV-TF集成接口
class NavTfInterface {
private:
    std::map<std::string, std::map<std::string, geometry_msgs::TransformStamped>> tf_cache_;
    
    // 地图参数 - 与SLAM生成的地图保持一致
    static constexpr int MAP_WIDTH_PIXELS = 800;
    static constexpr int MAP_HEIGHT_PIXELS = 800;
    static constexpr double MAP_RESOLUTION = 0.005; // 每像素0.005米 (5mm/pixel)
    
public:
    // 更新TF变换缓存
    void updateTfCache(const tf2_msgs::TFMessage& tf_msg) {
        for (const auto& transform_stamped : tf_msg.transforms) {
            std::string parent = transform_stamped.header.frame_id;
            std::string child = transform_stamped.child_frame_id;
            tf_cache_[parent][child] = transform_stamped;
        }
    }
    
    // 将地图坐标(米)转换为像素坐标
    std::pair<int, int> mapToPixels(double x_m, double y_m) const {
        int x_pixel = static_cast<int>(x_m / MAP_RESOLUTION) + MAP_WIDTH_PIXELS / 2;
        int y_pixel = static_cast<int>(y_m / MAP_RESOLUTION) + MAP_HEIGHT_PIXELS / 2;
        
        // 边界检查
        x_pixel = std::max(0, std::min(x_pixel, MAP_WIDTH_PIXELS - 1));
        y_pixel = std::max(0, std::min(y_pixel, MAP_HEIGHT_PIXELS - 1));
        
        return std::make_pair(x_pixel, y_pixel);
    }
    
    // 将像素坐标转换为地图坐标(米)
    std::pair<double, double> pixelsToMap(int x_pixel, int y_pixel) const {
        double x_m = (x_pixel - MAP_WIDTH_PIXELS / 2) * MAP_RESOLUTION;
        double y_m = (y_pixel - MAP_HEIGHT_PIXELS / 2) * MAP_RESOLUTION;
        
        return std::make_pair(x_m, y_m);
    }
    
    // 获取机器人在map坐标系中的位置
    std::pair<double, double> getRobotPositionInMap() const {
        double x = 0.0, y = 0.0;
        
        auto map_it = tf_cache_.find("map");
        if (map_it != tf_cache_.end()) {
            auto base_it = map_it->second.find("base_link");
            if (base_it != map_it->second.end()) {
                const auto& tf_msg = base_it->second;
                x = tf_msg.transform.translation.x;
                y = tf_msg.transform.translation.y;
            }
        }
        
        return std::make_pair(x, y);
    }
    
    // 转换融合位姿到NAV使用的像素坐标
    std::tuple<int, int, double> fusePoseToPixels(const geometry_msgs::Pose2D& fused_pose) const {
        auto pixel_coords = mapToPixels(fused_pose.x, fused_pose.y);
        return std::make_tuple(pixel_coords.first, pixel_coords.second, fused_pose.theta);
    }
    
    // 转换目标位置到地图坐标系
    geometry_msgs::Pose2D transformGoalToMap(const geometry_msgs::Pose2D& goal_pose) const {
        // 假设目标位姿已经在map坐标系中
        // 如果需要变换可以在这里添加
        return goal_pose;
    }
    
    // 获取激光雷达在地图中的障碍物坐标
    std::vector<std::pair<int, int>> getObstaclePixelsFromScan(
        const sensor_msgs::LaserScan& scan_msg) const {
        
        std::vector<std::pair<int, int>> obstacle_pixels;
        
        // 获取激光雷达到map的变换
        auto [laser_to_map_x, laser_to_map_y] = getBaseLinkPositionInMap();
        
        // 获取机器人的朝向角度
        double robot_yaw = getRobotYawInMap();
        
        // 转换激光点
        double angle = scan_msg.angle_min;
        for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
            double range = scan_msg.ranges[i];
            
            // 过滤无效数据
            if (std::isnan(range) || std::isinf(range) || 
                range < scan_msg.range_min || range > scan_msg.range_max) {
                angle += scan_msg.angle_increment;
                continue;
            }
            
            // 激光点在激光雷达坐标系中的坐标
            double x_laser = range * cos(angle);
            double y_laser = range * sin(angle);
            
            // 变换到机器人坐标系（base_link）
            double cos_yaw = cos(robot_yaw);
            double sin_yaw = sin(robot_yaw);
            double x_base = x_laser * cos_yaw - y_laser * sin_yaw;
            double y_base = x_laser * sin_yaw + y_laser * cos_yaw;
            
            // 变换到map坐标系
            double x_map = laser_to_map_x + x_base;
            double y_map = laser_to_map_y + y_base;
            
            // 转换到像素坐标
            auto [x_pixel, y_pixel] = mapToPixels(x_map, y_map);
            
            obstacle_pixels.push_back(std::make_pair(x_pixel, y_pixel));
            
            angle += scan_msg.angle_increment;
        }
        
        return obstacle_pixels;
    }
    
private:
    // 获取base_link在map中的位置
    std::pair<double, double> getBaseLinkPositionInMap() const {
        auto map_it = tf_cache_.find("map");
        if (map_it != tf_cache_.end()) {
            auto base_it = map_it->second.find("base_link");
            if (base_it != map_it->second.end()) {
                const auto& tf_msg = base_it->second;
                return std::make_pair(tf_msg.transform.translation.x, 
                                    tf_msg.transform.translation.y);
            }
        }
        return std::make_pair(0.0, 0.0);
    }
    
    // 获取机器人在map中的朝向
    double getRobotYawInMap() const {
        auto map_it = tf_cache_.find("map");
        if (map_it != tf_cache_.end()) {
            auto base_it = map_it->second.find("base_link");
            if (base_it != map_it->second.end()) {
                const auto& tf_msg = base_it->second;
                
                // 从四元数提取yaw角度
                double qx = tf_msg.transform.rotation.x;
                double qy = tf_msg.transform.rotation.y;
                double qz = tf_msg.transform.rotation.z;
                double qw = tf_msg.transform.rotation.w;
                
                return atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
            }
        }
        return 0.0;
    }
};

#endif // NAV_TF_INTEGRATION_H

