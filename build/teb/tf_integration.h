#ifndef TEB_TF_INTEGRATION_H
#define TEB_TF_INTEGRATION_H

#include "../../include/ros.h"
#include "inc/pose_se2.h"
#include <map>
#include <memory>

namespace teb_local_planner {

// TEB-TF集成接口
class TEBTfInterface {
private:
    std::map<std::string, std::map<std::string, geometry_msgs::TransformStamped>> tf_cache_;
    
    // 坐标系变换参数 - 与SLAM生成的地图保持一致
    static constexpr double SCALE_FACTOR = 0.005;  // 像素到米的缩放因子 (5mm/pixel)
    static constexpr int MAP_SIZE = 800;           // 地图尺寸（像素）
    
public:
    // 更新TF变换缓存
    void updateTfCache(const tf2_msgs::TFMessage& tf_msg) {
        for (const auto& transform_stamped : tf_msg.transforms) {
            std::string parent = transform_stamped.header.frame_id;
            std::string child = transform_stamped.child_frame_id;
            tf_cache_[parent][child] = transform_stamped;
        }
    }
    
    // 获取base_link在map坐标系中的位姿（用于TEB起点）
    PoseSE2 getRobotPoseInMap() const {
        PoseSE2 robot_pose;
        
        auto map_it = tf_cache_.find("map");
        if (map_it != tf_cache_.end()) {
            auto base_it = map_it->second.find("base_link");
            if (base_it != map_it->second.end()) {
                const auto& tf_msg = base_it->second;
                
                // 提取位置（米制坐标）
                double x_m = tf_msg.transform.translation.x;
                double y_m = tf_msg.transform.translation.y;
                
                // 从四元数提取yaw角
                double qx = tf_msg.transform.rotation.x;
                double qy = tf_msg.transform.rotation.y;
                double qz = tf_msg.transform.rotation.z;
                double qw = tf_msg.transform.rotation.w;
                
                double yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
                
                robot_pose = PoseSE2(x_m, y_m, yaw);
            }
        }
        
        return robot_pose;
    }
    
    // 激光雷达坐标变换（laser -> map）
    std::vector<Eigen::Vector2d> transformLaserScanToMap(
        const sensor_msgs::LaserScan& scan_msg) const {
        
        std::vector<Eigen::Vector2d> obstacles_in_map;
        
        // 获取激光雷达到base_link的变换
        Eigen::Isometry2d laser_to_base_link = getTransform_2D("base_link", "laser");
        
        // 获取base_link到map的变换
        Eigen::Isometry2d base_link_to_map = getTransform_2D("map", "base_link");
        
        // 组合变换：laser -> base_link -> map
        Eigen::Isometry2d laser_to_map = base_link_to_map * laser_to_base_link;
        
        // 转换激光点
        double angle = scan_msg.angle_min;
        for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
            double range = scan_msg.ranges[i];
            
            // 过滤无效数据
            if (std::isnan(range) || std::isinf(range) || range < scan_msg.range_min || range > scan_msg.range_max) {
                angle += scan_msg.angle_increment;
                continue;
            }
            
            // 激光点在laser坐标系中的坐标
            double x_laser = range * cos(angle);
            double y_laser = range * sin(angle);
            
            // 变换到map坐标系
            Eigen::Vector2d point_laser(x_laser, y_laser);
            Eigen::Vector2d point_map = laser_to_map * point_laser;
            
            obstacles_in_map.push_back(point_map);
            
            angle += scan_msg.angle_increment;
        }
        
        return obstacles_in_map;
    }
    
    // 目标点坐标变换到map坐标系
    PoseSE2 transformGoalToMap(const geometry_msgs::Pose2D& goal_pose) const {
        // 假设goal_pose已经在map坐标系中
        // 如果需要从其他坐标系转换，可以在这里添加变换逻辑
        
        PoseSE2 goal_in_map(goal_pose.x, goal_pose.y, goal_pose.theta);
        return goal_in_map;
    }
    
    // 速度指令变换（从base_link到map）
    geometry_msgs::Twist transformVelocityToMap(const geometry_msgs::Twist& cmd_vel) const {
        geometry_msgs::Twist transformed_vel = cmd_vel;
        
        // 这里简化处理，假设速度指令已经是正确的坐标系
        // 在实际应用中可能需要考虑坐标变换
        
        return transformed_vel;
    }
    
private:
    // 获取2D变换矩阵
    Eigen::Isometry2d getTransform_2D(const std::string& parent_frame, const std::string& child_frame) const {
        Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
        
        auto parent_it = tf_cache_.find(parent_frame);
        if (parent_it != tf_cache_.end()) {
            auto child_it = parent_it->second.find(child_frame);
            if (child_it != parent_it->second.end()) {
                const auto& tf_msg = child_it->second;
                
                // 2D位置
                Eigen::Translation2d translation(tf_msg.transform.translation.x, 
                                                tf_msg.transform.translation.y);
                
                // 2D旋转（从四元数提取yaw）
                double qx = tf_msg.transform.rotation.x;
                double qy = tf_msg.transform.rotation.y;
                double qz = tf_msg.transform.rotation.z;
                double qw = tf_msg.transform.rotation.w;
                
                double yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
                Eigen::Rotation2Dd rotation(yaw);
                
                transform = translation * rotation;
            }
        }
        
        return transform;
    }
};

} // namespace teb_local_planner

#endif // TEB_TF_INTEGRATION_H

