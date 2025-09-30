#ifndef AMCL_TF_INTEGRATION_H
#define AMCL_TF_INTEGRATION_H

#include "../../include/ros.h"
#include "include/pf/pf_vector.h"
#include <map>
#include <memory>

// AMCL-TF集成接口
class AMCLTfInterface {
private:
    std::map<std::string, std::map<std::string, geometry_msgs::TransformStamped>> tf_cache_;
    
public:
    // 更新TF变换缓存
    void updateTfCache(const tf2_msgs::TFMessage& tf_msg) {
        for (const auto& transform_stamped : tf_msg.transforms) {
            std::string parent = transform_stamped.header.frame_id;
            std::string child = transform_stamped.child_frame_id;
            tf_cache_[parent][child] = transform_stamped;
        }
    }
    
    // 获取base_link在地图中的位姿
    pf_vector_t getBaseLinkInMaP() const {
        pf_vector_t pose = {0, 0, 0};
        
        auto map_it = tf_cache_.find("map");
        if (map_it != tf_cache_.end()) {
            auto base_it = map_it->second.find("base_link");
            if (base_it != map_it->second.end()) {
                const auto& tf_msg = base_it->second;
                
                // 提取位置和角度
                pose.v[0] = tf_msg.transform.translation.x; // x坐标 (米)
                pose.v[1] = tf_msg.transform.translation.y; // y坐标 (米)
                
                // 从四元数提取yaw角（绕z轴旋转）
                double qx = tf_msg.transform.rotation.x;
                double qy = tf_msg.transform.rotation.y;
                double qz = tf_msg.transform.rotation.z;
                double qw = tf_msg.transform.rotation.w;
                
                pose.v[2] = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
            }
        }
        
        return pose;
    }
    
    // 获取里程计数据（base_link在odom中的位姿）
    pf_vector_t getOdometryPose() const {
        pf_vector_t pose = {0, 0, 0};
        
        auto odom_it = tf_cache_.find("odom");
        if (odom_it != tf_cache_.end()) {
            auto base_it = odom_it->second.find("base_link");
            if (base_it != odom_it->second.end()) {
                const auto& tf_msg = base_it->second;
                
                // 提取位置（米制坐标）
                pose.v[0] = tf_msg.transform.translation.x;
                pose.v[1] = tf_msg.transform.translation.y;
                
                // 从四元数提取yaw角
                double qx = tf_msg.transform.rotation.x;
                double qy = tf_msg.transform.rotation.y;
                double qz = tf_msg.transform.rotation.z;
                double qw = tf_msg.transform.rotation.w;
                
                pose.v[2] = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
            }
        }
        
        return pose;
    }
    
    // 计算里程计增量
    pf_vector_t calculateOdomDelta(const pf_vector_t& last_odom, const pf_vector_t& current_odom) {
        pf_vector_t delta = pf_vector_zero();
        
        // 计算位置增量
        double dx = current_odom.v[0] - last_odom.v[0];
        double dy = current_odom.v[1] - last_odom.v[1];
        
        // 将增量转换到base_link坐标系
        double cos_theta = cos(last_odom.v[2]);
        double sin_theta = sin(last_odom.v[2]);
        
        delta.v[0] = dx * cos_theta + dy * sin_theta;
        delta.v[1] = -dx * sin_theta + dy * cos_theta;
        delta.v[2] = normalizeAngle(current_odom.v[2] - last_odom.v[2]);
        
        return delta;
    }
    
    // 转换SLAM位姿到AMCL格式
    geometry_msgs::Pose2D slamPoseToAMCL(const pf_vector_t& slam_pose) {
        geometry_msgs::Pose2D amcl_pose;
        amcl_pose.x = slam_pose.v[0];
        amcl_pose.y = slam_pose.v[1];
        amcl_pose.theta = slam_pose.v[2];
        return amcl_pose;
    }
    
    // 标准化角度到[-π, π]
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }
    
    // 检查是否有有效的TF数据
    bool hasValidTransform(const std::string& parent_frame, const std::string& child_frame) const {
        auto parent_it = tf_cache_.find(parent_frame);
        if (parent_it == tf_cache_.end()) return false;
        
        auto child_it = parent_it->second.find(child_frame);
        return child_it != parent_it->second.end();
    }
};

#endif // AMCL_TF_INTEGRATION_H

