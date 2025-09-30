extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <eigen3/Eigen/Dense>
#include "../../include/ros.h"
#include "../../include/project_paths.h"

// 扩展卡尔曼滤波器用于位姿融合
class PoseFusionEKF {
private:
    // 状态向量: [x, y, theta, vx, vy, vtheta]
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    
    // 过程噪声协方差
    Eigen::MatrixXd process_noise_;
    
    // 观测模型
    Eigen::MatrixXd observation_model_;
    
    // 时间戳
    double last_update_time_;
    
public:
    PoseFusionEKF() {
        state_ = Eigen::VectorXd::Zero(6);
        covariance_ = Eigen::MatrixXd::Identity(6, 6);
        process_noise_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        observation_model_ = Eigen::MatrixXd::Identity(3, 6).block(0, 0, 3, 3);
        last_update_time_ = 0.0;
    }
    
    void predict(double dt) {
        if (dt <= 0) return;
        
        // 状态转移矩阵
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = dt; // x += vx * dt
        F(1, 4) = dt; // y += vy * dt
        F(2, 5) = dt; // theta += vtheta * dt
        
        // 运动模型噪声
        Eigen::MatrixXd Q = process_noise_;
        
        // 预测步骤
        state_ = F * state_;
        covariance_ = F * covariance_ * F.transpose() + Q;
        
        // 角度标准化
        normalizeAngle(state_(2));
    }
    
    void updateFromSLAM(double x, double y, double theta, double confidence) {
        Eigen::VectorXd measurement(3);
        measurement << x, y, theta;
        
        updateMeasurement(measurement, confidence, "SLAM");
    }
    
    void updateFromAMCL(double x, double y, double theta, double confidence) {
        Eigen::VectorXd measurement(3);
        measurement << x, y, theta;
        
        updateMeasurement(measurement, confidence, "AMCL");
    }
    
    void updateFromIMU(double angular_velocity_z) {
        // IMU仅提供角速度信息
        state_(5) = angular_velocity_z; // 更新角速度
    }
    
    geometry_msgs::Pose2D getFusedPose() {
        geometry_msgs::Pose2D pose;
        pose.x = state_(0);
        pose.y = state_(1);
        pose.theta = state_(2);
        return pose;
    }
    
    double getConfidence() {
        // 基于协方差矩阵对角线元素计算置信度
        double trace = covariance_.trace();
        return 1.0 / (1.0 + trace * 0.1);
    }
    
private:
    void updateMeasurement(const Eigen::VectorXd& z, double confidence, const std::string& source) {
        // 观测噪声协方差（基于置信度调整）
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * (1.0 - confidence + 0.01);
        
        // 卡尔曼增益
        Eigen::MatrixXd H = observation_model_;
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();
        
        // 更新
        Eigen::VectorXd innovation = z - H * state_;
        normalizeAngle(innovation(2));
        
        state_ = state_ + K * innovation;
        covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * covariance_;
        
        normalizeAngle(state_(2));
    }
    
    void normalizeAngle(double& angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
    }
};

// 位姿融合接口
class PoseFusionInterface {
private:
    std::unique_ptr<PoseFusionEKF> ekf_;
    double last_slam_time_;
    double last_amcl_time_;
    double last_imu_time_;
    
public:
    PoseFusionInterface() : 
        ekf_(std::make_unique<PoseFusionEKF>()),
        last_slam_time_(0.0),
        last_amcl_time_(0.0),
        last_imu_time_(0.0) {}

    void slam_callback(const geometry_msgs::Pose2D* slam_pose) {
        if (!slam_pose) return;
        
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        double current_time = seconds.count();
        
        // SLAM位姿置信度估算（这里简化处理）
        double confidence = 0.8;
        
        ekf_->updateFromSLAM(slam_pose->x, slam_pose->y, slam_pose->theta, confidence);
        last_slam_time_ = current_time;
    }
    
    void amcl_callback(const geometry_msgs::Pose2D* amcl_pose) {
        if (!amcl_pose) return;
        
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        double current_time = seconds.count();
        
        // AMCL位姿置信度估算
        double confidence = 0.9;
        
        ekf_->updateFromAMCL(amcl_pose->x, amcl_pose->y, amcl_pose->theta, confidence);
        last_amcl_time_ = current_time;
    }
    
    void imu_callback(const sensor_msgs::Imu* imu_data) {
        if (!imu_data) return;
        
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        double current_time = seconds.count();
        
        double dt = current_time - last_imu_time_;
        if (last_imu_time_ > 1e-9) { // 不是第一次
            ekf_->predict(dt);
        }
        
        ekf_->updateFromIMU(imu_data->angular_velocity.z);
        last_imu_time_ = current_time;
    }
    
    geometry_msgs::Pose2D get_fused_pose() {
        return ekf_->getFusedPose();
    }
    
    double get_confidence() {
        return ekf_->getConfidence();
    }
    
    void tick_callback(void* context) {
        // 定期发布融合位姿
        geometry_msgs::Pose2D fused_pose = get_fused_pose();
        double confidence = get_confidence();
        
        nlohmann::json output_json;
        output_json["x"] = fused_pose.x;
        output_json["y"] = fused_pose.y;
        output_json["theta"] = fused_pose.theta;
        output_json["confidence"] = confidence;
        output_json["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        std::string json_str = output_json.dump();
        
        std::string output_id = "fused_pose";
        int result = dora_send_output(
            context,
            const_cast<char*>(output_id.c_str()),
            output_id.length(),
            const_cast<char*>(json_str.c_str()),
            json_str.length()
        );
        
        if (result != 0) {
            std::cerr << "failed to send fused pose output" << std::endl;
        }
    }
};

// 全局实例
extern void* context_;
std::shared_ptr<PoseFusionInterface> pose_fusion;

// 回调函数
extern "C" void slam_pose_callback(const char* id, size_t id_len, const char* data, size_t data_len) {
    try {
        nlohmann::json json_data = nlohmann::json::parse(std::string(data, data_len));
        geometry_msgs::Pose2D slam_pose;
        slam_pose.x = json_data["x"];
        slam_pose.y = json_data["y"];
        slam_pose.theta = json_data["theta"];
        
        if (pose_fusion) {
            pose_fusion->slam_callback(&slam_pose);
        }
    } catch (const std::exception& e) {
        std::cerr << "SLAM pose callback error: " << e.what() << ::std::endl;
    }
}

extern "C" void amcl_pose_callback(const char* id, size_t id_len, const char* data, size_t data_len) {
    try {
        nlohmann::json json_data = nlohmann::json::parse(std::string(data, data_len));
        geometry_msgs::Pose2D amcl_pose;
        amcl_pose.x = json_data["x"];
        amcl_pose.y = json_data["y"];
        amcl_pose.theta = json_data["theta"];
        
        if (pose_fusion) {
            pose_fusion->amcl_callback(&amcl_pose);
        }
    } catch (const std::exception& e) {
        std::cerr << "AMCL pose callback error: " << e.what() << ::std::endl;
    }
}

extern "C" void imu_callback(const char* id, size_t id_len, const char* data, size_t data_len) {
    try {
        nlohmann::json json_data = nlohmann::json::parse(std::string(data, data_len));
        sensor_msgs::Imu imu_data;
        imu_data.angular_velocity.z = json_data["angular_velocity"]["z"];
        imu_data.linear_acceleration.x = json_data["linear_acceleration"]["x"];
        imu_data.linear_acceleration.y = json_data["linear_acceleration"]["y"];
        
        if (pose_fusion) {
            pose_fusion->imu_callback(&imu_data);
        }
    } catch (const std::exception& e) {
        std::cerr << "IMU callback error: " << e.what() << ::std::endl;
    }
}

extern "C" void tick_callback(const char* id, size_t id_len, const char* data, size_t data_len) {
    if (!pose_fusion) return;
    
    try {
        // 获取融合后的位姿
        geometry_msgs::Pose2D fused_pose = pose_fusion->get_fused_pose();
        double confidence = pose_fusion->get_confidence();
        
        // 添加置信度信息
        nlohmann::json output_json;
        output_json["x"] = fused_pose.x;
        output_json["y"] = fused_pose.y;
        output_json["theta"] = fused_pose.theta;
        output_json["confidence"] = confidence;
        output_json["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        std::string json_str = output_json.dump();
        
        std::string output_id = "fused_pose";
        int result = dora_send_output(
            context_,
            const_cast<char*>(output_id.c_str()),
            output_id.length(),
            const_cast<char*>(json_str.c_str()),
            json_str.length()
        );
        
        if (result != 0) {
            std::cerr << "failed to send fused pose output" << ::std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "tick callback error: " << e.what() << ::std::endl;
    }
}

extern "C" void* init_pose_fusion(void* context) {
    context_ = context;
    pose_fusion = std::make_shared<PoseFusionInterface>();
    
    printf("Pose Fusion初始化完成\n");
    return context;
}

extern "C" void run(void* context) {
    // 初始化融合器
    if (!pose_fusion) {
        init_pose_fusion(context);
    }
    
    printf("Pose Fusion: 开始运行\n");
    
    // 主运行循环
    while (true) {
        void* event = dora_next_event(context);
        
        if (event == NULL) {
            printf("Pose Fusion: 等待事件\n");
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
            
            printf("Pose Fusion输入: %s\n", id.c_str());
            
            try {
                if (id == "timer" || id == "tick") {
                    // Timer事件通常没有数据内容，直接处理
                    if (pose_fusion) {
                        pose_fusion->tick_callback(context);
                    }
                } else if (data_len > 0) {
                    // 只有当数据长度大于0时才解析JSON
                    nlohmann::json json_data = nlohmann::json::parse(std::string(data_ptr, data_len));
                    
                    if (id == "slam_pose") {
                        geometry_msgs::Pose2D slam_pose;
                        slam_pose.x = json_data["x"];
                        slam_pose.y = json_data["y"];
                        slam_pose.theta = json_data["theta"];
                        
                        if (pose_fusion) {
                            pose_fusion->slam_callback(&slam_pose);
                        }
                    } else if (id == "amcl_pose") {
                        geometry_msgs::Pose2D amcl_pose;
                        amcl_pose.x = json_data["x"];
                        amcl_pose.y = json_data["y"];
                        amcl_pose.theta = json_data["theta"];
                        
                        if (pose_fusion) {
                            pose_fusion->amcl_callback(&amcl_pose);
                        }
                    } else if (id == "imu_data") {
                        sensor_msgs::Imu imu_data;
                        imu_data.angular_velocity.z = json_data["angular_velocity"]["z"];
                        imu_data.linear_acceleration.x = json_data["linear_acceleration"]["x"];
                        imu_data.linear_acceleration.y = json_data["linear_acceleration"]["y"];
                        
                        if (pose_fusion) {
                            pose_fusion->imu_callback(&imu_data);
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Pose Fusion数据处理错误: " << e.what() << " (输入ID: " << id << ", 数据长度: " << data_len << ")" << std::endl;
            }
        }
        
        free_dora_event(event);
    }
}

// 全局变量
extern void* context_;
void* context_ = nullptr;
int main() { auto dora_context = init_dora_context_from_env(); run(dora_context); free_dora_context(dora_context); return 0; }
