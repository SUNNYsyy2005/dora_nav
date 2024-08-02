extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>

#include "../../include/ros.h"

#include <fstream>
#include <cmath>
#include <ctime>


// 存储上一次的里程计信息
nav_msgs::Odometry* last_odometry;
nav_msgs::Odometry* cur_odometry;

void scanCallback(const sensor_msgs::LaserScan* data) {
    if (last_odometry == nullptr) {
        last_odometry = cur_odometry;
        printf("Waiting for odometry message...");
        return;
    }

    auto last_pose = last_odometry->pose.pose;
    auto cur_pose = cur_odometry->pose.pose;
    auto last_orientation = last_pose.orientation;
    auto cur_orientation = cur_pose.orientation;

    // 计算时间差
    double current_time = data->header.stamp.sec + data->header.stamp.nsec / 1e9;
    double last_time = last_odometry->header.stamp.sec + last_odometry->header.stamp.nsec / 1e9;
    double time_diff = current_time - last_time;

    // 提取上一次和当前的位置和方向
    auto last_position = last_pose.position;
    auto cur_position = cur_pose.position;

    tf::Quaternion last_orientation_quat(last_orientation.x, last_orientation.y, last_orientation.z, last_orientation.w);
    tf::Quaternion cur_orientation_quat(cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w);

    // 计算线位移
    double linear_displacement = std::sqrt(std::pow(cur_position.x - last_position.x, 2) +
                                           std::pow(cur_position.y - last_position.y, 2) +
                                           std::pow(cur_position.z - last_position.z, 2));
    double linear_displacement_mm = linear_displacement * 1000;  // 转换为毫米

    // 计算角位移
    double last_roll, last_pitch, last_yaw;
    double cur_roll, cur_pitch, cur_yaw;
    tf::getRPY(last_orientation_quat,last_roll, last_pitch, last_yaw);
    tf::getRPY(cur_orientation_quat,cur_roll, cur_pitch, cur_yaw);
    double angular_displacement = cur_yaw - last_yaw;  // 假设只考虑绕Z轴的旋转
    double angular_displacement_degrees = angular_displacement * 180.0 / M_PI;  // 转换为度

    // 计算时间戳（以毫秒为单位）
    int64_t timestamp = static_cast<int64_t>(std::time(nullptr) * 1000);

    // 将激光雷达的距离数据转换为毫米单位
    std::vector<int> distances_mm;
    for (const auto& distance : data->ranges) {
        distances_mm.push_back(distance != std::numeric_limits<float>::infinity() ? static_cast<int>(distance * 1000) : 11000);
    }
    printf("time_diff: %f, linear_displacement_mm: %f, angular_displacement_degrees: %f\n", time_diff, linear_displacement_mm, angular_displacement_degrees);
    // 格式化数据
    std::ofstream file("laser_data.dat", std::ios::app);
    file << time_diff << " " << linear_displacement_mm << " " << angular_displacement_degrees << " ";
    for (const auto& distance : distances_mm) {
        file << distance << " ";
    }
    file << "\n";

    // 更新上一次的里程计信息
    last_odometry = cur_odometry;
}
void replace_null_with_nan(std::string& json_str) {
    std::string null_str = "null";
    std::string nan_str = "-1";
    size_t pos = 0;
    while ((pos = json_str.find(null_str, pos)) != std::string::npos) {
        json_str.replace(pos, null_str.length(), nan_str);
        pos += nan_str.length();
    }
}
int run(void *dora_context)
{
    printf("[c node] running...\n");
    sensor_msgs::LaserScan scan;
    nav_msgs::Odometry odom;
    printf("[c node] running...\n");
    while(true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        printf("[c node] received event type: %d\n", ty);
        if (ty == DoraEventType_Input)
        {
            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            std::vector<unsigned char> data;
            for (size_t i = 0; i < data_len; i++)
            {
                data.push_back(*(data_ptr + i));
            }
            printf("[c node] received input event: %s\n", id.c_str());
            if(id == "tick"){
                printf("tick\n");
            }
            else if(id == "scan"){
                std::string json_str(data_ptr, data_len);
                printf("json_str: %s\n", json_str.c_str());
                replace_null_with_nan(json_str);
                printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                sensor_msgs::LaserScan scan = sensor_msgs::LaserScan::from_json(json_obj);
                scanCallback(&scan);
            }else if(id == "odom"){
                odom = nav_msgs::Odometry::from_vector(data);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    auto dora_context = init_dora_context_from_env();

    std::cout << "HELLO FROM C++ (using C API)" << std::endl;
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
