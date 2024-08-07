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
#include <chrono>

// 存储上一次的里程计信息
nav_msgs::Odometry last_odometry;
nav_msgs::Odometry cur_odometry;
double last_time=0;

void scanCallback(const sensor_msgs::LaserScan* data) {
    //if (last_odometry == nullptr) {
    //    last_odometry = cur_odometry;
    //    printf("Waiting for odometry message...");
    //    return;
    //}
    printf("Received laser scan message\n");
    auto last_pose = last_odometry.pose.pose;
    auto cur_pose = cur_odometry.pose.pose;
    auto last_orientation = last_pose.orientation;
    auto cur_orientation = cur_pose.orientation;

    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) - std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);
    double current_time = seconds.count() + nanoseconds.count() / 1e9;
    double time_diff = current_time - last_time;
    last_time = current_time;
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
        //printf("distance: %f\n", distance);
        distances_mm.push_back(/*distance != std::numeric_limits<float>::infinity()*/(!std::isnan(distance)) ? round(distance * 1000) : std::numeric_limits<int>::infinity());
    }
    printf("time_diff: %f, linear_displacement_mm: %f, angular_displacement_degrees: %f\n", time_diff, linear_displacement_mm, angular_displacement_degrees);
    // 格式化数据
    std::ofstream file("/home/sunny/dora_nav/build/slam/build/laser_data.dat", std::ios::app);
    file<< time_diff << " " << linear_displacement_mm << " " << angular_displacement_degrees << " ";
    for (const auto& distance : distances_mm) {
        file << distance << " ";
    }
    file << "\n";

    // 更新上一次的里程计信息
    last_odometry = cur_odometry;
}
int counter = 0;
int run(void *dora_context)
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) - std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);
    double last_time = seconds.count() + nanoseconds.count() / 1e9;
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
            printf("[c node] received input event: %s\n", id.c_str());
            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            std::vector<unsigned char> data;
            for (size_t i = 0; i < data_len; i++)
            {
                data.push_back(*(data_ptr + i));
            }
            printf("[c node] received input event: %s\n", id.c_str());
            if(strncmp(id.c_str(), "scan", 4) == 0){
                odom.header.stamp.sec = seconds.count();
                odom.header.stamp.nsec = nanoseconds.count();
                odom.pose.pose.position.x = 0*counter;
                odom.pose.pose.position.y = 0;
                odom.pose.pose.position.z = 0;
                odom.pose.pose.orientation.x = 0;
                odom.pose.pose.orientation.y = 0;
                odom.pose.pose.orientation.z = 0;
                last_odometry = cur_odometry;
                cur_odometry = odom;
                counter++;
                std::string json_str(data_ptr, data_len);
                //printf("json_str: %d\n", data_len);
                //printf("json_str: %s\n", json_str.c_str());
                //replace_null_with_nan(json_str);
                //printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                sensor_msgs::LaserScan scan = sensor_msgs::LaserScan::from_json(json_obj);
                print(scan);
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
    last_odometry.pose.pose.position.x = 0;
    last_odometry.pose.pose.position.y = 0;
    last_odometry.pose.pose.position.z = 0;
    last_odometry.pose.pose.orientation.x = 0;
    last_odometry.pose.pose.orientation.y = 0;
    last_odometry.pose.pose.orientation.z = 0;
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
