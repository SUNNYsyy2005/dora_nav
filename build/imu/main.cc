extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include "../../include/ros.h"

using namespace std;
using namespace boost::asio;
using json = nlohmann::json;

io_service io;
serial_port imu(io, "/dev/ttyUSB0");

// 全局变量
double imu_yaw_calibration = 0.0;
double roll = 0;
double pitch = 0;
double yaw = 0;
int seq = 0;
vector<double> euler(3, 0);
double accel_factor = 9.806 / 256.0;
double degrees2rad = M_PI / 180.0;

sensor_msgs::Imu imu_msg;


// 辅助函数
int H_(char data) {
    int re = static_cast<unsigned char>(data) * 256;
    return re > 32767 ? re - 65536 : re;
}

int L_(char data) {
    return static_cast<unsigned char>(data);
}

// 四元数转欧拉角
vector<double> quaternion_to_euler(vector<double> q, int degree_mode = 1) {
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    double pitch = asin(2 * (qw * qy - qz * qx));
    double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    if (degree_mode == 1) {
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;
    }

    return { roll, pitch, yaw };
}

// 欧拉角转四元数
vector<double> euler_to_quaternion(vector<double> euler, int degree_mode = 0) {
    double roll = euler[0], pitch = euler[1], yaw = euler[2];
    if (degree_mode == 1) {
        roll = roll * M_PI / 180.0;
        pitch = pitch * M_PI / 180.0;
        yaw = yaw * M_PI / 180.0;
    }

    double qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    double qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    double qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    double qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

    return { qw, qx, qy, qz };
}

// 主函数
int run(void *dora_context) {
    imu.set_option(serial_port_base::baud_rate(115200));
    vector<unsigned char> write_data = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
    boost::asio::write(imu, boost::asio::buffer(write_data));

    while (true) {
        vector<unsigned char> data(40);
        boost::asio::read(imu, boost::asio::buffer(data));
        //printf("data size: %d\n", data.size());
        if (data[0] == 0xA5 && data[1] == 0x5A && data.back() == 0xAA) {
            double yaw_deg = (H_(data[3]) + L_(data[4])) / 10.0;

            if (yaw_deg > 180.0) yaw_deg -= 360.0;
            if (yaw_deg < -180.0) yaw_deg += 360.0;

            yaw = degrees2rad * yaw_deg;
            pitch = degrees2rad * (H_(data[7]) + L_(data[8])) / 10.0;
            roll = degrees2rad * (H_(data[5]) + L_(data[6])) / 10.0;
            imu_msg.linear_acceleration.x = 9.806 * (H_(data[9]) + L_(data[10])) / 16384.0;
            imu_msg.linear_acceleration.y = 9.806 * (H_(data[11]) + L_(data[12])) / 16384.0;
            imu_msg.linear_acceleration.z = 9.806 * (H_(data[13]) + L_(data[14])) / 16384.0;

            imu_msg.angular_velocity.x = degrees2rad * (H_(data[15]) + L_(data[16])) / 32.8;
            imu_msg.angular_velocity.y = degrees2rad * (H_(data[17]) + L_(data[18])) / 32.8;
            imu_msg.angular_velocity.z = degrees2rad * (H_(data[19]) + L_(data[20])) / 32.8;            

            euler = { roll, pitch, yaw };
            vector<double> q = euler_to_quaternion(euler);

            imu_msg.orientation.x = q[1];
            imu_msg.orientation.y = q[2];
            imu_msg.orientation.z = q[3];
            imu_msg.orientation.w = q[0];

            nlohmann::json json_obj = imu_msg.to_json();
            std::string json_str = json_obj.dump();
            printf("%s\n", json_str.c_str());
            const char* char_ptr = json_str.c_str();
            char* non_const_char_ptr = new char[json_str.size() + 1];
            std::memcpy(non_const_char_ptr, char_ptr, json_str.size() + 1);

            std::string out_id = "data";
            int result = dora_send_output(dora_context, &out_id[0], out_id.length(), reinterpret_cast<char*>(non_const_char_ptr), json_str.size());
            if (result != 0)
            {
                std::cerr << "failed to send output" << std::endl;
                return 1;
            }
            //cout << json_string << endl;

            // 如果需要发送或处理该JSON数据
            // const char* json_bytes = json_string.c_str();
            // 发送 json_bytes 数据...
        // printf("[c node] received event\n");
        }
    }

    return 0;
}
int main()
{
    std::cout << "dora node A driver" << std::endl;
 
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
 
 
    std::cout << "exit dora node A ..." << std::endl;
    return ret;
}