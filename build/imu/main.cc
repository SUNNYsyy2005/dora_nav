extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
 
#include <iostream>
#include <vector>
#include <cmath>
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include "../../include/ros.h"
 
const std::string serial_port = "/dev/ttyUSB1";
const unsigned long serial_baud = 115200;
 
int H_(unsigned char data) {
    int re = data * 256;
    return re > 32767 ? re - 65536 : re;
}
 
int L_(unsigned char data) {
    return data;
}
 
bool to_exit_process;
int message_count = 0; // 消息计数器
const int max_messages = 10000; // 最大消息数
 
int run(void *dora_context)
{
    to_exit_process = false;
    serial::Serial IMU;
 
    // Initialize serial port
    try {
        IMU.setPort(serial_port);
        IMU.setBaudrate(serial_baud);
        IMU.setTimeout(serial::Timeout::simpleTimeout(1000));
        IMU.open();
        if (IMU.isOpen()) {
            IMU.write("\xA5\x5A\x04\x01\x05\xAA\n");
        } else {
            std::cerr << "Failed to open serial port" << std::endl;
            return -1;
        }
    } catch (serial::IOException& e) {
        std::cerr << "Serial Exception: " << e.what() << std::endl;
        return -1;
    }
 
    while (!to_exit_process && message_count < max_messages)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }
 
        enum DoraEventType ty = read_dora_event_type(event);
 
        if (ty == DoraEventType_Input)
        {
            std::vector<unsigned char> buffer(256);
            size_t len = IMU.read(buffer, buffer.size());
 
            if (len > 0 && buffer[0] == 0xA5 && buffer[1] == 0x5A && buffer[len - 1] == 0xAA) {
                sensor_msgs::Imu imu;
 
                double yaw_deg = (H_(buffer[3]) + L_(buffer[4])) / 10.0;
                if (yaw_deg > 180.0) yaw_deg -= 360.0;
                if (yaw_deg < -180.0) yaw_deg += 360.0;
 
                imu.orientation.x = yaw_deg * M_PI / 180.0;
                imu.orientation.y = (H_(buffer[7]) + L_(buffer[8])) / 10.0 * M_PI / 180.0;
                imu.orientation.z = (H_(buffer[5]) + L_(buffer[6])) / 10.0 * M_PI / 180.0;
                imu.orientation.w = 1.0; // assuming w as 1.0 for this example
 
                imu.linear_acceleration.x = 9.806 * (H_(buffer[9]) + L_(buffer[10])) / 16384.0;
                imu.linear_acceleration.y = 9.806 * (H_(buffer[11]) + L_(buffer[12])) / 16384.0;
                imu.linear_acceleration.z = 9.806 * (H_(buffer[13]) + L_(buffer[14])) / 16384.0;
                imu.angular_velocity.x = (H_(buffer[15]) + L_(buffer[16])) / 32.8 * M_PI / 180.0;
                imu.angular_velocity.y = (H_(buffer[17]) + L_(buffer[18])) / 32.8 * M_PI / 180.0;
                imu.angular_velocity.z = (H_(buffer[19]) + L_(buffer[20])) / 32.8 * M_PI / 180.0;
 
                nlohmann::json json_obj = imu.to_json();
                std::string json_str = json_obj.dump();
                //printf("%s\n", json_str.c_str());
                const char* char_ptr = json_str.c_str();
                char* non_const_char_ptr = new char[json_str.size() + 1];
                std::memcpy(non_const_char_ptr, char_ptr, json_str.size() + 1);


                int result = dora_send_output(dora_context, &out_id[0], out_id.length(), reinterpret_cast<char*>(non_const_char_ptr), json_str.size());
                if (result != 0)
                {
                    std::cerr << "failed to send output" << std::endl;
                    return 1;
                }
                message_count++; // 递增消息计数器
                std::cout << "Message count: " << message_count << std::endl; // 打印消息计数器
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            to_exit_process = true;
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }
 
        free_dora_event(event);
    }
 
    if (message_count >= max_messages)
    {
        std::cout << "Reached the maximum number of messages: " << max_messages << std::endl;
        to_exit_process = true;
    }
 
    if (IMU.isOpen()) {
        IMU.close();
    }
 
    return 0;
}
 
int main()
{
    std::cout << "dora node A driver" << std::endl;
 
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
 
    to_exit_process = true;
 
    std::cout << "exit dora node A ..." << std::endl;
    return ret;
}