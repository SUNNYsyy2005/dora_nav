/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

class Request;




#include "input.h"
#include <string>

//#include <std_msgs/Int32.h>
//#include <pcl/point_types.h>
//#include <pcl_ros/impl/transforms.hpp>
//#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
//#include <pcl_ros/point_cloud.h>
#include <regex>
#include <atomic>
#include <thread>
#include <memory>
#include <mutex>
#include <algorithm>
/* #include "lslidar_cx_driver/lslidar_control.h"
#include "lslidar_cx_driver/time_service.h"
#include "lslidar_cx_driver/motor_speed.h"
#include "lslidar_cx_driver/motor_control.h"
#include "lslidar_cx_driver/remove_control.h"
#include "lslidar_cx_driver/data_port.h"
#include "lslidar_cx_driver/dev_port.h"
#include "lslidar_cx_driver/data_ip.h"
#include "lslidar_cx_driver/destination_ip.h" */


namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int SIZE_BLOCK = 100;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE;
    static const float DISTANCE_RESOLUTION = 0.01f; //meters
    static const uint16_t UPPER_BANK = 0xeeff;

// special defines for lslidarlidar support
    static const int FIRINGS_PER_BLOCK = 2;
    static const int SCANS_PER_FIRING = 16;
    static const int SCANS_PER_FIRING_CX = 32;

    static const int FIRING_TOFFSET = 32;
    //  static const int PACKET_SIZE = 1212;
    static const int BLOCKS_PER_PACKET = 12;
    static const int FIRINGS_PER_PACKET_CX = BLOCKS_PER_PACKET;
    static const int SCANS_PER_PACKET = SCANS_PER_FIRING * FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET; //384
    //unit:meter
    static const float R1_ = 0.0361f;        // C16 4.0 or C32 4.0
    static const float R1_C32W = 0.03416f;   // 新C32W   byte[1211] = 0x47
    static const float R1_90 = 0.0209f;      // 90度
    static const double R2_ = 0.0431;        // c16 3.0
    static const double R3_ = 0.0494;        // c32 3.0 

    static const int conversionAngle_ = 2025;
    static const int conversionAngle_90 = 2776; // 90度
    static const int conversionAngle_C16_3 = 1468;  //C16 3.0
    static const int conversionAngle_C32_3 = 1298;  //C32 3.0
// Pre-compute the sine and cosine for the altitude angles.
    //c32 32度
    static const float c32_vertical_angle[32] = {
            -16.0f, -8.0f, 0.0f, 8.0f, -15.0f, -7.0f, 1.0f, 9.0f,
            -14.0f, -6.0f, 2.0f, 10.0f, -13.0f, -5.0f, 3.0f, 11.0f,
            -12.0f, -4.0f, 4.0f, 12.0f, -11.0f, -3.0f, 5.0f, 13.0f,
            -10.0f, -2.0f, 6.0f, 14.0f, -9.0f, -1.0f, 7.0f, 15.0f};

    //c32 70度  0x45    W_P
    static const float c32wp_vertical_angle[32] = {
            -51.0f, -31.0f, -9.0f, 3.0f, -48.5f, -28.0f, -7.5f, 4.5f,
            -46.0f, -25.0f, -6.0f, 6.0f, -43.5f, -22.0f, -4.5f, 7.5f,
            -41.0f, -18.5f, -3.0f, 9.0f, -38.5f, -15.0f, -1.5f, 11.0f,
            -36.0f, -12.0f, 0.0f, 13.0f, -33.5f, -10.5f, 1.5f, 15.0f};
    
    //c32 70度  0x46    w 
    static const float c32_70_vertical_angle[32] = {
            -54.0f, -31.0f, -8.0f, 2.66f, -51.5f, -28.0f, -6.66f, 4.0f,
            -49.0f, -25.0f, -5.33f, 5.5f, -46.0f, -22.0f, -4.0f, 7.0f,
            -43.0f, -18.5f, -2.66f, 9.0f, -40.0f, -15.0f, -1.33f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.0f, 1.33f, 15.0f};
    
    //c32 70度  0x47    wn wb
    static const float c32wn_vertical_angle2[32] = {
            -54.7f, -31.0f, -9.0f, 3.0f, -51.5f, -28.0f, -7.5f, 4.5f,
            -49.0f, -25.0f, -6.0f, 6.0f, -46.0f, -22.0f, -4.5f, 7.5f,
            -43.0f, -18.5f, -3.0f, 9.0f, -40.0f, -15.0f, -1.5f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.5f, 1.5f, 15.0f};

    //c32 70度  0x47    wn wb
    static const float c32wb_vertical_angle2[32] = {
            -54.7f, -9.0f, -31.0f, 3.0f, -51.5f, -7.5f, -28.0f, 4.5f,
            -49.0f, -6.0f, -25.0f, 6.0f, -46.0f, -4.5f, -22.0f, 7.5f,
            -43.0f, -3.0f, -18.5f, 9.0f, -40.0f, -1.5f, -15.0f, 11.0f,
            -37.0f, 0.0f, -12.0f, 13.0f, -34.0f, 1.5f, -10.5f, 15.0f};

    //c32 90度
    static const float c32_90_vertical_angle[32] = {
            2.487f, 25.174f, 47.201f, 68.819f, 5.596f, 27.811f, 49.999f, 71.525f,
            8.591f, 30.429f, 52.798f, 74.274f, 11.494f, 33.191f, 55.596f, 77.074f,
            14.324f, 36.008f, 58.26f, 79.938f, 17.096f, 38.808f, 60.87f, 82.884f,
            19.824f, 41.603f, 63.498f, 85.933f, 22.513f, 44.404f, 66.144f, 89.105f};

    static const float c32rn_vertical_angle[32] = {
            2.487f, 47.201f, 25.174f, 68.819f, 5.596f, 49.999f, 27.811f, 71.525f,
            8.591f, 52.798f, 30.429f, 74.274f, 11.494f, 55.596f, 33.191f, 77.074f,
            14.324f, 58.26f, 36.008f, 79.938f, 17.096f, 60.87f, 38.808f, 82.884f,
            19.824f, 63.498f, 41.603f, 85.933f, 22.513f, 66.144f, 44.404f, 89.105f};

    static float c16_vertical_angle[16] = {-16.0f, 0.0f, -14.0f, 2.0f, -12.0f, 4.0f, -10.0f, 6.0f, 
                                           -8.0f, 8.0f, -6.0f, 10.0f, -4.0f, 12.0f, -2.0f, 14.0f};

    static const int c16_remap_angle[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

    static const float c8_vertical_angle[8] = {-10.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c8f_vertical_angle[8] = {-2.0f, 6.0f, 0.0f, 8.0f, 2.0f, 10.0f, 4.0f, 12.0f};
    
    static const float ckm8_vertical_angle[8] = {-12.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    //static const float c4_vertical_angle[4] = {-12.0f, 8.0f, 0.0f, 12.0f};

    static const float c1_vertical_angle[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //c16 3.0
    static const float c16_30_vertical_angle[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

    // c32 3.0
    static const float c32_30_vertical_angle[32] = {-16, 0, -15, 1, -14, 2, -13, 3, -12, 4, -11, 5, -10, 6, -9, 7, -8,
                                                   8, -7, 9, -6, 10, -5, 11, -4, 12, -3, 13, -2, 14, -1, 15};

    static const uint8_t adjust_angle_index[32] = {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9,
                                                   25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};

    union TwoBytes {
        uint16_t distance;
        uint8_t bytes[2];
    };

    struct RawBlock {
        uint16_t header;
        uint16_t rotation;  //0-35999
        uint8_t data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint8_t time_stamp[10];
        uint8_t factory[2];
    };

    struct FiringCX {
        uint16_t firing_azimuth[FIRINGS_PER_PACKET_CX];
        int azimuth[SCANS_PER_PACKET];
        float distance[SCANS_PER_PACKET];
        float intensity[SCANS_PER_PACKET];
    };

    /* struct PointXYZIRT {
        float x;
        float y;
        float z;
        float intensity;
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure our new allocators are aligned
    } */
     //enforce SSE padding for correct memory alignment

    static std::string lidar_type;

    class LslidarDriver {
    public:
        LslidarDriver(Ros::NodeHandle &node, Ros::NodeHandle &private_nh);

        ~LslidarDriver() {}

        bool checkPacketValidity(lslidar_cx_driver::LslidarPacketPtr &packet);

        //check if a point is in the required range
        // bool isPointInRange(const double &distance);
        
        // bool isPointInAngle(const double &azimuth);
        
        bool loadParameters();

        void initTimeStamp();

        bool createRosIO();

        void difopPoll();

        void publishPointcloud();

        void publishScan(void* dora_context);

        /* bool powerOn(lslidar_cx_driver::lslidar_control::Request &req,
                     lslidar_cx_driver::lslidar_control::Response &res);

        bool motorControl(lslidar_cx_driver::motor_control::Request &req,
                          lslidar_cx_driver::motor_control::Response &res);

        bool removeControl(lslidar_cx_driver::remove_control::Request &req,
                          lslidar_cx_driver::remove_control::Response &res);

        bool motorSpeed(lslidar_cx_driver::motor_speed::Request &req,
                        lslidar_cx_driver::motor_speed::Response &res);

        bool timeService(lslidar_cx_driver::time_service::Request &req,
                         lslidar_cx_driver::time_service::Response &res);

        bool setDataPort(lslidar_cx_driver::data_port::Request &req,
                         lslidar_cx_driver::data_port::Response &res);

        bool setDevPort(lslidar_cx_driver::dev_port::Request &req,
                        lslidar_cx_driver::dev_port::Response &res);

        bool setDataIp(lslidar_cx_driver::data_ip::Request &req,
                       lslidar_cx_driver::data_ip::Response &res);

        bool setDestinationIp(lslidar_cx_driver::destination_ip::Request &req,
                              lslidar_cx_driver::destination_ip::Response &res); */

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        //void decodePacket(const RawPacket* &packet);
        void decodePacket(lslidar_cx_driver::LslidarPacketPtr &packet);

        bool poll(void* dora_context);

        void pointcloudToLaserscan(const sensor_msgs::PointCloud2 &cloud_msg, sensor_msgs::LaserScan &output_scan);

        bool initialize();

        bool determineLidarType();

    public:
        int msop_udp_port{};
        int difop_udp_port{};
        int scan_num{};
        int point_num{};
        int angle_disable_min{};
        int angle_disable_max{};
        int angle_able_min{};
        int angle_able_max{};
        uint16_t last_azimuth;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;
        int return_mode;
        int fpga_type{};

        in_addr lidar_ip{};
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string pointcloud_topic;

        bool use_time_service{};
        bool pcl_type{};
        bool publish_scan{};
        bool coordinate_opt{};
        bool is_first_sweep;
        bool add_multicast{};
        bool angle_change{};
        // std::string c32_type;
        double distance_unit{};
        double min_range{};
        double max_range{};

        double sweep_end_time;
        double angle_base{};
        float cos_azimuth_table[36000]{};
        float sin_azimuth_table[36000]{};
        float R1 = 0.0f;

        boost::shared_ptr<Input> msop_input_;
        boost::shared_ptr<Input> difop_input_;
        boost::shared_ptr<boost::thread> difop_thread_;

        //lslidar_driver::LslidarScanPtr sweep_data;
        //lslidar_driver::LslidarScanPtr sweep_data_bak;
        std::mutex pointcloud_lock;
        Ros::NodeHandle nh;
        Ros::NodeHandle pnh;
        //Ros::Publisher packet_pub;
        Ros::Publisher pointcloud_pub;
        Ros::Publisher scan_pub;
        Ros::Publisher packets_status_pub;
        /* Ros::ServiceServer time_service_;               //授时
        Ros::ServiceServer lslidar_control_service_;    //上下电
        Ros::ServiceServer motor_control_service_;      //雷达转动/停转
        Ros::ServiceServer remove_control_service_;      //雷达去除雨雾尘级别
        Ros::ServiceServer motor_speed_service_;        //雷达频率
        Ros::ServiceServer data_port_service_;          //数据包端口
        Ros::ServiceServer dev_port_service_;           //设备包端口
        Ros::ServiceServer data_ip_service_;            //数据包ip
        Ros::ServiceServer destination_ip_service_;     //设备包ip */

        unsigned char difop_data[1206]{};
        unsigned char packetTimeStamp[10];
        struct tm cur_time{};
        Stamp timeStamp;
        Stamp timeStamp_bak;
        double packet_rate;
        double current_packet_time;
        double last_packet_time;

        FiringCX firings{};
        float scan_altitude[32] = {0.0f};
        float cos_scan_altitude[32] = {0.0f};
        float sin_scan_altitude[32] = {0.0f};
        float msc16_adjust_angle[16] = {0.0f};
        float msc16_offset_angle[16] = {0.0f};

        double horizontal_angle_resolution;
        int adjust_angle[4];
        int config_vert_num;
        double config_vertical_angle_32[32] = {0};
        double config_vertical_angle_tmp[32] = {0};
        double config_vertical_angle_16[16] = {0};

        int lidar_number_;
        int ring_;
        bool is_msc16;
        std::atomic<bool> is_get_difop_;
        std::atomic<bool> start_process_msop_;
        std::atomic<int> time_service_mode_{0};
        int remove_rain_flag;
        int conversionAngle{};
        int config_vertical_angle_flag;

        pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
        sensor_msgs::LaserScan::Ptr scan_msg;
        sensor_msgs::LaserScan::Ptr scan_msg_bak;
        uint point_size;
        int packet_num{};
    };

    typedef pcl::PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointcloud;


}  // namespace lslidar_driver

//POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRT,
//                                  (float, x, x)
//                                          (float, y, y)
//                                          (float, z, z)
//                                          (float, intensity, intensity)
//                                          (std::uint16_t, ring, ring)
//                                          (float, time, time))

#endif
/*
bool LslidarDriver::determineLidarType(){
        lslidar_cx_driver::LslidarPacketPtr pkt(new lslidar_cx_driver::LslidarPacket);
        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc_ = msop_input_->getPacket(pkt);
            if (rc_ == 0) break;
            if (rc_ < 0) return false;
        }

        if (pkt->data[1211] == 0x01) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C1";
            ROS_INFO("lidar type: C1");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x03) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C1P";
            ROS_INFO("lidar type: C1P");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        // } else if (pkt->data[1211] == 0x04) {
        //     for (int i = 0; i < 4; ++i) {
        //         sin_scan_altitude[i] = sin(c4_vertical_angle[i] * DEG_TO_RAD);
        //         cos_scan_altitude[i] = cos(c4_vertical_angle[i] * DEG_TO_RAD);
        //     }
        //     ring_ = 4;
        //     lidar_number_ = 4;
        //     R1 = R1_;
        //     conversionAngle = conversionAngle_;
        //     lidar_type = "C4";
        //     ROS_INFO("lidar type: C4");
        //     if (pkt->data[1210] == 0x39)  return_mode = 2;
        //     ROS_INFO("return mode: %d", return_mode);
        } else if(pkt->data[1211] == 0x06){
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            distance_unit = 0.1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "N301";
            ROS_INFO("lidar type: N301");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x07) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8f_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8f_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C8F";         
            ROS_INFO("lidar type: C8F");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x08) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "c8";               
            ROS_INFO("lidar type: C8");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x09) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(ckm8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(ckm8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "CKM8";
            ROS_INFO("lidar type: CKM8/C4");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x10) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C16";
            ROS_INFO("lidar type: C16");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x11) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            angle_change = true;  
            lidar_type = "MSC16";        
            ROS_INFO("lidar type: MSC16");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x12) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 17;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C16_domestic";
            ROS_INFO("lidar type: C16_domestic");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x20) {
            for (int i = 0; i < 32; ++i) {
                sin_scan_altitude[i] = sin(c32_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c32_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C32";
            ROS_INFO("lidar type: C32");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x45) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wp_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wp_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;                
            lidar_type = "C32WP";
            ROS_INFO("lidar type: C32WP");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x46) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32_70_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32_70_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            angle_change = true;
            conversionAngle = conversionAngle_;
            lidar_type = "C32W";
            ROS_INFO("lidar type: C32W");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x47) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wn_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wn_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_type = "C32WN";
            ROS_INFO("lidar type: C32WN");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x48) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wb_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wb_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_type = "C32WB";
            ROS_INFO("lidar type: C32WB");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x5a) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32_90_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32_90_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_type = "CH32R";
            ROS_INFO("lidar type: CH32R");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x5d) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32rn_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32rn_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_type = "CH32RN";
            ROS_INFO("lidar type: CH32R");
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x00 && pkt->data[1205] == 0x10) {
            for (int j = 0; j < 16; ++j) {
                    if (fabs(c16_30_vertical_angle[j] - config_vertical_angle_32[j]) > 1.5) {
                        config_vert_num++;
                    }
                }
            if (config_vert_num == 0) {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_32[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_32[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(c16_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c16_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R2_;
            conversionAngle = conversionAngle_C16_3;
            distance_unit = 0.25;
            lidar_type = "c16_3";
            ROS_INFO("lidar type: c16, version 3.0");
            if (pkt->data[1204] == 0x39) return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else if (pkt->data[1211] == 0x00 && pkt->data[1205] == 0x20) {
            for (int j = 0; j < 32; ++j) {
                config_vertical_angle_tmp[j] = config_vertical_angle_32[adjust_angle_index[j]];

                if (fabs(c32_30_vertical_angle[j] - config_vertical_angle_tmp[j]) > 3.0) {
                    config_vert_num++;
                }
            }
            if (config_vert_num == 0) {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(c32_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c32_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 31;
            lidar_number_ = 32;
            R1 = R3_;
            conversionAngle = conversionAngle_C32_3;
            lidar_type = "c32_3";
            ROS_INFO("lidar type: c32, version 3.0 ");
            if (pkt->data[1204] == 0x39) return_mode = 2;
            ROS_INFO("return mode: %d", return_mode);
        } else {
            ROS_ERROR("lidar type error,please check lidar model");
            Ros::shutdown();
            return false;
        }

        start_process_msop_ = true;
    }
    */
