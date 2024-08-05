#pragma once

#include <string>
#include <vector>
#include <cmath>

#include <vector>
#include <memory>

#include <array>

#include <nlohmann/json.hpp>
#include <boost/shared_ptr.hpp>



#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <limits>
#include <algorithm>

#include <cstddef>

#include <chrono>

namespace tf{
    class Quaternion {
        public:
            Quaternion(double x,double y,double z,double w){
                this->x = x;
                this->y = y;
                this->z = z;
                this->w = w;
            }
            Quaternion(){
                this->x = this->y = this->z = this->w = 0;
            }
            double x;
            double y;
            double z;
            double w;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["x"] = x;
                j["y"] = y;
                j["z"] = z;
                j["w"] = w;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Quaternion from_json(const nlohmann::json& j) {
                Quaternion quaternion;
                quaternion.x = j["x"];
                quaternion.y =j["y"];
                quaternion.z =j["z"];
                quaternion.w =j["w"];
                return quaternion;
            }
            static Quaternion from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    //Quaternion newQuaternion(double x, double y, double z, double w) {
    //    Quaternion q;
    //    q.x = x;
    //    q.y = y;
    //    q.z = z;
    //    q.w = w;
    //    return q;
    //}
    
}

class Vector3 {
    public:
        double x;
        double y;
        double z;
        nlohmann::json to_json() const{
             nlohmann::json j;
            j["x"] = x;
            j["y"] = y;
            j["z"] = z;
            return j;
        }
        std::vector<unsigned char> to_vector() const {
            nlohmann::json j = to_json();
            std::string json_str = j.dump();
            return std::vector<unsigned char>(json_str.begin(), json_str.end());
        }
        static Vector3 from_json(const nlohmann::json& j) {
            Vector3 vector3;
            vector3.x = j["x"];
            vector3.y =j["y"];
            vector3.z =j["z"];
            return vector3;
        }
        static Vector3 from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
};

class Stamp {
    public:
        long long sec;    // 秒
        long long nsec;   // 纳秒
        Stamp(){
            sec = 0;
            nsec = 0;
        }
        Stamp(long long sec, long long nsec){
            this->sec = sec;
            this->nsec = nsec;
        }
        nlohmann::json to_json() const{
            nlohmann::json j;
            j["sec"] = sec;
            j["nsec"] = nsec;
            return j;
        }
        std::vector<unsigned char> to_vector() const {
            nlohmann::json j = to_json();
            std::string json_str = j.dump();
            return std::vector<unsigned char>(json_str.begin(), json_str.end());
        }
        static Stamp from_json(const nlohmann::json& j) {
            Stamp stamp;
            stamp.sec = j["sec"];
            stamp.nsec =j["nsec"];
            return stamp;
        }
        static Stamp from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
        double toSec() const {
            return sec + nsec * 1e-9;
        }
        Stamp operator-(const Stamp& other) const {
            long long sec_diff = sec - other.sec;
            long long nsec_diff = nsec - other.nsec;
            if (nsec_diff < 0) {
                sec_diff -= 1;
                nsec_diff += 1000000000; // 1 秒 = 10^9 纳秒
            }
            return Stamp(sec_diff, nsec_diff);
        }
};
class Header {
    public:
        uint32_t seq;          // 扫描顺序增加的id
        Stamp stamp;              // 开始扫描的时间和与开始扫描的时间差
        std::string frame_id;      // 扫描的参考系名称
        nlohmann::json to_json() const{
             nlohmann::json j;
            j["seq"] = seq;
            j["stamp"] = stamp.to_json();
            j["frame_id"] = frame_id;
            return j;
        }
        std::vector<unsigned char> to_vector() const {
            nlohmann::json j = to_json();
            std::string json_str = j.dump();
            return std::vector<unsigned char>(json_str.begin(), json_str.end());
        }
        static Header from_json(const nlohmann::json& j) {
            Header header;
            header.seq = j["seq"];
            header.stamp = Stamp::from_json(j["stamp"]);
            header.frame_id =j["frame_id"];
            return header;
        }
        static Header from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
};
namespace sensor_msgs {
    class LaserScan {
        public:
            LaserScan(){
                header.seq = 632;
                header.stamp.sec = 16;
                header.stamp.nsec = 187000000;
                header.frame_id = "laser_frame";
                angle_min = -2.3561899662017822;
                angle_max = 2.3561899662017822;
                angle_increment = 0.004363314714282751;
                time_increment = 0.0;
                scan_time = 0.0;
                range_min = 0.10000000149011612;
                range_max = 10.0;
                ranges = {2.551586389541626, 2.5835444927215576, 2.5715863704681396, 2.5952813625335693, 2.60617995262146};
                intensities = {1.0, 2.0, 3.0, 4.0, 5.0};
            }
            Header header;             // Header结构体实例

            float angle_min;           // 开始扫描的角度
            float angle_max;           // 结束扫描的角度
            float angle_increment;     // 每一次扫描增加的角度

            float time_increment;      // 测量的时间间隔
            float scan_time;           // 扫描的时间间隔

            float range_min;           // 距离最小值
            float range_max;           // 距离最大值

            std::vector<float> ranges;         // 距离数组
            std::vector<float> intensities;    // 强度数组

            typedef boost::shared_ptr< ::sensor_msgs::LaserScan> Ptr;
            std::vector<float> json_to_vector(const nlohmann::json& j) const{
                std::vector<float> vec;
                for (const auto& val : j) {
                    if (val.is_string() && val.get<std::string>() == "NaN") {
                        vec.push_back(NAN);
                    } else {
                        vec.push_back(val.get<float>());
                    }
                }
                return vec;
            }
            nlohmann::json vector_to_json(const std::vector<float> & vec) const{
                nlohmann::json j = nlohmann::json::array();
                for (const auto& val : vec) {
                    if (std::isnan(val)) {
                        j.push_back("NaN");
                    } else {
                        j.push_back(val);
                    }
                }
                return j;
            }
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["header"]["seq"] = header.seq;
                j["header"]["stamp"]["sec"] = header.stamp.sec;
                j["header"]["stamp"]["nsec"] = header.stamp.nsec;
                j["header"]["frame_id"] = header.frame_id;
                j["angle_min"] = angle_min;
                j["angle_max"] = angle_max;
                j["angle_increment"] = angle_increment;
                j["time_increment"] = time_increment;
                j["scan_time"] = scan_time;
                j["range_min"] = range_min;
                j["range_max"] = range_max;
                j["ranges"] =  vector_to_json(ranges);//ranges;
                j["intensities"] = vector_to_json(intensities);//intensities;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static LaserScan from_json(const nlohmann::json& j) {
                LaserScan scan;
                scan.header.seq = j["header"]["seq"];
                scan.header.stamp.sec = j["header"]["stamp"]["sec"];
                scan.header.stamp.nsec = j["header"]["stamp"]["nsec"];
                scan.header.frame_id = j["header"]["frame_id"];
                scan.angle_min = j["angle_min"];
                scan.angle_max = j["angle_max"];
                scan.angle_increment = j["angle_increment"];
                scan.time_increment = j["time_increment"];
                scan.scan_time = j["scan_time"];
                scan.range_min = j["range_min"];
                scan.range_max = j["range_max"];
                scan.ranges = scan.json_to_vector(j["ranges"]);// j["ranges"].get<std::vector<float>>();
                scan.intensities = scan.json_to_vector(j["intensities"]);//j["intensities"].get<std::vector<float>>();
                return scan;
            }
            static LaserScan from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    //LaserScan newLaserScan() {
    //    LaserScan scan;
    //    scan.header.seq = 632;
    //     scan.header.stamp.sec = 16;
    //     scan.header.stamp.nsec = 187000000;
    //     scan.header.frame_id = "laser_frame";
    //     scan.angle_min = -2.3561899662017822;
    //     scan.angle_max = 2.3561899662017822;
    //     scan.angle_increment = 0.004363314714282751;
    //     scan.time_increment = 0.0;
    //     scan.scan_time = 0.0;
    //     scan.range_min = 0.10000000149011612;
    //     scan.range_max = 10.0;
    //     scan.ranges = {2.551586389541626, 2.5835444927215576, 2.5715863704681396, 2.5952813625335693, 2.60617995262146};
    //     scan.intensities = {1.0, 2.0, 3.0, 4.0, 5.0};
    //     return scan;
    // }
    
    class Imu {
        public:
            Imu(){
                header.seq = 50;
                header.stamp.sec = 17;
                header.stamp.nsec = 497000000;
                header.frame_id = "/imu_link";
                orientation.x = -1.4817208738488272e-05;
                orientation.y = 1.747766491371163e-05;
                orientation.z = 0.6963470506532196;
                orientation.w = 0.7177052211887164;
                orientation_covariance = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9};
                angular_velocity.x = 0.000868052236941635;
                angular_velocity.y = -0.000284583075071133;
                angular_velocity.z = 0.0018242036489127952;
                angular_velocity_covariance = {0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1};
                linear_acceleration.x = -0.2915581620976377;
                linear_acceleration.y = 0.03977790630637389;
                linear_acceleration.z = 9.805478448950305;
                linear_acceleration_covariance = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9};
            }
            Header header;

            tf::Quaternion orientation;
            std::array<double, 9> orientation_covariance; // 关于x, y, z轴的方向协方差，行优先

            Vector3 angular_velocity;
            std::array<double, 9> angular_velocity_covariance; // 关于x, y, z轴的角速度协方差，行优先

            Vector3 linear_acceleration;
            std::array<double, 9> linear_acceleration_covariance; // 关于x, y, z轴的线性加速度协方差，行优先
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["header"]["seq"] = header.seq;
                j["header"]["stamp"]["sec"] = header.stamp.sec;
                j["header"]["stamp"]["nsec"] = header.stamp.nsec;
                j["header"]["frame_id"] = header.frame_id;
                j["orientation"]["x"] = orientation.x;
                j["orientation"]["y"] = orientation.y;
                j["orientation"]["z"] = orientation.z;
                j["orientation"]["w"] = orientation.w;
                j["orientation_covariance"] = std::vector<double>(orientation_covariance.begin(),orientation_covariance.end());
                j["angular_velocity"]["x"] = angular_velocity.x;
                j["angular_velocity"]["y"] = angular_velocity.y;
                j["angular_velocity"]["z"] = angular_velocity.z;
                j["angular_velocity_covariance"] = std::vector<double>(angular_velocity_covariance.begin(),angular_velocity_covariance.end());
                j["linear_acceleration"]["x"] = linear_acceleration.x;
                j["linear_acceleration"]["y"] = linear_acceleration.y;
                j["linear_acceleration"]["z"] = linear_acceleration.z;
                j["linear_acceleration_covariance"] = std::vector<double>(linear_acceleration_covariance.begin(),linear_acceleration_covariance.end());
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Imu from_json(const nlohmann::json& j) {
                Imu imu;
                imu.header.seq = j["header"]["seq"];
                imu.header.stamp.sec = j["header"]["stamp"]["sec"];
                imu.header.stamp.nsec = j["header"]["stamp"]["nsec"];
                imu.header.frame_id = j["header"]["frame_id"];
                imu.orientation.x = j["orientation"]["x"];
                imu.orientation.y = j["orientation"]["y"];
                imu.orientation.z = j["orientation"]["z"];
                imu.orientation.w = j["orientation"]["w"];
                std::vector<double> vec = j["orientation_covariance"].get<std::vector<double>>();
                if (vec.size() != 9) {
                    throw std::runtime_error("JSON array size is not 9");
                }
                std::copy(vec.begin(), vec.end(),imu.orientation_covariance.begin());
                imu.angular_velocity.x = j["angular_velocity"]["x"];
                imu.angular_velocity.y = j["angular_velocity"]["y"];
                imu.angular_velocity.z = j["angular_velocity"]["z"];
                vec = j["angular_velocity_covariance"].get<std::vector<double>>();
                if (vec.size() != 9) {
                    throw std::runtime_error("JSON array size is not 9");
                }
                std::copy(vec.begin(), vec.end(),imu.angular_velocity_covariance.begin());
                imu.linear_acceleration.x = j["linear_acceleration"]["x"];
                imu.linear_acceleration.y = j["linear_acceleration"]["y"];
                imu.linear_acceleration.z = j["linear_acceleration"]["z"];
                vec = j["linear_acceleration_covariance"].get<std::vector<double>>();
                if (vec.size() != 9) {
                    throw std::runtime_error("JSON array size is not 9");
                }
                std::copy(vec.begin(), vec.end(),imu.linear_acceleration_covariance.begin());
                return imu;
            }
            static Imu from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    
    class PointField{
        public:
            std::string name;
            uint32_t offset;
            uint8_t datatype;
            uint32_t count;
    };
    class PointCloud2{
        public:
            Header header;
            uint32_t height;
            uint32_t width;
            std::vector<PointField> fields;
            bool is_bigendian;
            uint32_t point_step;
            uint32_t row_step;
            std::vector<unsigned char> data;
            bool is_dense;
    };
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2> PointCloud2Ptr;
    class PointCloud2Modifier {
        public:
            // 构造函数
            PointCloud2Modifier(PointCloud2& cloud) : cloud_(cloud) {}

            // 设置点云字段
            void setPointCloud2FieldsByString(int num_fields, const std::string& field_name) {
                if (field_name == "xyz") {
                    cloud_.fields.resize(3);
                    cloud_.fields[0].name = "x";
                    cloud_.fields[0].offset = 0;
                    cloud_.fields[0].datatype = 7; // FLOAT32
                    cloud_.fields[0].count = 1;

                    cloud_.fields[1].name = "y";
                    cloud_.fields[1].offset = 4;
                    cloud_.fields[1].datatype = 7; // FLOAT32
                    cloud_.fields[1].count = 1;

                    cloud_.fields[2].name = "z";
                    cloud_.fields[2].offset = 8;
                    cloud_.fields[2].datatype = 7; // FLOAT32
                    cloud_.fields[2].count = 1;

                    cloud_.point_step = 12;
                    cloud_.row_step = cloud_.point_step * cloud_.width;
                }
                // 可以根据需要添加更多字段类型
            }

            // 设置点云数据大小
            void resize(size_t size) {
                cloud_.data.resize(size);
            }

        private:
            PointCloud2& cloud_;
    };
    template<typename T>
    class PointCloud2Iterator {
        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = T;
            using difference_type = std::ptrdiff_t;
            using pointer = T*;
            using reference = T&;
            PointCloud2Iterator(PointCloud2& cloud,const std::string& field_name)
                    : cloud_(cloud), offset_(getFieldOffset(cloud, field_name)), ptr_(cloud.data.data()) {}
            PointCloud2Iterator(const PointCloud2& cloud, const std::string& field_name)
                    : cloud_(cloud), offset_(getFieldOffset(cloud, field_name)), ptr_(const_cast<unsigned char*>(cloud.data.data())) {}

            reference operator*(){
                return *reinterpret_cast<pointer>(ptr_ + offset_);
            }

            pointer operator->() {
                return reinterpret_cast<pointer>(ptr_ + offset_);
            }

            PointCloud2Iterator& operator++() {
                ptr_ += cloud_.point_step;
                return *this;
            }

            PointCloud2Iterator operator++(int) {
                PointCloud2Iterator tmp = *this;
                ++(*this);
                return tmp;
            }

            friend bool operator==(const PointCloud2Iterator& a, const PointCloud2Iterator& b) {
                return a.ptr_ == b.ptr_;
            }

            friend bool operator!=(const PointCloud2Iterator& a, const PointCloud2Iterator& b) {
                return a.ptr_ != b.ptr_;
            }

            static PointCloud2Iterator begin(const PointCloud2& cloud, const std::string& field_name) {
                return PointCloud2Iterator(cloud, field_name);
            }

            static PointCloud2Iterator end(const PointCloud2& cloud, const std::string& field_name) {
                PointCloud2Iterator it(cloud, field_name);
                it.ptr_ = cloud.data.data() + cloud.data.size();
                return it;
            }
            PointCloud2Iterator end() const {
                PointCloud2Iterator it = *this;
                it.ptr_ = const_cast<unsigned char*>(cloud_.data.data() + cloud_.data.size());
                return it;
            }
        private:
            const PointCloud2& cloud_;
            uint32_t offset_;
            unsigned char* ptr_;

            static uint32_t getFieldOffset(const PointCloud2& cloud, const std::string& field_name) {
                for (const auto& field : cloud.fields) {
                    if (field.name == field_name) {
                        return field.offset;
                    }
                }
                throw std::runtime_error("Field not found: " + field_name);
            }
    };
}




namespace geometry_msgs{
    class Pose2D {
        public:
            double x;
            double y;
            double theta;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["x"] = x;
                j["y"] = y;
                j["theta"] = theta;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Pose2D from_json(const nlohmann::json& j) {
                Pose2D pose;
                pose.x = j["x"];
                pose.y = j["y"];
                pose.theta = j["theta"];
                return pose;
            }
            static Pose2D from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    
    struct Point
    {
        double x;
        double y;
        double z;
    };
    
    class Pose {
        public:
            Point position;
            tf::Quaternion orientation;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["position"]["x"] = position.x;
                j["position"]["y"] = position.y;
                j["position"]["z"] = position.z;
                j["orientation"] = orientation.to_json();
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Pose from_json(const nlohmann::json& j) {
                Pose pose;
                pose.position.x = j["position"]["x"];
                pose.position.y = j["position"]["y"];
                pose.position.z = j["position"]["z"];
                pose.orientation = tf::Quaternion::from_json(j["orientation"]);
                return pose;
            }
            static Pose from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    class PoseWithCovariance{
        public:
            Pose pose;
            std::array<double, 36> covariance;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["pose"] = pose.to_json();
                j["covariance"] = std::vector<double>(covariance.begin(),covariance.end());;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static PoseWithCovariance from_json(const nlohmann::json& j) {
                PoseWithCovariance pose;
                pose.pose =  Pose::from_json(j["pose"]);
                std::vector<double> vec = j["covariance"].get<std::vector<double>>();
                if (vec.size() != 36) {
                    throw std::runtime_error("JSON array size is not 36");
                }
                std::copy(vec.begin(), vec.end(),pose.covariance.begin());
                return pose;
            }
            static PoseWithCovariance from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    class Twist {
        public:
            Twist(){
                linear.x = 0.1;
                linear.y = 0.2;
                linear.z = 0.3;
                angular.x = 0.4;
                angular.y = 0.5;
                angular.z = 0.6;
            }
            Vector3 linear;
            Vector3 angular;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["linear"]["x"] = linear.x;
                j["linear"]["y"] = linear.y;
                j["linear"]["z"] = linear.z;
                j["angular"]["x"] = angular.x;
                j["angular"]["y"] = angular.y;
                j["angular"]["z"] = angular.z;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Twist from_json(const nlohmann::json& j) {
                Twist twist;
                twist.linear.x = j["linear"]["x"];
                twist.linear.y =j["linear"]["y"];
                twist.linear.z =j["linear"]["z"];
                twist.angular.x =j["angular"]["x"];
                twist.angular.y =j["angular"]["y"];
                twist.angular.z =j["angular"]["z"];
                return twist;
            }
            static Twist from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    class TwistWithCovariance{
        public:
            TwistWithCovariance(){
                covariance = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,
                              0.11,0.22,0.33,0.44,0.55,0.66,0.77,0.88,0.99,
                              0.111,0.222,0.333,0.444,0.555,0.666,0.777,0.888,0.999,
                              0.1111,0.2222,0.3333,0.4444,0.5555,0.6666,0.7777,0.8888,0.9999
                };
            }
            Twist twist;
            std::array<double, 36> covariance;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["twist"] = twist.to_json();
                j["covariance"] = std::vector<double>(covariance.begin(),covariance.end());;
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static TwistWithCovariance from_json(const nlohmann::json& j) {
                TwistWithCovariance twist;
                twist.twist = Twist::from_json(j["twist"]);
                std::vector<double> vec = j["covariance"].get<std::vector<double>>();
                if (vec.size() != 36) {
                    throw std::runtime_error("JSON array size is not 36");
                }
                std::copy(vec.begin(), vec.end(),twist.covariance.begin());
                return twist;
            }
            static TwistWithCovariance from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
    
}
namespace nav_msgs{
    class Odometry{
        public:
            Odometry(){
                
            }
            Header header;
            std::string child_frame_id;
            geometry_msgs::PoseWithCovariance pose;
            geometry_msgs::TwistWithCovariance twist;
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["header"] = header.to_json();
                j["child_frame_id"] = child_frame_id;
                j["twist"] = twist.to_json();
                j["pose"] = twist.to_json();
                return j;
            }
            std::vector<unsigned char> to_vector() const {
                nlohmann::json j = to_json();
                std::string json_str = j.dump();
                return std::vector<unsigned char>(json_str.begin(), json_str.end());
            }
            static Odometry from_json(const nlohmann::json& j) {
                Odometry odom;
                odom.twist = geometry_msgs::TwistWithCovariance::from_json(j["twist"]);
                odom.pose = geometry_msgs::PoseWithCovariance::from_json(j["pose"]);
                odom.header = Header::from_json(j["header"]);
                odom.child_frame_id = j["child_frame_id"];
                return odom;
            }
            static Odometry from_vector(const std::vector<unsigned char>& vec) {
                std::string json_str(vec.begin(), vec.end());
                nlohmann::json j = nlohmann::json::parse(json_str);
                return from_json(j);
            }
    };
}
namespace pcl{
    struct PointXYZI
    {
        float x;
        float y;
        float z;
        uint8_t intensity;
    }; 

    struct PointXYZIRT
    {
        float x;
        float y;
        float z;
        float intensity;
        uint16_t ring;
        //double timestamp;
        float time;
    };

    template <typename T_Point>
    class PointCloud
    {
        public:
            typedef T_Point PointT;
            typedef std::vector<PointT> VectorT;

            uint32_t height = 0;    ///< Height of point cloud
            uint32_t width = 0;     ///< Width of point cloud
            bool is_dense = false;  ///< If is_dense is true, the point cloud does not contain NAN points,
            Header header;
            VectorT points;
            typedef boost::shared_ptr< ::pcl::PointCloud<T_Point>> Ptr;
    };
    template<typename PointT>
    void toROSMsg(const pcl::PointCloud<PointT>& cloud, sensor_msgs::PointCloud2& ros_msg) {
        // Set the header
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) - std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);
        ros_msg.header.stamp = Stamp(seconds.count(), nanoseconds.count());
        ros_msg.header.frame_id = cloud.header.frame_id;

        // Set the fields
        ros_msg.height = cloud.height;
        ros_msg.width = cloud.width;
        ros_msg.is_dense = cloud.is_dense;
        ros_msg.is_bigendian = false;

        // Set the point step and row step
        ros_msg.point_step = sizeof(PointT);
        ros_msg.row_step = ros_msg.point_step * cloud.width;

        // Set the fields
        sensor_msgs::PointCloud2Modifier modifier(ros_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        // Resize the data vector
        ros_msg.data.resize(cloud.points.size() * sizeof(PointT));

        // Copy the data
        sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");

        for (size_t i = 0; i < cloud.points.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = cloud.points[i].x;
            *iter_y = cloud.points[i].y;
            *iter_z = cloud.points[i].z;
        }
    }
}