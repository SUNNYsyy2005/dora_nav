#ifndef ROS_H
#define ROS_H

#include <string>
#include <vector>
#include <cmath>
#include <array>
#include <memory>
#include <nlohmann/json.hpp>

class Header {
public:
    uint32_t seq;          // 扫描顺序增加的id
    struct Stamp {
        long long sec;    // 秒
        long long nsec;   // 纳秒
        nlohmann::json to_json() const {                    
            nlohmann::json j;
            j["sec"] = sec;
            j["nsec"] = nsec;
            return j;
        }
        static Stamp from_json(const nlohmann::json& j) {
            Stamp stamp;
            stamp.sec = j["sec"];
            stamp.nsec = j["nsec"];
            return stamp;
        }
    } stamp;              // 开始扫描的时间和与开始扫描的时间差
    std::string frame_id;      // 扫描的参考系名称

    nlohmann::json to_json() const {
        nlohmann::json j;
        j["seq"] = seq;
        j["stamp"] = stamp.to_json();
        j["frame_id"] = frame_id;
        return j;
    }

    static Header from_json(const nlohmann::json& j) {
        Header header;
        header.seq = j["seq"];
        header.stamp = Stamp::from_json(j["stamp"]);
        header.frame_id = j["frame_id"];
        return header;
    }
};

namespace tf {
    class Quaternion {
    public:
        Quaternion(double x, double y, double z, double w) {
            this->x = x;
            this->y = y;
            this->z = z;
            this->w = w;
        }
        Quaternion() {
            this->x = this->y = this->z = this->w = 0;
        }
        double x;
        double y;
        double z;
        double w;
        nlohmann::json to_json() const {
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
            quaternion.y = j["y"];
            quaternion.z = j["z"];
            quaternion.w = j["w"];
            return quaternion;
        }
        static Quaternion from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
    };

    Quaternion newQuaternion(double x, double y, double z, double w) {
        Quaternion q;
        q.x = x;
        q.y = y;
        q.z = z;
        q.w = w;
        return q;
    }

    void getRPY(const Quaternion& m, double& roll, double& pitch, double& yaw) {
        const double epsilon = 1e-6;
        const double pi = 3.14159265358979323846;

        double sqx = m.x * m.x;
        double sqy = m.y * m.y;
        double sqz = m.z * m.z;

        // roll (x-axis rotation)
        double t0 = +2.0 * (m.w * m.x + m.y * m.z);
        double t1 = +1.0 - 2.0 * (sqx + sqy);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (m.w * m.y - m.z * m.x);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (m.w * m.z + m.x * m.y);
        double t4 = +1.0 - 2.0 * (sqy + sqz);
        yaw = std::atan2(t3, t4);
    }
}

class Vector3 {
public:
    double x;
    double y;
    double z;
    nlohmann::json to_json() const {
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
        vector3.y = j["y"];
        vector3.z = j["z"];
        return vector3;
    }
    static Vector3 from_vector(const std::vector<unsigned char>& vec) {
        std::string json_str(vec.begin(), vec.end());
        nlohmann::json j = nlohmann::json::parse(json_str);
        return from_json(j);
    }
};

namespace sensor_msgs {
    class LaserScan {
    public:
        LaserScan() {
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
        nlohmann::json to_json() const {
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

    LaserScan newLaserScan() {
        LaserScan scan;
        scan.header.seq = 632;
        scan.header.stamp.sec = 16;
        scan.header.stamp.nsec = 187000000;
        scan.header.frame_id = "laser_frame";
        scan.angle_min = -2.3561899662017822;
        scan.angle_max = 2.3561899662017822;
        scan.angle_increment = 0.004363314714282751;
        scan.time_increment = 0.0;
        scan.scan_time = 0.0;
        scan.range_min = 0.10000000149011612;
        scan.range_max = 10.0;
        scan.ranges = {2.551586389541626, 2.5835444927215576, 2.5715863704681396, 2.5952813625335693, 2.60617995262146};
        scan.intensities = {1.0, 2.0, 3.0, 4.0, 5.0};
        return scan;
    }

    void print(const LaserScan& scan) {
        printf("seq: %d\n", scan.header.seq);
        printf("stamp: %lld.%lld\n", scan.header.stamp.sec, scan.header.stamp.nsec);
        printf("frame_id: %s\n", scan.header.frame_id.c_str());
        printf("angle_min: %f\n", scan.angle_min);
        printf("angle_max: %f\n", scan.angle_max);
        printf("angle_increment: %f\n", scan.angle_increment);
        printf("time_increment: %f\n", scan.time_increment);
        printf("scan_time: %f\n", scan.scan_time);
        printf("range_min: %f\n", scan.range_min);
        printf("range_max: %f\n", scan.range_max);
        printf("ranges: ");
        for (float range : scan.ranges) {
            printf("%f ", range);
        }
        printf("\n");
        printf("intensities: ");
        for (float intensity : scan.intensities) {
            printf("%f ", intensity);
        }
        printf("\n");
    }

    class Imu {
    public:
        Imu() {
            header.seq = 50;
            header.stamp.sec = 17;
            header.stamp.nsec = 497000000;
            header.frame_id = "/imu_link";
            orientation.x = -1.4817208738488272e-05;
            orientation.y = 1.747766491371163e-05;
            orientation.z = 0.6963470506532196;
            orientation.w = 0.7177052211887164;
            orientation_covariance = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
            angular_velocity.x = 0.000868052236941635;
            angular_velocity.y = -0.000284583075071133;
            angular_velocity.z = 0.0018242036489127952;
            angular_velocity_covariance = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
            linear_acceleration.x = -0.2915581620976377;
            linear_acceleration.y = 0.03977790630637389;
            linear_acceleration.z = 9.805478448950305;
            linear_acceleration_covariance = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
        }

        Header header;

        tf::Quaternion orientation;
        std::array<double, 9> orientation_covariance; // 关于x, y, z轴的方向协方差，行优先

        Vector3 angular_velocity;
        std::array<double, 9> angular_velocity_covariance; // 关于x, y, z轴的角速度协方差，行优先

        Vector3 linear_acceleration;
        std::array<double, 9> linear_acceleration_covariance; // 关于x, y, z轴的线性加速度协方差，行优先

        nlohmann::json to_json() const {
            nlohmann::json j;
            j["header"]["seq"] = header.seq;
            j["header"]["stamp"]["sec"] = header.stamp.sec;
            j["header"]["stamp"]["nsec"] = header.stamp.nsec;
            j["header"]["frame_id"] = header.frame_id;
            j["orientation"]["x"] = orientation.x;
            j["orientation"]["y"] = orientation.y;
            j["orientation"]["z"] = orientation.z;
            j["orientation"]["w"] = orientation.w;
            j["orientation_covariance"] = std::vector<double>(orientation_covariance.begin(), orientation_covariance.end());
            j["angular_velocity"]["x"] = angular_velocity.x;
            j["angular_velocity"]["y"] = angular_velocity.y;
            j["angular_velocity"]["z"] = angular_velocity.z;
            j["angular_velocity_covariance"] = std::vector<double>(angular_velocity_covariance.begin(), angular_velocity_covariance.end());
            j["linear_acceleration"]["x"] = linear_acceleration.x;
            j["linear_acceleration"]["y"] = linear_acceleration.y;
            j["linear_acceleration"]["z"] = linear_acceleration.z;
            j["linear_acceleration_covariance"] = std::vector<double>(linear_acceleration_covariance.begin(), linear_acceleration_covariance.end());
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
            std::copy(vec.begin(), vec.end(), imu.orientation_covariance.begin());
            imu.angular_velocity.x = j["angular_velocity"]["x"];
            imu.angular_velocity.y = j["angular_velocity"]["y"];
            imu.angular_velocity.z = j["angular_velocity"]["z"];
            vec = j["angular_velocity_covariance"].get<std::vector<double>>();
            if (vec.size() != 9) {
                throw std::runtime_error("JSON array size is not 9");
            }
            std::copy(vec.begin(), vec.end(), imu.angular_velocity_covariance.begin());
            imu.linear_acceleration.x = j["linear_acceleration"]["x"];
            imu.linear_acceleration.y = j["linear_acceleration"]["y"];
            imu.linear_acceleration.z = j["linear_acceleration"]["z"];
            vec = j["linear_acceleration_covariance"].get<std::vector<double>>();
            if (vec.size() != 9) {
                throw std::runtime_error("JSON array size is not 9");
            }
            std::copy(vec.begin(), vec.end(), imu.linear_acceleration_covariance.begin());
            return imu;
        }

        static Imu from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
    };

    void print(const Imu& imu) {
        printf("seq: %d\n", imu.header.seq);
        printf("stamp: %lld.%lld\n", imu.header.stamp.sec, imu.header.stamp.nsec);
        printf("frame_id: %s\n", imu.header.frame_id.c_str());
        printf("orientation: x:%f y:%f z:%f w:%f\n", imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
        printf("orientation_covariance: ");
        for (double cov : imu.orientation_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
        printf("angular_velocity: x:%f y:%f z:%f\n", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        printf("angular_velocity_covariance: ");
        for (double cov : imu.angular_velocity_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
        printf("linear_acceleration: x:%f y:%f z:%f\n", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        printf("linear_acceleration_covariance: ");
        for (double cov : imu.linear_acceleration_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
    }
}

namespace geometry_msgs {
    class Pose2D {
    public:
        double x;
        double y;
        double theta;
        nlohmann::json to_json() const {
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

    void print(const Pose2D& pose) {
        printf("x:%f y:%f theta:%f\n", pose.x, pose.y, pose.theta);
    }

    struct Point {
        double x;
        double y;       
        double z;
    };

    class Pose {
    public:
        Point position;
        tf::Quaternion orientation;
        nlohmann::json to_json() const {
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

    class PoseWithCovariance {
    public:
        Pose pose;
        std::array<double, 36> covariance;
        nlohmann::json to_json() const {
            nlohmann::json j;
            j["pose"] = pose.to_json();
            j["covariance"] = std::vector<double>(covariance.begin(), covariance.end());
            return j;
        }
        std::vector<unsigned char> to_vector() const {
            nlohmann::json j = to_json();
            std::string json_str = j.dump();
            return std::vector<unsigned char>(json_str.begin(), json_str.end());
        }
        static PoseWithCovariance from_json(const nlohmann::json& j) {
            PoseWithCovariance pose;
            pose.pose = Pose::from_json(j["pose"]);
            std::vector<double> vec = j["covariance"].get<std::vector<double>>();
            if (vec.size() != 36) {
                throw std::runtime_error("JSON array size is not 36");
            }
            std::copy(vec.begin(), vec.end(), pose.covariance.begin());
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
        Twist() {
            linear.x = 0.1;
            linear.y = 0.2;
            linear.z = 0.3;
            angular.x = 0.4;
            angular.y = 0.5;
            angular.z = 0.6;
        }
        Vector3 linear;
        Vector3 angular;
        nlohmann::json to_json() const {
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
            twist.linear.y = j["linear"]["y"];
            twist.linear.z = j["linear"]["z"];
            twist.angular.x = j["angular"]["x"];
            twist.angular.y = j["angular"]["y"];
            twist.angular.z = j["angular"]["z"];
            return twist;
        }
        static Twist from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
    };

    class TwistWithCovariance {
    public:
        TwistWithCovariance() {
            covariance = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
                          0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77, 0.88, 0.99,
                          0.111, 0.222, 0.333, 0.444, 0.555, 0.666, 0.777, 0.888, 0.999,
                          0.1111, 0.2222, 0.3333, 0.4444, 0.5555, 0.6666, 0.7777, 0.8888, 0.9999};
        }
        Twist twist;
        std::array<double, 36> covariance;
        nlohmann::json to_json() const {
            nlohmann::json j;
            j["twist"] = twist.to_json();
            j["covariance"] = std::vector<double>(covariance.begin(), covariance.end());
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
            std::copy(vec.begin(), vec.end(), twist.covariance.begin());
            return twist;
        }
        static TwistWithCovariance from_vector(const std::vector<unsigned char>& vec) {
            std::string json_str(vec.begin(), vec.end());
            nlohmann::json j = nlohmann::json::parse(json_str);
            return from_json(j);
        }
    };

    void print(const TwistWithCovariance& twist) {
        printf("linear.x:%f  linear.y:%f  linear.z:%f\n", twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
        printf("angular.x:%f  angular.y:%f  angular.z:%f\n", twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);
        for (double cov : twist.covariance) {
            printf("%f ", cov);
        }
        printf("\n");
    }
}

namespace nav_msgs {
    class Odometry {
    public:
        Odometry() {
        }
        Header header;
        std::string child_frame_id;
        geometry_msgs::PoseWithCovariance pose;
        geometry_msgs::TwistWithCovariance twist;
        nlohmann::json to_json() const {
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

#endif // ROS_H
