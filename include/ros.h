#include <string>
#include <vector>
#include <cmath>
#include <arrow/api.h>
#include <vector>
#include <memory>

#include <array>

#include <nlohmann/json.hpp>
namespace tf{
    struct Quaternion {
        double x;
        double y;
        double z;
        double w;
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

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Stamp {
    long long secs;    // 秒
    long long nsecs;   // 纳秒
};
struct Header {
    unsigned int seq;          // 扫描顺序增加的id
    Stamp stamp;              // 开始扫描的时间和与开始扫描的时间差
    std::string frame_id;      // 扫描的参考系名称
};
namespace sensor_msgs {
    class LaserScan {
        public:
            LaserScan(){
                header.seq = 632;
                header.stamp.secs = 16;
                header.stamp.nsecs = 187000000;
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
            nlohmann::json to_json() const{
                nlohmann::json j;
                j["header"]["seq"] = header.seq;
                j["header"]["stamp"]["secs"] = header.stamp.secs;
                j["header"]["stamp"]["nsecs"] = header.stamp.nsecs;
                j["header"]["frame_id"] = header.frame_id;
                j["angle_min"] = angle_min;
                j["angle_max"] = angle_max;
                j["angle_increment"] = angle_increment;
                j["time_increment"] = time_increment;
                j["scan_time"] = scan_time;
                j["range_min"] = range_min;
                j["range_max"] = range_max;
                j["ranges"] = ranges;
                j["intensities"] = intensities;
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
                scan.header.stamp.secs = j["header"]["stamp"]["secs"];
                scan.header.stamp.nsecs = j["header"]["stamp"]["nsecs"];
                scan.header.frame_id = j["header"]["frame_id"];
                scan.angle_min = j["angle_min"];
                scan.angle_max = j["angle_max"];
                scan.angle_increment = j["angle_increment"];
                scan.time_increment = j["time_increment"];
                scan.scan_time = j["scan_time"];
                scan.range_min = j["range_min"];
                scan.range_max = j["range_max"];
                scan.ranges = j["ranges"].get<std::vector<float>>();
                scan.intensities = j["intensities"].get<std::vector<float>>();
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
        scan.header.stamp.secs = 16;
        scan.header.stamp.nsecs = 187000000;
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
    std::shared_ptr<arrow::RecordBatch> ConvertLaserScanToArrow(const LaserScan& scan) {
        arrow::MemoryPool* pool = arrow::default_memory_pool();
        arrow::Status status;
        // 定义Header字段
        auto seq_field = arrow::field("seq", arrow::uint32());
        auto sec_field = arrow::field("sec", arrow::int64());
        auto nsec_field = arrow::field("nsec", arrow::int64());
        auto frame_id_field = arrow::field("frame_id", arrow::utf8());

        // 定义LaserScan其他字段
        auto angle_min_field = arrow::field("angle_min", arrow::float32());
        auto angle_max_field = arrow::field("angle_max", arrow::float32());
        auto angle_increment_field = arrow::field("angle_increment", arrow::float32());
        auto time_increment_field = arrow::field("time_increment", arrow::float32());
        auto scan_time_field = arrow::field("scan_time", arrow::float32());
        auto range_min_field = arrow::field("range_min", arrow::float32());
        auto range_max_field = arrow::field("range_max", arrow::float32());

        auto ranges_field = arrow::field("ranges", arrow::list(arrow::float32()));
        auto intensities_field = arrow::field("intensities", arrow::list(arrow::float32()));

        // 创建Schema
        auto schema = arrow::schema({seq_field, sec_field, nsec_field, frame_id_field, 
        angle_min_field, angle_max_field,angle_increment_field,
        time_increment_field,scan_time_field,
        range_min_field, range_max_field , ranges_field, intensities_field});

        // 创建Builders
        arrow::UInt32Builder seq_builder(pool);
        arrow::Int64Builder sec_builder(pool);
        arrow::Int64Builder nsec_builder(pool);
        arrow::StringBuilder frame_id_builder(pool);

        arrow::FloatBuilder angle_min_builder(pool);
        arrow::FloatBuilder angle_max_builder(pool);
        arrow::FloatBuilder angle_increment_builder(pool);
        arrow::FloatBuilder time_increment_builder(pool);
        arrow::FloatBuilder scan_time_builder(pool);
        arrow::FloatBuilder range_min_builder(pool);
        arrow::FloatBuilder range_max_builder(pool);

        arrow::ListBuilder ranges_builder(pool, std::make_shared<arrow::FloatBuilder>(pool));
        arrow::ListBuilder intensities_builder(pool, std::make_shared<arrow::FloatBuilder>(pool));

        // 填充Header数据
        if(seq_builder.Append(scan.header.seq).ok() == false) {
            return nullptr;
        }
        if(sec_builder.Append(scan.header.stamp.secs).ok() == false) {
            return nullptr;
        }
        if(nsec_builder.Append(scan.header.stamp.nsecs).ok() == false) {
            return nullptr;
        }
        if(frame_id_builder.Append(scan.header.frame_id).ok() == false) {
            return nullptr;
        }

        // 填充LaserScan数据
        if(angle_min_builder.Append(scan.angle_min).ok() == false) {
            return nullptr;
        }
        if(angle_max_builder.Append(scan.angle_max).ok() == false) {
            return nullptr;
        }
        if(angle_increment_builder.Append(scan.angle_increment).ok() == false) {
            return nullptr;
        }
        if(time_increment_builder.Append(scan.time_increment).ok() == false) {
            return nullptr;
        }
        if(scan_time_builder.Append(scan.scan_time).ok() == false) {
            return nullptr;
        }
        if(range_min_builder.Append(scan.range_min).ok() == false) {
            return nullptr;
        }
        if(range_max_builder.Append(scan.range_max).ok() == false) {
            return nullptr;
        }
        int tot =0 ;
        for (float range : scan.ranges) {
            tot++;printf("%f @\n",range);
            if(static_cast<arrow::FloatBuilder*>(ranges_builder.value_builder())->Append(range).ok() == false) {
                return nullptr;
            }
        }
        
        printf("tot: %d length:%d\n", tot,static_cast<arrow::FloatBuilder*>(ranges_builder.value_builder())->length());
        if(ranges_builder.Append(true).ok() == false){return nullptr;} // 假设每个LaserScan只有一个ranges数组

        for (float intensity : scan.intensities) {
            if(static_cast<arrow::FloatBuilder*>(intensities_builder.value_builder())->Append(intensity).ok() == false) {
                return nullptr;
            }
        }
        if(intensities_builder.Append(true).ok() == false){return nullptr;} // 假设每个LaserScan只有一个intensities数组
        // 创建RecordBatch
        int64_t num_rows = 1;
        auto batch = arrow::RecordBatch::Make(schema, num_rows, 
        {seq_builder.Finish().ValueOrDie(), 
        sec_builder.Finish().ValueOrDie(), 
        nsec_builder.Finish().ValueOrDie(),
        frame_id_builder.Finish().ValueOrDie(), 
        angle_min_builder.Finish().ValueOrDie(), 
        angle_max_builder.Finish().ValueOrDie(), 
        angle_increment_builder.Finish().ValueOrDie(), 
        time_increment_builder.Finish().ValueOrDie(), 
        scan_time_builder.Finish().ValueOrDie(), 
        range_min_builder.Finish().ValueOrDie(), 
        range_max_builder.Finish().ValueOrDie(),
        ranges_builder.Finish().ValueOrDie(), 
        intensities_builder.Finish().ValueOrDie()});
        return batch;
    }
    LaserScan ConvertArrowToLaserScan(std::shared_ptr<arrow::RecordBatch> batch) {
        LaserScan laserData;
        laserData.header.seq = std::static_pointer_cast<arrow::UInt32Array>(batch->column(0))->Value(0);
        laserData.header.stamp.secs = std::static_pointer_cast<arrow::DoubleArray>(batch->column(1))->Value(0);
        laserData.header.stamp.nsecs = std::static_pointer_cast<arrow::DoubleArray>(batch->column(2))->Value(0);
        laserData.header.frame_id = std::static_pointer_cast<arrow::StringArray>(batch->column(3))->GetString(0);

        laserData.angle_min = std::static_pointer_cast<arrow::FloatArray>(batch->column(4))->Value(0);
        laserData.angle_max = std::static_pointer_cast<arrow::FloatArray>(batch->column(5))->Value(0);
        laserData.angle_increment = std::static_pointer_cast<arrow::FloatArray>(batch->column(6))->Value(0);

        laserData.time_increment = std::static_pointer_cast<arrow::FloatArray>(batch->column(7))->Value(0);
        laserData.scan_time = std::static_pointer_cast<arrow::FloatArray>(batch->column(8))->Value(0);

        laserData.range_min = std::static_pointer_cast<arrow::FloatArray>(batch->column(9))->Value(0);
        laserData.range_max = std::static_pointer_cast<arrow::FloatArray>(batch->column(10))->Value(0);

        auto rangesArray = std::static_pointer_cast<arrow::ListArray>(batch->column(11));
        printf("%d \n",rangesArray->length());
        for(int i=0;i<rangesArray->length();++i){
            auto slice = rangesArray->value_slice(i-1);
            auto float_array = std::static_pointer_cast<arrow::FloatArray>(slice);
            //printf("%d %d \n",i-1,float_array->length());
            for (int j = 0; j < float_array->length(); ++j) {
                laserData.ranges.push_back(float_array->Value(j));
                //printf("%f ",float_array->Value(j));
            }
        }
        
        auto intensitiesArray = std::static_pointer_cast<arrow::ListArray>(batch->column(12));
        for(int i=0;i<intensitiesArray->length();++i){
            auto slice = intensitiesArray->value_slice(i-1);
            auto float_array = std::static_pointer_cast<arrow::FloatArray>(slice);
            for (int j = 0; j < float_array->length(); ++j) {
                laserData.intensities.push_back(float_array->Value(j));
            }
        }
        return laserData;
    }
    void print(LaserScan scan) {
        printf("seq: %d\n", scan.header.seq);
        printf("stamp: %lld.%lld\n", scan.header.stamp.secs, scan.header.stamp.nsecs);
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
            Imu(){
                header.seq = 50;
                header.stamp.secs = 17;
                header.stamp.nsecs = 497000000;
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
                j["header"]["stamp"]["secs"] = header.stamp.secs;
                j["header"]["stamp"]["nsecs"] = header.stamp.nsecs;
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
                imu.header.stamp.secs = j["header"]["stamp"]["secs"];
                imu.header.stamp.nsecs = j["header"]["stamp"]["nsecs"];
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
    void print(Imu imu){
        printf("seq: %d\n", imu.header.seq);
        printf("stamp: %lld.%lld\n", imu.header.stamp.secs, imu.header.stamp.nsecs);
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
    void print(Pose2D pose){
        printf("x:%f y:%f z:%f\n",pose.x,pose.y,pose.theta);
    }
    /*
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/Pose pose
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
    float64[36] covariance
    */
    class Twist {
        public:
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
}
namespace nav_msgs{
    class Odometry{
        public:
            Header header;
            std::string child_frame_id;
            
            nav_msgs::Odometry
    }
}

/*

Header header
    uint32 seq
    time stamp
    string frame_id
string child_frame_id

geometry_msgs/TwistWithCovariance twist
    geometry_msgs/Twist twist
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
    float64[36] covariance
*/