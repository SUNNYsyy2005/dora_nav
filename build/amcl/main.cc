extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>
#include "../../include/ros.h"
#include <chrono>
#include <cmath>
#include <algorithm>

#include "include/map/map.h"
#include "include/sensors/amcl_laser.h"
#include "include/sensors/amcl_odom.h"


#define M_PI 3.14159265358979323846
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
double global_x = 0.0; // x坐标
double global_y = 0.0; // y坐标
double global_theta = M_PI/2; // 角度
double last_theta = M_PI/2;
double steer_theta = 0.0;
double last_velocity = 0.0;
double last_steering_angle_velocity = 0.0;
geometry_msgs::Pose2D msg2;
std::chrono::steady_clock::time_point last_update_time = std::chrono::steady_clock::now();

pf_t *pf;
map_t *map;
amcl::AMCLLaser laser_sensor(10, NULL);
amcl::AMCLOdom odom_sensor;
amcl::AMCLLaserData laser_data;
amcl::AMCLOdomData odom_data;
void replace_null_with_nan(std::string& json_str) {
    std::string null_str = "null";
    std::string nan_str = "-1";
    size_t pos = 0;
    while ((pos = json_str.find(null_str, pos)) != std::string::npos) {
        json_str.replace(pos, null_str.length(), nan_str);
        pos += nan_str.length();
    }
}
void updateParticlePoses() {
    // 确保有足够的位移或转向变化再更新
    odom_data.delta.v[0] = global_x;
    odom_data.delta.v[1] = global_y;
    odom_data.delta.v[2] = global_theta-last_theta;
    odom_data.pose.v[0] = 0;
    odom_data.pose.v[1] = 0;
    odom_data.pose.v[2] = global_theta;
    odom_sensor.UpdateAction(pf,&odom_data);
    pf_sample_set_t *set = pf->sets + pf->current_set;
    update_kdtree(set);
    pf_cluster_stats(pf, set);
    global_x = 0;
    global_y = 0;
    last_theta = global_theta;
}

void imuCallback(const sensor_msgs::Imu * msg) {
    // 获取四元数姿态
    tf::Quaternion q =tf::newQuaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double roll, pitch, yaw;
    tf::getRPY(q,roll, pitch, yaw); // 将四元数转换为欧拉角
    global_theta = yaw;// 小车的朝向
}
double toPI(double angle){
    while(angle > M_PI){
        angle -= 2*M_PI;
    }
    while(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}
void process_samples(pf_t *pf) {
    int i;
    double max_weight = 0.0;
    pf_cluster_t *max_cluster = NULL;
    pf_sample_set_t *set = pf->sets + pf->current_set;

    // 遍历所有聚类
    for (i = 0; i < set->cluster_count; i++) {
        pf_cluster_t *cluster = set->clusters + i;
        double weight;
        pf_vector_t mean;
        pf_matrix_t cov;

        // 获取当前聚类的统计数据
        if (pf_get_cluster_stats(pf, i, &weight, &mean, &cov)) {
            // 检查是否是遇到的最大权重聚类
            if (weight > max_weight) {
                max_weight = weight;
                max_cluster = cluster;
            }
        }
    }
    // 如果找到了权重最大的聚类，输出其平均值
    if (max_cluster != NULL) {
        printf("Max weight cluster weight: %f\n", max_weight);
        msg2.x = MAP_GXWX(map,max_cluster->mean.v[0]);
        msg2.y = MAP_GYWY(map,max_cluster->mean.v[1]);
        msg2.theta = max_cluster->mean.v[2];
    } else {
        printf("No clusters found.\n");
    }
    // 输出平均位置
    printf("Average pose: %f %f %f global theta%f\n",msg2.x,msg2.y,msg2.theta,global_theta);
}
void laserCallback(const sensor_msgs::LaserScan* msg) {
    auto now = std::chrono::steady_clock::now();
    // 检查自上次回调以来是否已经过了5秒
    if (std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count() < 200000) {
        // 如果没有过5秒，就直接返回，不处理这次消息
        return;
    }
    updateParticlePoses();
    // 更新上次处理消息的时间
    last_time = now;
    laser_data.ranges.resize(msg->ranges.size(), std::vector<double>(2)); 
    laser_data.range_count = msg->ranges.size();
    laser_data.range_max = msg->range_max;
    double range_min = msg->range_min;
    double angle_increment = msg->angle_increment;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        laser_data.ranges[i][0] = msg->ranges[i];
        laser_data.ranges[i][1] = msg->angle_min + i * angle_increment;
        //printf("range: %f angle: %f\n",laser_data.ranges[i][0]/0.05,laser_data.ranges[i][1]);
    }
    
    laser_sensor.UpdateSensor(pf, &laser_data);
    pf_sample_set_t *set = pf->sets + pf->current_set;
    pf_cluster_stats(pf, set);
    //std::cout<<"Update sensor\n";
    process_samples(pf);
}
void ackermannCmdCallback(const geometry_msgs::Twist* msg) {
    //print(msg);
    double velocity =  last_velocity; // 车辆速度
    double steering_angle_velocity = last_steering_angle_velocity; // 转向角度
   // std::cout<<velocity<<" "<<steering_angle_velocity<<std::endl;
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_time).count()/1000000.0;
    last_update_time = now;
    last_steering_angle_velocity = msg->angular.z;
    last_velocity = msg->linear.x;
    global_x += velocity * dt * cos(global_theta);
    global_y -= velocity * dt * sin(global_theta);
    if (std::isnan(global_x)){
        std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
        exit(0);
    }
}
pf_vector_t random_pose_init(void *data) {
    pf_vector_t pose;
    // 这里假设一个均匀分布，实际应用中可能需要根据实际情况调整
    pose.v[0] = (double) rand() / RAND_MAX; // x 坐标
    pose.v[1] = (double) rand() / RAND_MAX; // y 坐标
    pose.v[2] = (double) rand() / RAND_MAX * 2 * M_PI - M_PI; // 角度，从 -π 到 π
    printf("Random pose: (%f, %f, %f)\n", pose.v[0], pose.v[1], pose.v[2]);
    return pose;
}
int run(void *dora_context)
{
    unsigned char counter = 0;
    msg2.x=1000;msg2.y=1000;msg2.theta=0;
    map = map_alloc();
    map_load_occ(map, "/home/sunny/dora_nav/build/amcl/map/1.pgm", 0.04,0);

    printf("map size: %d %d\n", map->size_x, map->size_y);
    // 设置AMCL的激光雷达传感器模型
    amcl::AMCLLaser aa((size_t)1080, map);
    laser_sensor = aa;
    pf_vector_t v;
    v.v[0]=0;v.v[1]=0;v.v[2]=0;
    laser_sensor.SetLaserPose(v); // 设置激光雷达在机器人坐标系中的位置
    laser_sensor.SetModelLikelihoodField(0.99, 0.01, 0.1, 10); // 激光模型参数
    laser_data.sensor = &laser_sensor;

    // 创建粒子滤波器
    int min_samples = 100;      // 最小粒子数
    int max_samples = 1000;    // 最大粒子数
    double alpha_slow = 0.1;   // 慢衰减率
    double alpha_fast = 0.1;   // 快衰减率
    pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                        random_pose_init, &laser_data);
    pf->selective_resampling = 1;
    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return -1;
    }
    // 初始均值和协方差矩阵
    pf_vector_t mean = {0, 0, 0}; // 初始均值 [x, y, theta]
    pf_matrix_t cov = {1, 0, 0, 0, 1, 0 , 0, 0, M_PI*M_PI}; // 初始协方差
    // 使用高斯模型初始化粒子滤波器
    pf_init(pf, mean, cov);

    while(true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            continue;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        printf("[c node] received event: %d\n", ty);

        if (ty == DoraEventType_Input)
        {
            counter += 1;

            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            printf("id: %s\n", id.c_str());
            if(id == "tick")
            {
                std::string out_id = "pose";
                nlohmann::json json_obj = msg2.to_json();
                std::string json_str = json_obj.dump();
                printf("%s\n", json_str.c_str());
                const char* char_ptr = json_str.c_str();
                char* non_const_char_ptr = new char[json_str.size() + 1];
                std::memcpy(non_const_char_ptr, char_ptr, json_str.size() + 1);
                int result = dora_send_output(dora_context, &out_id[0], out_id.length(), reinterpret_cast<char*>(non_const_char_ptr), json_str.size());
                if (result != 0)
                {
                    std::cerr << "failed to send output" << std::endl;
                    return 1;
                }
                printf("tick\n");
            }else if(id == "scan2"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("json_str: %s\n", json_str.c_str());
                replace_null_with_nan(json_str);
                printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                sensor_msgs::LaserScan scan = sensor_msgs::LaserScan::from_json(json_obj);
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
                printf("\n  ");
                laserCallback(&scan);
            }else if(id == "imu"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                //std::vector<unsigned char> data;
                //for (size_t i = 0; i < data_len; i++)
                //{
                //    data.push_back(*(data_ptr + i));
                //}
                //sensor_msgs::Imu imu = sensor_msgs::Imu::from_vector(data);
                //imuCallback(&imu);
            }else if(id == "twist"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("json_str: %s\n", json_str.c_str());
                replace_null_with_nan(json_str);
                printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                geometry_msgs::Twist twist = geometry_msgs::Twist::from_json(json_obj);
                printf("linear x: %f\n", twist.linear.x);
                printf("angular z: %f\n", twist.angular.z);
                //std::vector<unsigned char> data;
                //for (size_t i = 0; i < data_len; i++)
                //{
                //    data.push_back(*(data_ptr + i));
                //}
                //geometry_msgs::Twist twist = geometry_msgs::Twist::from_vector(data);
                //ackermannCmdCallback(&twist);
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
    map_free(map);
    std::cout << "AMCL TEST MISC" << std::endl;
    return 0;
}

int main()
{
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
