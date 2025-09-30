extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>
#include "../../include/ros.h"
#include "../../include/project_paths.h"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

#include "include/map/map.h"
#include "include/sensors/amcl_laser.h"
#include "include/sensors/amcl_odom.h"

FILE * file;
cv::Mat map_image;

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
sensor_msgs::LaserScan scan;
sensor_msgs::Imu imu;
geometry_msgs::Twist twist;
std::chrono::steady_clock::time_point last_update_time = std::chrono::steady_clock::now();

pf_t *pf;
map_t *map;
amcl::AMCLLaser laser_sensor(10, NULL);
amcl::AMCLOdom odom_sensor;
amcl::AMCLLaserData laser_data;
amcl::AMCLOdomData odom_data;

std::mutex laser_mutex, imu_mutex, ackermann_mutex;

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
    // 计算自上次更新以来的位移增量
    double delta_x = global_x;
    double delta_y = global_y;
    double delta_theta = global_theta - last_theta;
    
    printf("AMCL粒子位姿更新: delta_x=%.6f, delta_y=%.6f, delta_theta=%.6f\n", 
           delta_x, delta_y, delta_theta);
    
    // 设置里程计数据
    odom_data.delta.v[0] = delta_x;
    odom_data.delta.v[1] = delta_y;
    odom_data.delta.v[2] = delta_theta;
    odom_data.pose.v[0] = 0;
    odom_data.pose.v[1] = 0;
    odom_data.pose.v[2] = global_theta;
    
    // 更新粒子滤波器
    odom_sensor.UpdateAction(pf, &odom_data);
    pf_sample_set_t *set = pf->sets + pf->current_set;
    update_kdtree(set);
    pf_cluster_stats(pf, set);
    
    // 重置累积位移（已经用于更新）
    global_x = 0;
    global_y = 0;
    last_theta = global_theta;
}
// 角度归一化函数
double toPI(double angle){
    while(angle > M_PI){
        angle -= 2*M_PI;
    }
    while(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

bool ifimu = false;
double imu_yaw_in_odom = 0.0;  // IMU在odom坐标系中的yaw角
double map_to_odom_yaw_offset = 0.0;  // map坐标系相对于odom坐标系的旋转偏移

void imuCallback(const sensor_msgs::Imu * msg) {
    // 获取四元数姿态
    tf::Quaternion q =tf::newQuaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double roll, pitch, yaw;
    tf::getRPY(q,roll, pitch, yaw); // 将四元数转换为欧拉角
    fprintf(file,"msg:\t x:%g \t y:%g \t z:%g \t w:%g \n",msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    fprintf(file,"yaw:%f\n", yaw);
    
    // 保存IMU在odom坐标系中的yaw角
    imu_yaw_in_odom = yaw;
    
    // 计算在map坐标系中的角度
    // global_theta(map) = imu_yaw(odom) + map_to_odom_offset
    global_theta = imu_yaw_in_odom + map_to_odom_yaw_offset;
    
    // 归一化到[-π, π]
    global_theta = toPI(global_theta);
    
    if(!ifimu){
        ifimu = true;
        printf("AMCL IMU初始化: imu_yaw_odom=%.3f, map_to_odom_offset=%.3f, global_theta=%.3f\n", 
               imu_yaw_in_odom, map_to_odom_yaw_offset, global_theta);
    }
    else{
        fprintf(file,"AMCL IMU角度更新: imu_yaw_odom=%.3f, map_to_odom_offset=%.3f, global_theta=%.3f\n", 
                imu_yaw_in_odom, map_to_odom_yaw_offset, global_theta);
    }
}
void process_samples(pf_t *pf) {
    int i;
    double max_weight = 0.0;
    pf_cluster_t *max_cluster = NULL;
    pf_sample_set_t *set = pf->sets + pf->current_set;

    // 遍历所有聚类
    printf("AMCL聚类统计: 总聚类数=%d, 总粒子数=%d\n", set->cluster_count, set->sample_count);
    
    // 输出前几个粒子的信息
    printf("AMCL前5个粒子位姿: ");
    for (i = 0; i < std::min(5, (int)set->sample_count); i++) {
        pf_sample_t *sample = set->samples + i;
        printf("(%.3f,%.3f,%.3f,w=%.6f) ", sample->pose.v[0], sample->pose.v[1], sample->pose.v[2], sample->weight);
    }
    printf("\n");
    
    for (i = 0; i < set->cluster_count; i++) {
        pf_cluster_t *cluster = set->clusters + i;
        double weight;
        pf_vector_t mean;
        pf_matrix_t cov;

        // 获取当前聚类的统计数据
        if (pf_get_cluster_stats(pf, i, &weight, &mean, &cov)) {
            printf("AMCL聚类[%d]: weight=%.6f, mean=(%.3f,%.3f,%.3f), global_theta=%.3f, diff=%.3f\n", 
                   i, weight, mean.v[0], mean.v[1], mean.v[2], global_theta, abs(mean.v[2]-global_theta));
            // 检查是否是遇到的最大权重聚类
            if (weight > max_weight) {
                max_weight = weight;
                max_cluster = cluster;
            }
        }
    }
    // 如果找到了权重最大的聚类，输出其平均值
    if (max_cluster != NULL && abs(max_cluster->mean.v[2]-global_theta)<M_PI) {
        //printf("Max weight cluster weight: %f\n", max_weight);
        msg2.x = max_cluster->mean.v[0];
        msg2.y = max_cluster->mean.v[1];
        msg2.theta = max_cluster->mean.v[2];
        printf("AMCL使用聚类位姿: x=%.3f, y=%.3f, theta=%.3f\n", msg2.x, msg2.y, msg2.theta);
    } else {
        printf("No valid clusters found, keeping last pose.\n");
        // 当没有找到有效聚类时，保持上次的有效位姿不变
        printf("AMCL保持上次位姿: x=%.3f, y=%.3f, theta=%.3f\n", msg2.x, msg2.y, msg2.theta);
    }
    // 输出平均位置
    fprintf(file,"Average pose: %f %f %f global theta%f\n",msg2.x,msg2.y,msg2.theta,global_theta);
    //msg2.theta = global_theta;
}
void laserCallback(const sensor_msgs::LaserScan* msg) {     
    auto now = std::chrono::steady_clock::now();
    // 检查自上次回调以来是否已经过了5秒
    //if (std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count() < 200000) {
        // 如果没有过5秒，就直接返回，不处理这次消息
    //    return;
    //}
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
    
    printf("AMCL激光传感器更新: range_count=%d, range_max=%.3f, 前5个距离: ", 
           laser_data.range_count, laser_data.range_max);
    for(int i=0; i<std::min(5, (int)laser_data.range_count); i++) {
        printf("%.3f ", laser_data.ranges[i][0]);
    }
    printf("\n");
    
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
    //if (std::isnan(global_x)){
    //    std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
    //    exit(0);
    //}
}
pf_vector_t random_pose_init(void *data) {
    pf_vector_t pose;
    // 在地图范围内随机初始化粒子
    // 地图范围：-2m 到 +2m (4m x 4m 地图，原点在中心)
    pose.v[0] = ((double) rand() / RAND_MAX) * 4.0 - 2.0; // x 坐标 [-2, 2]
    pose.v[1] = ((double) rand() / RAND_MAX) * 4.0 - 2.0; // y 坐标 [-2, 2]
    pose.v[2] = ((double) rand() / RAND_MAX) * 2 * M_PI - M_PI; // 角度，从 -π 到 π
    printf("Random pose: (%.3f, %.3f, %.3f)\n", pose.v[0], pose.v[1], pose.v[2]);
    return pose;
}

// 激光扫描线程函数
void laser_thread_function() {
    while (true) {
        std::lock_guard<std::mutex> lock(laser_mutex);
        laserCallback(&scan);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// IMU线程函数
void imu_thread_function() {
    while (true) {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imuCallback(&imu);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Ackermann控制线程函数
void ackermann_thread_function() {
    while (true) {
        std::lock_guard<std::mutex> lock(ackermann_mutex);
        ackermannCmdCallback(&twist);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int run(void *dora_context)
{
    unsigned char counter = 0;
    msg2.x=0.0;msg2.y=0.0;msg2.theta=0;
    map = map_alloc();
    map_load_occ(map, ProjectPaths::build_nav_laser_data().c_str(), 0.005,1);
    map_image = cv::Mat(map->size_x, map->size_y, CV_8UC3, cv::Scalar(0, 0, 0));
    printf("AMCL地图信息: size_x=%d, size_y=%d, scale=%.6f, origin_x=%.6f, origin_y=%.6f\n", 
           map->size_x, map->size_y, map->scale, map->origin_x, map->origin_y);
    // 设置AMCL的激光雷达传感器模型
    amcl::AMCLLaser aa((size_t)2000, map);
    laser_sensor = aa;
    pf_vector_t v;
    v.v[0]=0;v.v[1]=0;v.v[2]=0;
    laser_sensor.SetLaserPose(v); // 设置激光雷达在机器人坐标系中的位置
    laser_sensor.SetModelLikelihoodField(0.99, 0.01, 0.1, 200); // 激光模型参数
    laser_data.sensor = &laser_sensor;

    // 创建粒子滤波器
    int min_samples = 100;      // 最小粒子数
    int max_samples = 500;    // 最大粒子数
    double alpha_slow = 0.1;   // 慢衰减率
    double alpha_fast = 0.2;   // 快衰减率
    pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                        random_pose_init, &laser_data);
    pf->selective_resampling = 1;
    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return -1;
    }
    // 初始均值和协方差矩阵
    pf_vector_t mean = {0, 0, M_PI/2}; // 初始均值 [x, y, theta]
    pf_matrix_t cov = {0.5, 0, 0, 0, 0.5, 0 , 0, 0, M_PI*M_PI}; // 初始协方差
    // 使用高斯模型初始化粒子滤波器
    pf_init(pf, mean, cov);
    
    // 启动线程 - 暂时注释掉以防止段错误
    // std::thread laser_thread(laser_thread_function);
    // std::thread imu_thread(imu_thread_function);
    // std::thread ackermann_thread(ackermann_thread_function);

    while(true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            // printf("[c node] ERROR: unexpected end of event\n");
            continue;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        //printf("[c node] received event: %d\n", ty);

        if (ty == DoraEventType_Input)
        {
            counter += 1;

            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            printf("AMCL输入ID: %s\n", id.c_str());

            if(id == "tick")
            {
                map_image.setTo(cv::Scalar(0, 0, 0));
                for(int i=0;i<map->size_x;i++){
                    for(int j=0;j<map->size_y;j++){
                        if(map->cells[MAP_INDEX(map, i, j)].occ_state==-1){
                            map_image.at<cv::Vec3b>(j,i)[0] = 255;
                        }
                    }
                }
                for(int i=0;i<2000;i++){
                    double angle = scan.angle_min + i * scan.angle_increment;
                    double x = scan.ranges[i] * cos(angle+msg2.theta)/map->scale+msg2.x;
                    double y = -scan.ranges[i] * sin(angle+msg2.theta)/map->scale+msg2.y;
                    if(x>0 && x<map->size_x && y>0 && y<map->size_y){
                        map_image.at<cv::Vec3b>(y,x)[1] = 255;
                        map_image.at<cv::Vec3b>(y,x)[2] = 255;
                    }
                }
                cv::circle(map_image, cv::Point(msg2.x, msg2.y), 10, cv::Scalar(0, 255, 0), -1);

                fprintf(file,"x: %f y: %f theta: %f\n",msg2.x,msg2.y,msg2.theta);
                cv::imshow("map",map_image);
                
                std::string out_id = "pose";
                nlohmann::json json_obj = msg2.to_json();
                std::string json_str = json_obj.dump();
                // printf("%s\n", json_str.c_str());
                const char* char_ptr = json_str.c_str();
                char* non_const_char_ptr = new char[json_str.size() + 1];
                std::memcpy(non_const_char_ptr, char_ptr, json_str.size() + 1);
                int result = dora_send_output(dora_context, &out_id[0], out_id.length(), reinterpret_cast<char*>(non_const_char_ptr), json_str.size());
                if (result != 0)
                {
                    std::cerr << "failed to send output" << std::endl;
                    return 1;
                }
                cv::waitKey(1);
                //printf("tick\n");
            }else if(id == "scan2"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("AMCL收到激光数据: %s\n", json_str.substr(0, 100).c_str());
                //replace_null_with_nan(json_str);
                //fprintf(file,"json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                scan = sensor_msgs::LaserScan::from_json(json_obj);
                // printf("seq: %d\n", scan.header.seq);
                // printf("stamp: %lld.%lld\n", scan.header.stamp.sec, scan.header.stamp.nsec);
                /* printf("frame_id: %s\n", scan.header.frame_id.c_str());
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
                printf("\n  "); */
                {
                    std::lock_guard<std::mutex> lock(laser_mutex);
                    scan = sensor_msgs::LaserScan::from_json(json_obj);
                }
                printf("AMCL激光数据解析: angle_min=%.6f, angle_max=%.6f, angle_increment=%.6f, range_min=%.6f, range_max=%.6f, ranges_count=%zu\n",
                       scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max, scan.ranges.size());
                
                // 调用激光传感器更新
                laserCallback(&scan);
                
                printf("AMCL位姿更新: x=%.3f, y=%.3f, theta=%.3f\n", msg2.x, msg2.y, msg2.theta);
            }else if(id == "imu"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("AMCL收到IMU数据: %s\n", json_str.substr(0, 100).c_str());
                //replace_null_with_nan(json_str);
                //printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                imu = sensor_msgs::Imu::from_json(json_obj);
                //std::vector<unsigned char> data;
                //for (size_t i = 0; i < data_len; i++)
                //{
                //    data.push_back(*(data_ptr + i));
                //}
                //sensor_msgs::Imu imu = sensor_msgs::Imu::from_vector(data);
                {
                    std::lock_guard<std::mutex> lock(imu_mutex);
                    imu = sensor_msgs::Imu::from_json(json_obj);
                }
                // 调用IMU回调函数更新global_theta
                imuCallback(&imu);
                
                // 发布IMU预估位姿（用于可视化对比）
                geometry_msgs::Pose2D imu_pose;
                imu_pose.x = msg2.x;  // 位置保持与AMCL一致（IMU只提供角度）
                imu_pose.y = msg2.y;
                imu_pose.theta = global_theta;  // 使用IMU提供的角度
                
                std::string imu_out_id = "imu_pose";
                nlohmann::json imu_json_obj = imu_pose.to_json();
                std::string imu_json_str = imu_json_obj.dump();
                char* imu_char_ptr = new char[imu_json_str.size() + 1];
                std::memcpy(imu_char_ptr, imu_json_str.c_str(), imu_json_str.size() + 1);
                dora_send_output(dora_context, &imu_out_id[0], imu_out_id.length(), imu_char_ptr, imu_json_str.size());
                delete[] imu_char_ptr;
                
                // 更新粒子滤波和位姿估计
                updateParticlePoses();
                process_samples(pf);
                printf("AMCL位姿更新: x=%.3f, y=%.3f, theta=%.3f, IMU角度=%.3f\n", msg2.x, msg2.y, msg2.theta, global_theta);
            }else if(id == "initial_pose"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("AMCL收到初始位姿: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                geometry_msgs::Pose2D initial_pose = geometry_msgs::Pose2D::from_json(json_obj);
                
                // 使用初始位姿重新初始化粒子滤波器
                pf_vector_t mean = {initial_pose.x, initial_pose.y, initial_pose.theta};
                pf_matrix_t cov = {0.5, 0, 0, 0, 0.5, 0, 0, 0, M_PI*M_PI};
                pf_init(pf, mean, cov);
                
                // 同步更新全局角度变量
                global_theta = initial_pose.theta;
                last_theta = initial_pose.theta;
                
                // 计算map→odom的旋转偏移
                // 当用户设置initial_pose时，我们假设：
                // - initial_pose.theta 是机器人在map坐标系中的真实朝向
                // - imu_yaw_in_odom 是机器人在odom坐标系中的朝向（来自IMU）
                // 因此：map_to_odom_offset = theta_map - theta_odom
                map_to_odom_yaw_offset = initial_pose.theta - imu_yaw_in_odom;
                map_to_odom_yaw_offset = toPI(map_to_odom_yaw_offset);
                
                printf("AMCL粒子滤波器已重新初始化: x=%.3f, y=%.3f, theta=%.3f\n", 
                       initial_pose.x, initial_pose.y, initial_pose.theta);
                printf("AMCL map→odom偏移已更新: offset=%.3f (map_theta=%.3f - odom_yaw=%.3f)\n", 
                       map_to_odom_yaw_offset, initial_pose.theta, imu_yaw_in_odom);
            }else if(id == "twist"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                //printf("json_str: %s\n", json_str.c_str());
                //replace_null_with_nan(json_str);
                //printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                twist = geometry_msgs::Twist::from_json(json_obj);
                //printf("linear x: %f\n", twist.linear.x);
                //printf("angular z: %f\n", twist.angular.z);
                //std::vector<unsigned char> data;
                //for (size_t i = 0; i < data_len; i++)
                //{
                //    data.push_back(*(data_ptr + i));
                //}
                //geometry_msgs::Twist twist = geometry_msgs::Twist::from_vector(data);
                // ackermannCmdCallback(&twist);
                {
                    std::lock_guard<std::mutex> lock(ackermann_mutex);
                    twist = geometry_msgs::Twist::from_json(json_obj);
                }
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
    file = fopen(ProjectPaths::amcl_txt().c_str(),"w");
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
