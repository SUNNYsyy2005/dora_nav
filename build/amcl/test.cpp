#include <iostream>
#include "amcl/map/map.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_odom.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>

#define M_PI 3.14159265358979323846
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
// 一个简单的随机姿态生成函数，用于初始化粒子
double global_x = 0.0; // x坐标
double global_y = 0.0; // y坐标
double global_theta = M_PI/2; // 角度
double last_theta = M_PI/2;
double steer_theta = 0.0;
double last_velocity = 0.0;
double last_steering_angle_velocity = 0.0;
geometry_msgs::Pose2D msg2;
std::chrono::steady_clock::time_point last_update_time = std::chrono::steady_clock::now();
pf_vector_t random_pose_init(void *data) {
    pf_vector_t pose;
    // 这里假设一个均匀分布，实际应用中可能需要根据实际情况调整
    pose.v[0] = (double) rand() / RAND_MAX * 2000 - 1000.0; // x 坐标
    pose.v[1] = (double) rand() / RAND_MAX * 2000 - 1000.0; // y 坐标
    pose.v[2] = (double) rand() / RAND_MAX * 2 * M_PI - M_PI; // 角度，从 -π 到 π
    printf("Random pose: (%f, %f, %f)\n", pose.v[0], pose.v[1], pose.v[2]);
    return pose;
}

pf_t *pf;
map_t *map;
amcl::AMCLLaser laser_sensor(10, NULL);
amcl::AMCLOdom odom_sensor;
amcl::AMCLLaserData laser_data;
amcl::AMCLOdomData odom_data;
sensor_msgs::LaserScan::ConstPtr last_msg;
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
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    global_theta = yaw;
    //ROS_INFO("Yaw: [%f]", yaw);
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 获取四元数姿态
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 将四元数转换为欧拉角
    global_theta = yaw;
    // yaw是小车的朝向
    //ROS_INFO("IMU Yaw: [%f]", yaw);
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
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    auto now = std::chrono::steady_clock::now();
    // 检查自上次回调以来是否已经过了5秒
    if (std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count() < 200000) {
        // 如果没有过5秒，就直接返回，不处理这次消息
        return;
    }
    updateParticlePoses();
    // 更新上次处理消息的时间
    last_time = now;
    // 更新上一次接收到的消息
    last_msg = msg;
    //std::cout<<msg->ranges.size()<<std::endl;
    //for(int i=0;i<msg->ranges.size();i++){
        //std::cout<<msg->ranges[i]<<" ";
    //}
    laser_data.ranges.resize(msg->ranges.size(), std::vector<double>(2)); 
    laser_data.range_count = msg->ranges.size();
    laser_data.range_max = msg->range_max;
    double range_min = msg->range_min;
    double angle_increment = msg->angle_increment;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        // 填充激光雷达数据，这里只处理了距离，角度需要您根据实际情况计算
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
void print(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("Linear:");
    ROS_INFO("  x: %f", msg->linear.x);
    ROS_INFO("  y: %f", msg->linear.y);
    ROS_INFO("  z: %f", msg->linear.z);

    ROS_INFO("Angular:");
    ROS_INFO("  x: %f", msg->angular.x);
    ROS_INFO("  y: %f", msg->angular.y);
    ROS_INFO("  z: %f", msg->angular.z);
}
void ackermannCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
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
    //std::cout<<dt<<"@@@@@@@@@"<<std::endl;
    /* if (std::fabs(steering_angle_velocity) < 1e-6) { // 直行
        if(std::fabs(velocity) > 1e-3){
            global_x += velocity * dt * cos(global_theta);
            global_y -= velocity * dt * sin(global_theta);
            //global_theta += steer_theta;
            steer_theta = 0;
        }
    } else { // 转弯
        double abs_velocity = std::fabs(velocity); // 取速度的绝对值
        if(abs_velocity <1e-3){
            steer_theta += (velocity > 0 ? 1 : -1) * steering_angle_velocity * dt;

        }else{
            double turning_radius = abs_velocity / steering_angle_velocity;
            double theta_change = abs_velocity * dt / turning_radius;
            global_x += turning_radius * (sin(global_theta + theta_change) - sin(global_theta));
            global_y += turning_radius * (cos(global_theta + theta_change) - cos(global_theta)); // 注意y轴向下的坐标系统
            //global_theta += (velocity > 0 ? 1 : -1) * theta_change;
            //global_theta += steer_theta;
            steer_theta = 0;
        }
    } */
    if (std::isnan(global_x)){
        std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
        exit(0);
    }
    // 确保角度保持在合理的范围内
    //global_theta = toPI(global_theta);
    //std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
    //std::cout << "X: " << global_x << ", Y: " << global_y << ", Theta: " << global_theta << std::endl;
}




int main(int argc, char** argv){
    ros::init(argc, argv, "amcl_node");
    ros::NodeHandle nh;
    ros::Subscriber ackermann_sub = nh.subscribe("/tianracer/cmd_vel", 100000, ackermannCmdCallback);
    ros::Subscriber sub = nh.subscribe("tianracer/scan", 500, laserCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("tianracer/pose", 1000);
    ros::Subscriber imu_sub = nh.subscribe("/tianracer/imu", 1000, imuCallback);
    msg2.x=1000;msg2.y=1000;msg2.theta=0;
    
    map = map_alloc();
    map_load_occ(map, "map/1.pgm", 0.04,0);

    printf("%d %d %f %f\n", map->size_x, map->size_y, map->origin_x, map->origin_y);
    // 粒子滤波器的参数
    int min_samples = 100;      // 最小粒子数
    int max_samples = 1000;    // 最大粒子数
    double alpha_slow = 0.1;   // 慢衰减率
    double alpha_fast = 0.1;   // 快衰减率
    // 设置AMCL的激光雷达传感器模型
    amcl::AMCLLaser aa((size_t)1080, map);
    laser_sensor = aa;
    pf_vector_t v;
    v.v[0]=0;v.v[1]=0;v.v[2]=0;
    laser_sensor.SetLaserPose(v); // 设置激光雷达在机器人坐标系中的位置
    //printf("Laser sensor created %p@\n",laser_sensor.map);
    laser_sensor.SetModelLikelihoodField(0.99, 0.01, 0.1, 10); // 激光模型参数
    //printf("Laser sensor model set\n");
    laser_data.sensor = &laser_sensor;
    // 创建粒子滤波器
    pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                        random_pose_init, &laser_data);
    pf->selective_resampling = 1;
    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return -1;
    }
    //printf("Particle filter created\n");
    // 初始均值和协方差矩阵
    pf_vector_t mean = {0, 0, 0}; // 初始均值 [x, y, theta]
    pf_matrix_t cov = {1, 0, 0, 0, 1, 0 , 0, 0, M_PI*M_PI}; // 初始协方差
    // 使用高斯模型初始化粒子滤波器
    pf_init(pf, mean, cov);
    //printf("Particle filter initialized\n");
    ros::Rate loop_rate(100000);
    while (ros::ok())
    {
        pose_pub.publish(msg2);
        ros::spinOnce();
        loop_rate.sleep();
        msg2.x += global_x;
        msg2.y += global_y;
        msg2.theta = global_theta + steer_theta;
    }
    map_free(map);
    std::cout << "AMCL TEST MISC" << std::endl;
    return 0;
}