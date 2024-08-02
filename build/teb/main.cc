extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>

#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>

#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "../../include/ros.h"

using namespace teb_local_planner;
const int step = 3;
const int width = 500;
const int height = 500;
const int map_width = 800;
const int map_height = 800;
const int consider_width = 200;
const int consider_height = 200;
const double PI = 3.1415926;
const double scale = 0.04;
double GXtGY(double x){
    return map_width-x;
}
double GYtGX(double y){
    return map_height-y;
}
int GXtMX(double x)
{
    return (int)((((x-map_width/2) / consider_width)+0.5) * width);
}
int GYtMY(double y)
{
    return (int)((((y-map_height/2) / consider_height)+0.7) * height);
}
int MXtMY(int x)
{
    return (int)(width - x);
}
int MYtMX(int y)
{
    return (int)(height - y);
}
double GXtRX(double x){
    return x*scale;
}
double GYtRY(double y){
    return y*scale;
}
int RXtMY(double x){
    return GYtMY(GXtGY(x/scale));
}
int RYtMX(double y){
    return GXtMX(GYtGX(y/scale));
}
geometry_msgs::Pose2D robot;
sensor_msgs::LaserScan scan;
void replace_null_with_nan(std::string& json_str) {
    std::string null_str = "null";
    std::string nan_str = "-1";
    size_t pos = 0;
    while ((pos = json_str.find(null_str, pos)) != std::string::npos) {
        json_str.replace(pos, null_str.length(), nan_str);
        pos += nan_str.length();
    }
}
int run(void *dora_context)
{
    std::string line;
    std::ifstream file("/home/sunny/dora_nav/build/teb/path.csv");
    std::vector<std::pair<float, float>> pathh;
    if (file.is_open()) {
        
        while (std::getline(file, line)) {
            std::stringstream linestream(line);
            std::string value;
            float x, y;
            std::getline(linestream, value, ',');
            x = std::stof(value);
            std::getline(linestream, value, ',');
            y = std::stof(value);
            pathh.push_back(std::make_pair(x, y));
        }
        file.close();
        //for (const auto& point : pathh) {
        //    std::cout << "Point: (" << point.first << ", " << point.second << ")" << std::endl;
        //}
        std::reverse(pathh.begin(), pathh.end());
         //打印路径以验证
        for (const auto& point : pathh) {
            std::cout << "Point: (" << point.first << ", " << point.second << ")" << std::endl;
        }
    } else {
        std::cout << "Unable to open file" << std::endl;
    }
    // 参数配置
    TebConfig config;
    PoseSE2 start(100, 0, PI/2);
    PoseSE2 end(GXtRX(GYtGX(pathh[0].second)),GXtRX(GYtGX((pathh[0].first))), 0);
    printf("end x: %f, y: %f\n", GYtGX(end.y()/scale), GXtGY(end.x()/scale));
    std::vector<ObstaclePtr> obst_vector;
    ViaPointContainer via_points;
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);
    int reach_num = 0;

    while(true)
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

            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            
            if(id == "tick"){
                
                memset(show_map.data, 0, 500 * 500 * 3);
                try
                {
                    start.x() = GXtRX(GYtGX(robot.y));
                    start.y() = GYtRY(GXtGY(robot.x));
                    start.theta() = robot.theta-PI/2;
                    obst_vector.clear();
                    int r = 10; // 设置半径大小
                    cv::Point center(GXtMX(robot.x), GYtMY(robot.y)); // 计算圆心
                    cv::circle(show_map, center, r, cv::Scalar(255, 255, 255), -1); // 绘制填充圆
                    for(int i=0;i<scan.range_min;i++){
                        double angle = scan.angle_min + i*scan.angle_increment;
                        double gx = scan.ranges[i] * cos(angle + robot.theta)/scale + robot.x;
                        double gy = -scan.ranges[i] * sin(angle + robot.theta)/scale + robot.y;
                        double x = GXtRX(GYtGX(gy));
                        double y = GYtRY(GXtGY(gx));
                        int x_ = GXtMX(gx);
                        int y_ = GYtMY(gy);
                        //printf("x: %d, y: %d\n", x_, y_);
                        if(x_>=0 && x_<500 && y_>=0 && y_<500){
                            if(scan.ranges[i] < 2.5){
                                show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(125, 125, 125);
                            }
                            else{
                            show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(50, 50, 50);
                            }
                        }
                        if(scan.ranges[i] < 1.5){
                            obst_vector.emplace_back(boost::make_shared<PointObstacle>(x, y));
                        }
                    }
                    auto s = std::chrono::high_resolution_clock::now();
                    printf("start x: %f, y: %f, theta: %f", GYtGX(start.y()/scale), GXtGY(start.x()), start.theta());
                    planner->plan(start, end);
                    // do somthine
                    auto e = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(e - s);
                    std::cout << "cost "<< ms.count()  <<" ms"<< std::endl;
                    // vi
                    std::vector<Eigen::Vector3f> path;
                    planner->getFullTrajectory(path);
                    std::cout << "path size: " << path.size() << std::endl;
                    for (int i = 0; i < path.size() - 1; i++)
                    {
                        int y = RXtMY(path.at(i)[0]);
                        int x = RYtMX(path.at(i)[1]);
                        int next_y = RXtMY(path.at(i + 1)[0]);
                        int next_x = RYtMX(path.at(i + 1)[1]);
                        cv::line(show_map, cv::Point(x, y), cv::Point(next_x, next_y), cv::Scalar(0, 0, 255));
                    }
                    float vx, vy, w;
                    planner->getVelocityCommand(vx, vy, w,step);
                    printf("next x: %f, y: %f, theta: %f",  GYtGX(path.at(step)[1]/scale), GXtGY(path.at(step)[0]/scale), path.at(step)[2]);
                    printf("end x: %f, y: %f, theta: %f",  GYtGX(end.y()/scale), GXtGY(end.x()/scale), end.theta());
                    printf("vx: %f,vy: %f, w: %f\n", vx,vy, w);
                    std::string out_id = "twist";
                    geometry_msgs::Twist twist;
                    twist.linear.x = vx;
                    twist.angular.z = w;
                    nlohmann::json json_obj = twist.to_json();
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
                    cv::imshow("path", show_map);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "捕获到异常: " << e.what() << std::endl;
                    break;
                }
                catch (...)
                {
                    std::cerr << "捕获到未知类型的异常" << std::endl;
                    break;
                }
                cv::waitKey(10);
                while(pow(pathh[reach_num].first-robot.x,2)+pow(pathh[reach_num].second-robot.y,2)<100){
                    reach_num++;
                    end.x() = GXtRX(GYtGX(pathh[reach_num].second));
                    end.y() = GYtRY(GXtGY(pathh[reach_num].first));
                    std::string out_id = "twist";
                    geometry_msgs::Twist twist;
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    //
                    //twist.linear.x = vx;
                    //twist.angular.z = vw;
                    //std::vector<unsigned char> out_vec = twist.to_vector();
                    nlohmann::json json_obj = twist.to_json();
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
                }
            }
            else if(id == "pose"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("json_str: %s\n", json_str.c_str());
                replace_null_with_nan(json_str);
                printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                robot = geometry_msgs::Pose2D::from_json(json_obj);
            }else if(id == "scan"){
                char *data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                printf("json_str: %s\n", json_str.c_str());
                replace_null_with_nan(json_str);
                printf("json_str: %s\n", json_str.c_str());
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                /* unsigned char* uchar_ptr = reinterpret_cast<unsigned char*>(data_ptr);
                std::vector<unsigned char> uchar_vec(uchar_ptr, uchar_ptr + data_len);
                std::vector<unsigned char> data;
                for (size_t i = 0; i < data_len; i++)
                {
                    data.push_back(*(data_ptr + i));
                } */
                scan = sensor_msgs::LaserScan::from_json(json_obj);
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
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
