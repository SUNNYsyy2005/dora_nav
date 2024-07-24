#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>
//
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include "include/teb/vision.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

int main(int argc, char** argv)
{
    std::string line;
    std::ifstream file("path.csv");
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
    //obst_vector.emplace_back(boost::make_shared<PointObstacle>(200, 100));
    //obst_vector.emplace_back(boost::make_shared<PointObstacle>(300, 100));
    ViaPointContainer via_points;
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

    ros::init(argc, argv, "teb_node");
    Action action_module;
    Vision vision_module;
    ros::Rate rate(10); // 50Hz
    int reach_num = 0;
    while (ros::ok() && reach_num<pathh.size())
    {
        ros::spinOnce();
        memset(show_map.data, 0, 500 * 500 * 3);
        try
        {
            start.x() = GXtRX(GYtGX(vision_module._my_robot.y));
            start.y() = GYtRY(GXtGY(vision_module._my_robot.x));
            start.theta() = vision_module._my_robot.orientation-PI/2;
            obst_vector.clear();
            int r = 10; // 设置半径大小
            cv::Point center(GXtMX(vision_module._my_robot.x), GYtMY(vision_module._my_robot.y)); // 计算圆心
            cv::circle(show_map, center, r, cv::Scalar(255, 255, 255), -1); // 绘制填充圆
            for(int i=0;i<vision_module.data.range_count;i++){
                double angle = vision_module.data.angle_min + i*vision_module.data.angle_increment;
                //double x = vision_module.data.ranges[i] * cos(angle + start.theta()) + start.x();
                //double y = vision_module.data.ranges[i] * sin(angle + start.theta()) + start.y();
                //int y_ = RXtMY(x);
                //int x_ = RYtMX(y);
                double gx = vision_module.data.ranges[i] * cos(angle + vision_module._my_robot.orientation)/scale + vision_module._my_robot.x;
                double gy = -vision_module.data.ranges[i] * sin(angle + vision_module._my_robot.orientation)/scale + vision_module._my_robot.y;
                double x = GXtRX(GYtGX(gy));
                double y = GYtRY(GXtGY(gx));
                int x_ = GXtMX(gx);
                int y_ = GYtMY(gy);
                //printf("x: %d, y: %d\n", x_, y_);
                if(x_>=0 && x_<500 && y_>=0 && y_<500){
                    if(vision_module.data.ranges[i] < 2.5){
                        show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(125, 125, 125);
                    }
                    else{
                    show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(50, 50, 50);
                    }
                }
                if(vision_module.data.ranges[i] < 1.5){
                    obst_vector.emplace_back(boost::make_shared<PointObstacle>(x, y));
                }
            }
            auto s = std::chrono::high_resolution_clock::now();
            ROS_INFO("start x: %f, y: %f, theta: %f", GYtGX(start.y()/scale), GXtGY(start.x()), start.theta());
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
            ROS_INFO("next x: %f, y: %f, theta: %f",  GYtGX(path.at(step)[1]/scale), GXtGY(path.at(step)[0]/scale), path.at(step)[2]);
            ROS_INFO("end x: %f, y: %f, theta: %f",  GYtGX(end.y()/scale), GXtGY(end.x()/scale), end.theta());
            printf("vx: %f,vy: %f, w: %f\n", vx,vy, w);
            action_module.sendCommand(vx, w);
            //exit(0);
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
        while(pow(pathh[reach_num].first-vision_module._my_robot.x,2)+pow(pathh[reach_num].second-vision_module._my_robot.y,2)<100){
            reach_num++;
            end.x() = GXtRX(GYtGX(pathh[reach_num].second));
            end.y() = GYtRY(GXtGY(pathh[reach_num].first));
            action_module.sendCommand(0,0);
        }
        rate.sleep();
    }
    return 0;
}
