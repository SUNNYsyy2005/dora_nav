#include "../../../../include/ros.h"
class Robot {
    public:
        double x, y, orientation;
        Robot(){
            this->x=-999999; 
            this->y=-999999;
            this->orientation=0;
        } 
    };
class Vision {
public:
    Vision(){
        _my_robot.x = 400;
        _my_robot.y = 400;
        _my_robot.orientation = 1.57;
    }

    void scanCallback(const sensor_msgs::LaserScan msg) {
        data = msg;
    }

    void poseCallback(const geometry_msgs::Pose2D msg) {
        _my_robot.x = msg->x;
        _my_robot.y = msg->y;
        _my_robot.orientation = msg->theta; 
    }

public:
    Robot _my_robot;
    sensor_msgs::LaserScan data;
};