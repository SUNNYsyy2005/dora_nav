#include "../../../../include/ros.h"

class Action {
public:
    Action() {
        pub = nh.advertise<geometry_msgs::Twist>("/tianracer/cmd_vel", 1000);
        last_call_time = ros::Time::now().toSec();
        steer_angle = 0.0;
        last_vm = false;
        last_vw = 0.0;
    }

    void sendCommand(double vx = 0, double vw = 0) {
        double current_time = ros::Time::now().toSec();
        double interval = current_time - last_call_time;
        last_call_time = current_time;

        if (last_vm) {
            steer_angle += vw * interval;
            ROS_INFO("steer_angle: %f", steer_angle);
        } else {
            steer_angle = 0.0;
        }

        if (abs(vx) < 1e-6) {
            last_vm = true;
            last_vw = vw;
        } else {
            last_vm = false;
        }

        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = vx;
        twist_cmd.angular.z = vw;

        //ROS_INFO("Interval: %.2f seconds, Publishing Twist Command: linear.x=%f, angular.z=%f", interval, twist_cmd.linear.x, twist_cmd.angular.z);
        pub.publish(twist_cmd);
    }

private:
    double last_call_time;
    double steer_angle;
    bool last_vm;
    double last_vw;
};

/* int main(int argc, char** argv) {
    ros::init(argc, argv, "action_node");
    Action action_module;
    ros::Rate rate(50); // 50Hz

    while (ros::ok()) {
        action_module.sendCommand(100, 10);
        rate.sleep();
    }

    return 0;
} */