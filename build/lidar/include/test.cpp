#include <ctime>
#include "ros.h"


int main() {
    ros::NodeHandle nh;
    std::string device_ip;
    nh.param<std::string>("device_ip", device_ip, "default_ip");
    printf("device_ip: %s\n", device_ip.c_str());
    ros::Rate rate;
    rate.sleep();
    printf("sleep 100ms\n");
    time_t now = time(NULL);
    printf("now: %ld\n", now);
    return 0;
}