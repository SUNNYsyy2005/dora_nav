#include "include/ros.h"
#include <iostream>
#include "nlohmann/json.hpp"
int main(){
    printf("Hello World\n");
    sensor_msgs::LaserScan msg = sensor_msgs::newLaserScan();
    auto JSON = msg.to_vector();//msg.to_json();
    auto msg2 = sensor_msgs::LaserScan::from_vector(JSON);//from_json(JSON);
    print(msg2);
    sensor_msgs::Imu imu;
    auto JSON2 = imu.to_vector();
    auto imu2 = sensor_msgs::Imu::from_vector(JSON2);
    print(imu2);
    //auto batch = sensor_msgs::ConvertLaserScanToArrow(msg);
    //auto msg2 = sensor_msgs::ConvertArrowToLaserScan(batch);
    //print(msg2);
    return 0;
}