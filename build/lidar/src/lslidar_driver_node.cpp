#include "../include/lslidar_driver/lslidar_driver.h"

#include <thread>
#include <boost/thread/thread.hpp>

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;
//std::string lidar_type;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char **argv) {
    //Ros::init(argc, argv, "lslidar_driver");
    Ros::NodeHandle node;
    Ros::NodeHandle private_nh;

    signal(SIGINT, my_handler);
    private_nh.param("lidar_type", lidar_type, std::string("c16"));
    printf("lslidar type: %s\n", lidar_type.c_str());
   // ROS_INFO("lslidar type: %s", lidar_type.c_str());

    lslidar_driver::LslidarDriver dvr(node, private_nh);
    // loop until shut down or end of file
    if (!dvr.initialize()) {
        std::cerr<<"cannot initialize lslidar driver."<<std::endl;
        //ROS_ERROR("cannot initialize lslidar driver.");
        return 0;
    }
    while(true){
        dvr.poll();
        //Ros::spinOnce();
    }
    /* boost::thread poll_thread([&]() {
                                while (true) {
                                    dvr.poll();
                                    //Ros::spinOnce();
                                }
                            }
    );
    poll_thread.detach(); */
    //Ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    //spinner.spin();

    return 0;
}
