extern "C"
{
#include "../include/dora/node_api.h"
}

#include <iostream>
#include <vector>

#include "../include/lslidar_driver/lslidar_driver.h"

#include <thread>
#include <boost/thread/thread.hpp>

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;
//std::string lidar_type;

static void my_handler(int sig) {
    flag = 0;
}
Ros::NodeHandle node;
Ros::NodeHandle private_nh;
lslidar_driver::LslidarDriver dvr(node, private_nh);

int run(void *dora_context)
{
    unsigned char counter = 0;

    while(true)
    {
        dvr.poll(dora_context);
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            counter += 1;

            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);
            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            std::vector<unsigned char> data;
            for (size_t i = 0; i < data_len; i++)
            {
                data.push_back(*(data_ptr + i));
            }

            std::cout
                << "Received input "
                << " (counter: " << (unsigned int)counter << ") data: [";
            for (unsigned char &v : data)
            {
                std::cout << (unsigned int)v << ", ";
            }
            std::cout << "]" << std::endl;

            
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
    

    signal(SIGINT, my_handler);
    private_nh.param("lidar_type", lidar_type, std::string("c16"));
    printf("lslidar type: %s\n", lidar_type.c_str());
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    // loop until shut down or end of file
    if (!dvr.initialize()) {
        std::cerr<<"cannot initialize lslidar driver."<<std::endl;
        //ROS_ERROR("cannot initialize lslidar driver.");
        return 0;
    }

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
