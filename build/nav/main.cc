#include <iostream>
#include <cmath>
#include <unistd.h>
#include "A_star_dwa.cc"

double getAngle(int x1, int y1, int x2, int y2) {
    double angle = atan2(y1 - y2, x1 - x2);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}
//g++ -o main main.cc -I/usr/local/include -L/usr/local/lib -llz4 -lm
int main(int argc, char **argv) {

    usleep(100000); // 否则是-999999
    int start_x = 400, start_y = 400;
    double start_angle = M_PI / 2;
    int goal_x = 200, goal_y = 250;
    if(argc==5){
        start_x = std::stoi(argv[1]);
        start_y = std::stoi(argv[2]);
        goal_x = std::stoi(argv[3]);
        goal_y = std::stoi(argv[4]);
    }else if(argc==3){
        goal_x = std::stoi(argv[1]);
        goal_y = std::stoi(argv[2]);
    }
    printf("start_x: %d, start_y: %d, goal_x: %d, goal_y: %d\n", start_x, start_y, goal_x, goal_y);
    

    double goal_angle = getAngle(start_x, start_y, goal_x, goal_y);

    Astar_DWA planner;
    for (int i = 1; i < 2; ++i) {
        auto [path_x, path_y, flag] = planner.plan(start_x, start_y, start_angle, goal_x, goal_y, goal_angle);
        if (flag == 1) {
            std::cout << "Path found!" << std::endl;
        } else {
            std::cout << "Path not found!" << std::endl;
        }
    }

    return 0;
}
