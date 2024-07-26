#include <iostream>
#include <cmath>
#include <unistd.h>
#include "A_star_dwa.cpp"

double getAngle(int x1, int y1, int x2, int y2) {
    double angle = atan2(y1 - y2, x1 - x2);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

int main() {
    usleep(100000); // 否则是-999999
    int start_x = 400, start_y = 400;
    double start_angle = M_PI / 2;
    int goal_x = 330, goal_y = 300;

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