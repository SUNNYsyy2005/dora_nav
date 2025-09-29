extern "C"
{
#include "node_api.h"
}

#include <thread>
#include <mutex>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include "../../include/ros.h"
#include "../../include/project_paths.h"



using namespace teb_local_planner;

// 全局变量与互斥锁
std::mutex scan_mutex, pose_mutex;
geometry_msgs::Pose2D robot;
sensor_msgs::LaserScan scan;
std::vector<std::pair<float, float>> pathh;
bool iftick = false;
FILE *filename;

const int step = 20;
const int width = 500;
const int height = 500;
const int map_width = 800;
const int map_height = 800;
const int consider_width = 800;
const int consider_height = 800;
const double PI = 3.1415926;
const double scale = 0.04;

double GXtGY(double x) { return map_width - x; }
double GYtGX(double y) { return map_height - y; }
int GXtMX(double x) { return (int)((((x - map_width / 2) / consider_width) + 0.5) * width); }
int GYtMY(double y) { return (int)((((y - map_height / 2) / consider_height) + 0.5) * height); }
double GXtRX(double x) { return x * scale; }
double GYtRY(double y) { return y * scale; }
int RXtMY(double x) { return GYtMY(GXtGY(x / scale)); }
int RYtMX(double y) { return GXtMX(GYtGX(y / scale)); }

void replace_null_with_nan(std::string& json_str) {
    std::string null_str = "null";
    std::string nan_str = "-1";
    size_t pos = 0;
    while ((pos = json_str.find(null_str, pos)) != std::string::npos) {
        json_str.replace(pos, null_str.length(), nan_str);
        pos += nan_str.length();
    }
}

// 激光扫描线程函数
void scan_thread_function(TebOptimalPlanner* planner, std::vector<ObstaclePtr>& obst_vector, cv::Mat& show_map, TebConfig& config) {
    while (true) {
        std::lock_guard<std::mutex> lock(scan_mutex);
        if (iftick) {
            // 处理激光扫描数据
            memset(show_map.data, 0, 800 * 800 * 3);
            obst_vector.clear();
            for (int i = 0; i < scan.ranges.size(); i++) {
                if (scan.ranges[i] == NAN) continue;
                double angle = scan.angle_min + i * scan.angle_increment;
                double gx = scan.ranges[i] * cos(angle + robot.theta) / scale + robot.x;
                double gy = -scan.ranges[i] * sin(angle + robot.theta) / scale + robot.y;
                double x = GXtRX(GYtGX(gy));
                double y = GYtRY(GXtGY(gx));
                int x_ = GXtMX(gx);
                int y_ = GYtMY(gy);
                if (x_ >= 0 && x_ <= 800 && y_ >= 0 && y_ <= 800) {
                    if (scan.ranges[i] < 3) {
                        show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(125, 125, 125);
                    } else {
                        show_map.at<cv::Vec3b>(y_, x_) = cv::Vec3b(50, 50, 50);
                    }
                }
                if (scan.ranges[i] < 20) {
                    obst_vector.emplace_back(boost::make_shared<PointObstacle>(x, y));
                }
            }
            cv::imshow("path", show_map);
            cv::waitKey(1);
        }
    }
}

void pose_thread_function() {
    geometry_msgs::Pose2D last_pose = robot;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(pose_mutex);

            if (robot.x != last_pose.x || robot.y != last_pose.y || robot.theta != last_pose.theta) {
                std::cout << "Current Pose - X: " << robot.x << " Y: " << robot.y << " Theta: " << robot.theta << std::endl;
                fprintf(filename, "Current Pose - X: %f Y: %f Theta: %f\n", robot.x, robot.y, robot.theta);

                if (robot.theta < 0) {
                    robot.theta += 2 * PI;
                } else if (robot.theta >= 2 * PI) {
                    robot.theta -= 2 * PI;
                }

                last_pose = robot;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 路径规划线程函数
void path_planning_thread_function(TebOptimalPlanner* planner, cv::Mat& show_map, TebConfig& config, std::vector<std::pair<float, float>>& pathh, void* dora_context) {
    int reach_num = 0;
    while (true) {
        if (iftick) {
            try {
                PoseSE2 start(GXtRX(GYtGX(robot.y)), GYtRY(GXtGY(robot.x)), robot.theta - PI / 2);
                PoseSE2 end(GXtRX(GYtGX(pathh[reach_num].second)), GXtRX(GYtGX(pathh[reach_num].first)), PI / 2);
                planner->plan(start, end);

                float vx, vy, w;
                planner->getVelocityCommand(vx, vy, w, step);
                geometry_msgs::Twist twist;
                twist.linear.x = vx;
                twist.angular.z = w;

                std::string out_id = "twist";
                nlohmann::json json_obj = twist.to_json();
                std::string json_str = json_obj.dump();
                const char* char_ptr = json_str.c_str();
                char* non_const_char_ptr = new char[json_str.size() + 1];
                std::memcpy(non_const_char_ptr, char_ptr, json_str.size() + 1);
                int result = dora_send_output(dora_context, &out_id[0], out_id.length(), reinterpret_cast<char*>(non_const_char_ptr), json_str.size());
                if (result != 0) {
                    std::cerr << "failed to send output" << std::endl;
                    return;
                }
                cv::imshow("path", show_map);
                cv::waitKey(1);
            } catch (const std::exception& e) {
                std::cerr << "捕获到异常: " << e.what() << std::endl;
                break;
            } catch (...) {
                std::cerr << "捕获到未知类型的异常" << std::endl;
                break;
            }
        }
    }
}

// 主运行函数
int run(void* dora_context) {
    // 初始化路径
    std::string line;
    std::ifstream file(ProjectPaths::build_teb_path_csv());
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
        std::reverse(pathh.begin(), pathh.end());
    } else {
        std::cout << "Unable to open file: " <<ProjectPaths::build_teb_path_csv()<< std::endl;
    }

    // 初始化
    TebConfig config;
    cv::Mat show_map = cv::Mat::zeros(cv::Size(800, 800), CV_8UC3);
    std::vector<ObstaclePtr> obst_vector;
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, nullptr, nullptr);

    // 启动线程 - 暂时注释掉以防止段错误
    // std::thread scan_thread(scan_thread_function, planner, std::ref(obst_vector), std::ref(show_map), std::ref(config));
    // std::thread pose_thread(pose_thread_function);
    // std::thread path_planning_thread(path_planning_thread_function, planner, std::ref(show_map), std::ref(config), std::ref(pathh), dora_context);

    // 事件处理循环
    while (true) {
        void* event = dora_next_event(dora_context);
        if (event == NULL) {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input) {
            char* id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);

            if (id == "tick") {
                iftick = true;
            } else if (id == "pose") {
                std::lock_guard<std::mutex> lock(pose_mutex);
                char* data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                robot = geometry_msgs::Pose2D::from_json(json_obj);
            } else if (id == "scan" && iftick) {
                std::lock_guard<std::mutex> lock(scan_mutex);
                char* data_ptr;
                size_t data_len;
                read_dora_input_data(event, &data_ptr, &data_len);
                std::string json_str(data_ptr, data_len);
                nlohmann::json json_obj = nlohmann::json::parse(json_str);
                scan = sensor_msgs::LaserScan::from_json(json_obj);
            }
        } else if (ty == DoraEventType_Stop) {
            printf("[c node] received stop event\n");
            break;
        } else {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    // 等待线程结束 - 暂时注释掉防止段错误
    // scan_thread.join();
    // pose_thread.join();
    // path_planning_thread.join();

    return 0;
}

int main() {
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    auto dora_context = init_dora_context_from_env();
    filename = fopen(ProjectPaths::teb_txt().c_str(), "w");
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
