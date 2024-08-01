#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <fstream>
#include <flann/flann.hpp>

class Node {
public:
    int x, y;
    Node* parent;
    double G, H;

    Node(int x, int y, Node* parent_node) : x(x), y(y), parent(parent_node), G(0), H(0) {}
};

class Astar_DWA {
public:
    int MAX_EDGE_LEN;
    int LIMIT_TRIAL;
    int minx, maxx, miny, maxy;
    int robot_size;
    int avoid_dist;
    int r;
    flann::Index<flann::L2<int>>* obstree;
    int step_length;
    std::vector<Node*> openlist;
    std::vector<Node*> closelist;
    int pathPointInterval;

    Astar_DWA(int MAX_EDGE_LEN = 5000, int LIMIT_TRIAL = 500000) 
        : MAX_EDGE_LEN(MAX_EDGE_LEN), LIMIT_TRIAL(LIMIT_TRIAL), minx(0), maxx(800), miny(0), maxy(800),
          robot_size(4), avoid_dist(4), r(10), obstree(nullptr), step_length(5), pathPointInterval(20) {}

    std::tuple<std::vector<int>, std::vector<int>, int> plan(int start_x, int start_y, double start_angle, int goal_x, int goal_y, double goal_angle) {
        // 初始化障碍物坐标数组
        std::vector<int> obstacle_x, obstacle_y;

        // 打开PGM文件
        std::string image_path = "../1.pgm";
        std::ifstream file(image_path);
        if (!file) {
            std::cerr << "Failed to open image file: " << image_path << std::endl;
            return {{}, {}, 0};
        }
        printf("Open image file successfully!\n");
        // 读取PGM文件头
        std::string magic_number;
        int width, height, max_val;
        // 跳过注释行
        file >> magic_number;
        file.ignore(1); // 跳过一个字符（通常是换行符）
        std::string line;
        std::getline(file, line);
        while (line[0] == '#') {
            std::cout<<"line: "<<line<<std::endl;
            std::getline(file, line);
        }
        std::cout<<"line: "<<line<<std::endl;
        // 读取宽度、高度和最大灰度值
        std::istringstream iss(line);
        iss >> width >> height;
        file >> max_val;
        file.ignore(1); // 跳过一个字符（通常是换行符）

        if (magic_number != "P5") {
            std::cerr << "Unsupported PGM format: " << magic_number << std::endl;
            return {{}, {}, 0};
        }

        // 读取图像数据
        std::vector<uint8_t> img_data(width * height);
        file.read(reinterpret_cast<char*>(img_data.data()), img_data.size());
        printf("Read image data successfully!\n");
        printf("width: %d, height: %d\n", width, height);
        // 遍历图像中的每个像素
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (img_data[y * width + x] < 254) {
                    obstacle_x.push_back(x);
                    obstacle_y.push_back(y);
                    if(x==330 && y==300){
                        printf("error");
                    }
                    //printf("x: %d, y: %d\n", x, y);
                }
            }
        }

        // 构建障碍物 KD 树
        flann::Matrix<int> dataset(new int[obstacle_x.size() * 2], obstacle_x.size(), 2);
        for (size_t i = 0; i < obstacle_x.size(); ++i) {
            dataset[i][0] = obstacle_x[i];
            dataset[i][1] = obstacle_y[i];
        }
        obstree = new flann::Index<flann::L2<int>>(dataset, flann::KDTreeIndexParams(4));
        obstree->buildIndex();

        // 检查路径文件是否存在
        std::vector<std::vector<int>> path;
        std::ifstream path_file("build/teb/path.csv");
        if (!path_file.is_open()) {
            printf("Path file not found! Run A_star.\n");
            path = A_star(start_x, start_y, goal_x, goal_y);
            if (path.empty()) {
                std::cerr << "No path found!" << std::endl;
                return {{}, {}, 0};
            }
            std::ofstream path_file_out("build/teb/path.csv");
            for (const auto& point : path) {
                path_file_out << point[0] << "," << point[1] << "\n";
            }
        } else {
            printf("Path file found! Read path from file.\n");
            std::string line;
            while (std::getline(path_file, line)) {
                std::istringstream ss(line);
                std::string token;
                std::vector<int> point;
                while (std::getline(ss, token, ',')) {
                    point.push_back(std::stoi(token));
                }
                path.push_back(point);
            }
        }

        std::vector<int> path_x, path_y;
        for (const auto& point : path) {
            path_x.push_back(point[0]);
            path_y.push_back(point[1]);
        }

        return {path_x, path_y, 1};
    }

    std::vector<std::vector<int>> A_star(int start_x, int start_y, int goal_x, int goal_y) {
        Node* start_position = new Node(start_x, start_y, nullptr);
        openlist.push_back(start_position);
        std::vector<std::vector<int>> path_list;

        for (int i = 0; i < LIMIT_TRIAL; ++i) {
            Node* point = is_final(goal_x, goal_y);
            if (point) {
                while (point->parent != nullptr) {
                    path_list.push_back({point->x, point->y});
                    point = point->parent;
                }
                std::reverse(path_list.begin(), path_list.end());
                std::cout << "The path is found!" << std::endl;
                return path_list;
            }

            if (openlist.empty()) {
                return {};
            }

            Node* point_minF = ifFmin();
            closelist.push_back(point_minF);
            openlist.erase(std::remove(openlist.begin(), openlist.end(), point_minF), openlist.end());

            search_path(new Node(point_minF->x - step_length, point_minF->y, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x - step_length, point_minF->y - step_length, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x, point_minF->y - step_length, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x + step_length, point_minF->y - step_length, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x + step_length, point_minF->y, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x + step_length, point_minF->y + step_length, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x, point_minF->y + step_length, point_minF), goal_x, goal_y);
            search_path(new Node(point_minF->x - step_length, point_minF->y + step_length, point_minF), goal_x, goal_y);
        }

        return {};
    }

    void search_path(Node* node, int goal_x, int goal_y) {
        if (check_obs(node->x, node->y, *obstree)) {
            return;
        }

        if (isCloseList(node->x, node->y)) {
            return;
        }

        node->G = node->parent->G + step_length;
        node->H = std::abs(node->x - goal_x) + std::abs(node->y - goal_y);

        Node* point = isOpenList(node->x, node->y);
        if (point) {
            if ((node->G + node->H) <= (point->G + point->H)) {
                point = node;
            }
        } else {
            openlist.push_back(node);
        }
    }

    bool check_obs(int node_x, int node_y, flann::Index<flann::L2<int>>& tree) {
        std::vector<int> query = {node_x, node_y};
        std::vector<int> indices(1);
        std::vector<float> dists(1);
        const flann::Matrix<int> query_mat(&query[0], 1, 2); 
        flann::Matrix<int> indices_mat(&indices[0], 1, 1);
        flann::Matrix<float> dists_mat(&dists[0], 1, 1); 

        tree.knnSearch(query_mat, indices_mat, dists_mat, 1, flann::SearchParams(128));

        if (indices[0] == 0) {
            std::cerr << "Warning: The robot is out of the map!" << std::endl;
            return false;
        }

        if (dists[0] > MAX_EDGE_LEN) {
            return true;
        }

        int step_size = robot_size + avoid_dist;
        int steps = std::round(dists[0] / step_size);
        for (int i = 0; i < steps; ++i) {
            if (dists[0] <= robot_size + avoid_dist) {
                return true;
            }
        }

        if (dists[0] <= step_size) {
            return true;
        }

        return false;
    }

    Node* is_final(int goal_x, int goal_y) {
        for (Node* node : closelist) {
            if (node->x == goal_x && node->y == goal_y) {
                return node;
            }
        }
        return nullptr;
    }

    Node* ifFmin() {
        return *std::min_element(openlist.begin(), openlist.end(), [](Node* a, Node* b) {
            return (a->G + a->H) < (b->G + b->H);
        });
    }

    bool isCloseList(int x, int y) {
        for (Node* node : closelist) {
            if (node->x == x && node->y == y) {
                return true;
            }
        }
        return false;
    }

    Node* isOpenList(int x, int y) {
        for (Node* node : openlist) {
            if (node->x == x && node->y == y) {
                return node;
            }
        }
        return nullptr;
    }
};
