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

    Astar_DWA(int MAX_EDGE_LEN = 2*800*800, int LIMIT_TRIAL = 50000000) 
        : MAX_EDGE_LEN(MAX_EDGE_LEN), LIMIT_TRIAL(LIMIT_TRIAL), minx(0), maxx(800), miny(0), maxy(800),
          robot_size(4), avoid_dist(4), r(10), obstree(nullptr), step_length(5), pathPointInterval(20) {}

    std::tuple<std::vector<int>, std::vector<int>, int> plan(int start_x, int start_y, double start_angle, int goal_x, int goal_y, double goal_angle) {
        // 初始化障碍物坐标数组
        std::vector<int> obstacle_x, obstacle_y;

        // 打开PGM文件
        std::string image_path = "/home/sunny/dora_nav/build/nav/laser_data.pgm";
        std::ifstream file(image_path);
        if (!file) {
            std::cerr << "Failed to open image file: " << image_path << std::endl;
            return {{}, {}, 0};
        }
        printf("Open image file successfully!\n");
        // 读取PGM文件头
        std::string line, magic_number;
        int width, height, max_val;

        // 读取 PGM 文件头
        std::getline(file, magic_number);
        if (magic_number != "P2") {
            std::cerr << "Unsupported PGM format: " << magic_number << std::endl;
            return {{}, {}, 0};
        }

        // 跳过注释行
        std::getline(file, line);
        while (line[0] == '#') {
            std::cout << "line: " << line << std::endl;
            std::getline(file, line);
        }
        std::cout << "line: " << line << std::endl;
        // 读取宽度、高度和最大灰度值
        std::istringstream iss(line);
        iss >> width >> height;
        file >> max_val;
        file.ignore(1); // 跳过一个字符（通常是换行符）

        // 读取图像数据
        std::vector<uint8_t> img_data(width * height);
        for (int i = 0; i < width * height; ++i) {
            int pixel;
            file >> pixel;
            img_data[i] = static_cast<uint8_t>(pixel);
        }

        printf("Read image data successfully!\n");
        printf("width: %d, height: %d\n", width, height);
        // 遍历图像中的每个像素
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if(x<start_x+step_length*8 && x>start_x-step_length*8 && y<start_y+step_length*8 && y>start_y-step_length*8){
                    img_data[y * width + x] = 255;
                    continue;
                }
                if (img_data[y * width + x] < 254/2 ) {
                    obstacle_x.push_back(x);
                    obstacle_y.push_back(y);
                    img_data[y * width + x] = 0;
                    //if(x==330 && y==300){
                    //    printf("error");
                    //}
                    //printf("x: %d, y: %d\n", x, y);
                }else{
                    img_data[y * width + x] = 255;
                }
            }
        }
        img_data[start_y * width + start_x] = 0;
        img_data[goal_y * width + goal_x] = 0;
        std::ofstream output_file("/home/sunny/dora_nav/build/nav/output.pgm");
        output_file << "P2\n" << width << " " << height << " 255\n";
        for (int i = 0; i < width * height; ++i) {
            output_file << static_cast<int>(img_data[i]) << " ";
            if ((i + 1) % width == 0) {
                output_file << "\n";
            }
        }
        output_file.close();    

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
        std::ifstream path_file("/home/sunny/dora_nav/build/teb/path.csv");
        if (!path_file.is_open()) {
            printf("Path file not found! Run A_star.\n");
            path = A_star(start_x, start_y, goal_x, goal_y);
            if (path.empty()) {
                std::cerr << "No path found!" << std::endl;
                return {{}, {}, 0};
            }
            std::ofstream path_file_out("/home/sunny/dora_nav/build/teb/path.csv");
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
            img_data[point[1] * width + point[0]] = 0;

            path_x.push_back(point[0]);
            path_y.push_back(point[1]);
        }
        std::ofstream output_file2("/home/sunny/dora_nav/build/nav/output2.pgm");
        output_file2 << "P2\n" << width << " " << height << " 255\n";
        for (int i = 0; i < width * height; ++i) {
            output_file2 << static_cast<int>(img_data[i]) << " ";
            if ((i + 1) % width == 0) {
                output_file2 << "\n";
            }
        }
        output_file2.close();  

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
        //printf("x: %d, y: %d\n", node->x, node->y);
        if (check_obs(node->x, node->y, *obstree)) {
            //printf("check_obs failed!\n");
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
            //printf("@x: %d, y: %d\n", node->x, node->y);
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
        //printf("indices[0]: %d, dists[0]: %f\n", indices[0], dists[0]);
        //printf("node_x: %d, node_y: %d\n", node_x, node_y);
        //printf("obstacle_x: %d, obstacle_y: %d\n", tree.getPoint(indices[0])[0], tree.getPoint(indices[0])[1]);
/*         if (indices[0] == 0) {
            std::cerr << "Warning: The robot is out of the map!" << std::endl;
            return false;
        } */

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
