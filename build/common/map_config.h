#ifndef MAP_CONFIG_H
#define MAP_CONFIG_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// 地图配置类 - 用于读取YAML地图元数据
class MapConfig {
private:
    std::string image_path;
    double resolution;
    double origin_x, origin_y, origin_theta;
    bool negate;
    double occupied_thresh;
    double free_thresh;
    
    bool parse_yaml_line(const std::string& line) {
        std::istringstream iss(line);
        std::string key, value;
        
        // 跳过注释行和空行
        if (line.empty() || line[0] == '#') {
            return true;
        }
        
        // 解析键值对
        if (getline(iss, key, ':') && getline(iss, value)) {
            // 去除前后空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            if (key == "image") {
                image_path = value;
            } else if (key == "resolution") {
                resolution = std::stod(value);
            } else if (key == "origin") {
                // 解析原点："[x, y, theta]"
                value = value.substr(1, value.length() - 2); // 去掉方括号
                std::istringstream origin_stream(value);
                std::string x_str, y_str, theta_str;
                
                getline(origin_stream, x_str, ',');
                getline(origin_stream, y_str, ',');
                getline(origin_stream, theta_str);
                
                origin_x = std::stod(x_str);
                origin_y = std::stod(y_str);
                origin_theta = std::stod(theta_str);
            } else if (key == "negate") {
                negate = (value == "1");
            }
            else if (key == "occupied_thresh") {
                occupied_thresh = std::stod(value);
            }
            else if (key == "free_thresh") {
                free_thresh = std::stod(value);
            }
        }
        
        return true;
    }
    
public:
    MapConfig() : resolution(0.04), origin_x(0.0), origin_y(0.0), origin_theta(0.0),
                  negate(false), occupied_thresh(0.65), free_thresh(0.196) {}
    
    // 从YAML文件加载配置
    bool loadFromFile(const std::string& yaml_file) {
        std::ifstream file(yaml_file);
        if (!file.is_open()) {
            std::cerr << "无法打开YAML配置文件: " << yaml_file << std::endl;
            return false;
        }
        
        std::string line;
        while (getline(file, line)) {
            parse_yaml_line(line);
        }
        file.close();
        
        std::cout << "地图配置加载成功:" << std::endl;
        std::cout << "  图像路径: " << image_path << std::endl;
        std::cout << "  分辨率: " << resolution << " m/pixel" << std::endl;
        std::cout << "  原点: (" << origin_x << ", " << origin_y << ", " << origin_theta << ")" << std::endl;
        std::cout << "  阈值 - 占用:" << occupied_thresh << ", 自由:" << free_thresh << std::endl;
        
        return true;
    }
    
    // 保存YAML配置文件
    bool saveToFile(const std::string& yaml_file, const std::string& image_file) const {
        std::ofstream file(yaml_file);
        if (!file.is_open()) {
            std::cerr << "无法创建YAML配置文件: " << yaml_file << std::endl;
            
            return false;
        }
        
        file << "image: " << image_file << std::endl;
        file << "resolution: " << resolution << std::endl;
        file << "origin: [" << origin_x << ", " << origin_y << ", " << origin_theta << "]" << std::endl;
        file << "negate: " << (negate ? 1 : 0) << std::endl;
        file << "occupied_thresh: " << occupied_thresh << std::endl;
        file << "free_thresh: " << free_thresh << std::endl;
        
        file.close();
        std::cout << "地图配置已保存到: " << yaml_file << std::endl;
        
        return true;
    }
    
    // 获取图像文件路径（相对于YAML文件的完整路径）
    std::string getImagePath(const std::string& yaml_dir = "") const {
        if (image_path.empty()) return "";
        
        // 如果是绝对路径，直接返回
        if (image_path[0] == '/') {
            return image_path;
        }
        
        // 相对路径，需要与YAML文件目录组合
        if (yaml_dir.empty()) {
            return image_path;
        }
        
        // 组合目录路径
        std::string dir = yaml_dir;
        if (dir.back() != '/') {
            dir += '/';
        }
        
        return dir + image_path;
    }
    
    // 基本地图信息
    double getResolution() const { return resolution; }
    void setResolution(double res) { resolution = res; }
    
    double getOriginX() const { return origin_x; }
    double getOriginY() const { return origin_y; }
    double getOriginTheta() const { return origin_theta; }
    
    void setOrigin(double x, double y, double theta) {
        origin_x = x; origin_y = y; origin_theta = theta;
    }
    
    bool getNegate() const { return negate; }
    void setNegate(bool neg) { negate = neg; }
    
    double getOccupiedThresh() const { return occupied_thresh; }
    void setOccupiedThresh(double thresh) { occupied_thresh = thresh; }
    
    double getFreeThresh() const { return free_thresh; }
    void setFreeThresh(double thresh) { free_thresh = thresh; }
    
    // 图像尺寸和坐标转换
    int pixelsToMeters(int pixels) const {
        return static_cast<int>(pixels * resolution);
    }
    
    double pixelsToMetersDouble(int pixels) const {
        return pixels * resolution;
    }
    
    int metersToPixels(double meters) const {
        return static_cast<int>(meters / resolution);
    }
    
    // 根据像素坐标计算世界坐标（考虑原点偏移）
    std::pair<double, double> pixelToWorld(int x_pixel, int y_pixel) const {
        double world_x = origin_x + x_pixel * resolution;
        double world_y = origin_y + (800 - y_pixel) * resolution; // Y轴翻转
        return std::make_pair(world_x, world_y);
    }
    
    // 根据世界坐标计算像素坐标（考虑原点偏移）
    std::pair<int, int> worldToPixel(double world_x, double world_y) const {
        int x_pixel = static_cast<int>((world_x - origin_x) / resolution);
        int y_pixel = 800 - static_cast<int>((world_y - origin_y) / resolution);
        return std::make_pair(x_pixel, y_pixel);
    }
};

#endif // MAP_CONFIG_H

