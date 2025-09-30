#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <fstream>
#include <iostream>

// 统一的地图配置管理器
class MapManager {
public:
    struct MapConfig {
        std::string image_file;
        double resolution;          // m/pixel
        std::vector<double> origin; // [x, y, theta]
        int width;                  // pixels
        int height;                 // pixels
        double occupied_thresh;
        double free_thresh;
        bool negate;
        
        // 扩展信息
        double map_size_meters_x;
        double map_size_meters_y;
        std::string map_frame;
        
        MapConfig() : 
            resolution(0.005), 
            origin({0.0, 0.0, 0.0}),
            width(800), 
            height(800),
            occupied_thresh(0.65),
            free_thresh(0.196),
            negate(false),
            map_size_meters_x(4.0),
            map_size_meters_y(4.0),
            map_frame("map") {}
    };

private:
    static std::shared_ptr<MapManager> instance_;
    MapConfig config_;
    bool is_loaded_;
    std::string current_map_file_;

    MapManager() : is_loaded_(false) {}

public:
    // 单例模式
    static std::shared_ptr<MapManager> getInstance() {
        if (!instance_) {
            instance_ = std::shared_ptr<MapManager>(new MapManager());
        }
        return instance_;
    }

    // 从YAML文件加载地图配置
    bool loadFromYaml(const std::string& yaml_file) {
        try {
            YAML::Node yaml_node = YAML::LoadFile(yaml_file);
            
            config_.image_file = yaml_node["image"].as<std::string>();
            config_.resolution = yaml_node["resolution"].as<double>();
            
            if (yaml_node["origin"]) {
                config_.origin = yaml_node["origin"].as<std::vector<double>>();
            }
            
            config_.occupied_thresh = yaml_node["occupied_thresh"].as<double>(0.65);
            config_.free_thresh = yaml_node["free_thresh"].as<double>(0.196);
            config_.negate = yaml_node["negate"].as<bool>(false);
            
            // 从元数据获取尺寸信息
            if (yaml_node["map_size_pixels"]) {
                config_.width = config_.height = yaml_node["map_size_pixels"].as<int>();
            }
            
            if (yaml_node["map_size_meters"]) {
                config_.map_size_meters_x = config_.map_size_meters_y = 
                    yaml_node["map_size_meters"].as<double>();
            } else {
                // 根据分辨率和像素尺寸计算
                config_.map_size_meters_x = config_.width * config_.resolution;
                config_.map_size_meters_y = config_.height * config_.resolution;
            }
            
            current_map_file_ = yaml_file;
            is_loaded_ = true;
            
            std::cout << "[MapManager] ✅ 地图配置加载成功: " << yaml_file << std::endl;
            std::cout << "[MapManager] 分辨率: " << config_.resolution << " m/pixel" << std::endl;
            std::cout << "[MapManager] 尺寸: " << config_.width << "x" << config_.height << " pixels" << std::endl;
            std::cout << "[MapManager] 实际尺寸: " << config_.map_size_meters_x << "x" << config_.map_size_meters_y << " meters" << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "[MapManager] ❌ 地图配置加载失败: " << e.what() << std::endl;
            return false;
        }
    }

    // 自动搜索并加载地图配置
    bool autoLoadMapConfig() {
        // 按优先级搜索地图配置文件
        std::vector<std::string> search_paths = {
            "build/slam/laser_data.datslam_map.yaml",          // SLAM生成的地图
            "build/nav/laser_data.yaml",                       // NAV模块地图
            "build/simulation_map_800x800.yaml",               // 仿真地图
            "/home/sunny/dora_nav/build/slam/laser_data.datslam_map.yaml",
            "/home/sunny/dora_nav/build/nav/laser_data.yaml",
            "/home/sunny/dora_nav/build/simulation_map_800x800.yaml"
        };
        
        for (const auto& path : search_paths) {
            std::ifstream file(path);
            if (file.good()) {
                std::cout << "[MapManager] 尝试加载: " << path << std::endl;
                if (loadFromYaml(path)) {
                    return true;
                }
            }
        }
        
        std::cout << "[MapManager] ⚠️ 未找到地图配置文件，使用默认配置" << std::endl;
        useDefaultConfig();
        return false;
    }

    // 使用默认配置
    void useDefaultConfig() {
        config_ = MapConfig(); // 使用默认构造函数的值
        is_loaded_ = true;
        current_map_file_ = "default";
        
        std::cout << "[MapManager] 使用默认地图配置" << std::endl;
        std::cout << "[MapManager] 分辨率: " << config_.resolution << " m/pixel" << std::endl;
        std::cout << "[MapManager] 尺寸: " << config_.width << "x" << config_.height << " pixels" << std::endl;
    }

    // 获取配置
    const MapConfig& getConfig() const {
        if (!is_loaded_) {
            throw std::runtime_error("地图配置未加载，请先调用 autoLoadMapConfig() 或 loadFromYaml()");
        }
        return config_;
    }

    // 坐标转换函数
    std::pair<int, int> worldToPixel(double x, double y) const {
        const auto& cfg = getConfig();
        
        int pixel_x = static_cast<int>((x - cfg.origin[0]) / cfg.resolution);
        int pixel_y = static_cast<int>((y - cfg.origin[1]) / cfg.resolution);
        
        // 如果原点在地图中心，需要偏移
        if (cfg.origin[0] == 0.0 && cfg.origin[1] == 0.0) {
            pixel_x += cfg.width / 2;
            pixel_y = cfg.height / 2 - pixel_y; // Y轴翻转
        }
        
        // 边界检查
        pixel_x = std::max(0, std::min(pixel_x, cfg.width - 1));
        pixel_y = std::max(0, std::min(pixel_y, cfg.height - 1));
        
        return {pixel_x, pixel_y};
    }

    std::pair<double, double> pixelToWorld(int pixel_x, int pixel_y) const {
        const auto& cfg = getConfig();
        
        double x, y;
        
        // 如果原点在地图中心
        if (cfg.origin[0] == 0.0 && cfg.origin[1] == 0.0) {
            x = (pixel_x - cfg.width / 2) * cfg.resolution;
            y = (cfg.height / 2 - pixel_y) * cfg.resolution; // Y轴翻转
        } else {
            x = pixel_x * cfg.resolution + cfg.origin[0];
            y = pixel_y * cfg.resolution + cfg.origin[1];
        }
        
        return {x, y};
    }

    // 获取地图信息的JSON格式（用于Dora消息）
    std::string getMapInfoJson() const {
        const auto& cfg = getConfig();
        
        nlohmann::json map_info;
        map_info["image_file"] = cfg.image_file;
        map_info["resolution"] = cfg.resolution;
        map_info["origin"] = cfg.origin;
        map_info["width"] = cfg.width;
        map_info["height"] = cfg.height;
        map_info["occupied_thresh"] = cfg.occupied_thresh;
        map_info["free_thresh"] = cfg.free_thresh;
        map_info["negate"] = cfg.negate;
        map_info["map_size_meters_x"] = cfg.map_size_meters_x;
        map_info["map_size_meters_y"] = cfg.map_size_meters_y;
        map_info["map_frame"] = cfg.map_frame;
        map_info["source_file"] = current_map_file_;
        
        return map_info.dump();
    }

    // 检查是否已加载
    bool isLoaded() const { return is_loaded_; }
    
    // 获取当前地图文件路径
    const std::string& getCurrentMapFile() const { return current_map_file_; }
};

// 静态成员定义
std::shared_ptr<MapManager> MapManager::instance_ = nullptr;

// 便捷的全局访问函数
inline std::shared_ptr<MapManager> getMapManager() {
    return MapManager::getInstance();
}

#endif // MAP_MANAGER_H

