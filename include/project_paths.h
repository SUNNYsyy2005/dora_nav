#ifndef PROJECT_PATHS_H
#define PROJECT_PATHS_H

#include <string>
#include <string.h>
#include <unistd.h>

// 获取项目根目录的绝对路径
inline std::string get_project_root() {
    // 从环境变量获取项目根目录，如果没有则使用默认路径
    const char* env_root = getenv("DORA_NAV_ROOT");
    if (env_root != nullptr) {
        return std::string(env_root);
    }
    
    // 尝试从当前执行文件路径推导项目根目录
    // 假设执行文件在 build/[module]/ 或 build/[module]/build/ 目录下
    char path[1024];
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (len != -1) {
        path[len] = '\0';
        std::string exe_path(path);
        
        // 查找 "/dora_nav" 目录
        size_t pos = exe_path.find("/dora_nav");
        if (pos != std::string::npos) {
            return exe_path.substr(0, pos + strlen("/dora_nav"));
        }
    }
    
    // 默认回退路径
    return "/home/sunny/dora_nav";
}

// 构建文件路径的辅助函数
inline std::string build_path(const std::string& relative_path) {
    return get_project_root() + "/" + relative_path;
}

// 常用的项目路径
namespace ProjectPaths {
    inline std::string build_nav_laser_data() {
        return build_path("build/nav/laser_data.pgm");
    }
    
    inline std::string build_teb_path_csv() {
        return build_path("build/teb/path.csv");
    }
    
    inline std::string build_slam_data() {
        return build_path("build/slam/laser_data.dat");
    }
    
    inline std::string build_nav_data() {
        return build_path("build/nav/");
    }
    
    inline std::string amcl_txt() {
        return build_path("amcl.txt");
    }
    
    inline std::string teb_txt() {
        return build_path("teb.txt");
    }
    
    inline std::string nav_output_pgm() {
        return build_path("build/nav/output.pgm");
    }
    
    inline std::string nav_output2_pgm() {
        return build_path("build/nav/output2.pgm");
    }
    
    inline std::string nav_data_pgm(const std::string& dataset) {
        return build_path("build/nav/" + dataset + ".pgm");
    }
    
    inline std::string nav_data_yaml(const std::string& dataset) {
        return build_path("build/nav/" + dataset + ".yaml");
    }
    
    inline std::string amcl_output_pgm() {
        return build_path("build/amcl/output.pgm");
    }
}


#endif // PROJECT_PATHS_H
