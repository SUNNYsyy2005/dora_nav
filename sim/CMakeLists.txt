 cmake_minimum_required(VERSION 3.16)
project(test_sim_data)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加包含目录
include_directories(${CMAKE_SOURCE_DIR}/../include)
include_directories(${CMAKE_SOURCE_DIR}/../../../include)

# 查找依赖
find_package(PkgConfig REQUIRED)

# OpenCV
pkg_check_modules(OpenCV REQUIRED opencv4)
include_directories(${OpenCV_INCLUDE_DIRS})

# nlohmann/json
find_package(nlohmann_json REQUIRED)

# 包含配置文件
include(/home/sunny/dora_nav/cmake_config.cmake)

# 添加nlohmann/json包含路径
include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})

# 添加源文件
add_executable(test_sim_data_node test_sim_data_node.cc)

# 链接库
target_link_libraries(test_sim_data_node 
    ${NLOHMANN_JSON_LIBRARIES}
    ${DORA_LIB_PATH}/libdora_node_api_c.a
    pthread
    rt
    dl
    ${OpenCV_LIBRARIES}
)
