cmake_minimum_required(VERSION 3.17)
project(teb)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 包含目录配置
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/c
    ${EIGEN_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    $ENV{HOME}/dora/apis/c/node  # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
    ${FMT_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIRS}
)

# 添加teb_local_planner的库
add_library(lib_teb
    src/teb_config.cpp
    src/obstacles.cpp
    src/visualization.cpp
    src/optimal_planner.cpp
    src/timed_elastic_band.cpp
)

# 添加可执行文件
add_executable(test main.cc)

# 链接库
target_link_libraries(
    test
    lib_teb
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
    ${SUITESPARSE_LIBRARIES}
    ${G2O_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${FMT_LIBRARIES}
    rt
    dl
    pthread
    cxsparse  # csparse库
)
