cmake_minimum_required(VERSION 3.17)
project(teb)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 手动指定库和包含目录
include_directories(
    include
    /usr/include/eigen3
    /usr/local/include/g2o
    /usr/include/opencv4
    /usr/include/suitesparse  # SuiteSparse 头文件路径
    ${YAMLCPP_INCLUDE_DIRS}
    $ENV{HOME}/dora/apis/c/node  # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
    ${CMAKE_CURRENT_SOURCE_DIR}/c # 添加c文件夹作为include目录
)

# 手动指定库的位置
set(G2O_LIBRARIES
    /usr/local/lib/libg2o_core.so
    /usr/local/lib/libg2o_stuff.so
    /usr/local/lib/libg2o_types_slam2d.so
    /usr/local/lib/libg2o_types_slam3d.so
    /usr/local/lib/libg2o_solver_cholmod.so
    /usr/local/lib/libg2o_solver_pcg.so
    /usr/local/lib/libg2o_solver_csparse.so
    /usr/local/lib/libg2o_csparse_extension.so
)

# SuiteSparse 库的位置
set(SUITESPARSE_LIBRARIES
    /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
    /usr/lib/x86_64-linux-gnu/libcholmod.so
    /usr/lib/x86_64-linux-gnu/libccolamd.so
    /usr/lib/x86_64-linux-gnu/libcolamd.so
    /usr/lib/x86_64-linux-gnu/libcamd.so
    /usr/lib/x86_64-linux-gnu/libamd.so
    /usr/lib/x86_64-linux-gnu/libcxsparse.so  # 添加这个库
)

set(OpenCV_LIBS
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
    /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
)

set(Boost_LIBRARIES
    /opt/boost-1.85.0-ubuntu-x64/lib/libboost_system.so
    /opt/boost-1.85.0-ubuntu-x64/lib/libboost_thread.so
    /opt/boost-1.85.0-ubuntu-x64/lib/libboost_graph.so
)

set(FMT_LIBRARIES
    /usr/lib/x86_64-linux-gnu/libfmt.a
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
    ${CMAKE_CURRENT_SOURCE_DIR}/libbreezyslam.so  # 链接本地的libbreezyslam库
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
    ${SUITESPARSE_LIBRARIES}
    ${G2O_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${FMT_LIBRARIES}
    pthread
    rt  # 添加 rt 库
    dl  # 添加 dl 库
)

# 找到并链接线程库
find_package(Threads REQUIRED)
target_link_libraries(test Threads::Threads)
