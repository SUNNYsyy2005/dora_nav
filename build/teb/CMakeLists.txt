cmake_minimum_required(VERSION 3.17)
project(teb)

# set(CMAKE_CXX_STANDARD 17)
# SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

# 手动指定库和包含目录
include_directories(
    include
    /usr/include/eigen3
    /usr/include/suitesparse
    /usr/include/g2o
    /usr/include/opencv4
    ${YAMLCPP_INCLUDE_DIRS}
    $ENV{HOME}/dora/apis/c/node  # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
    ${CMAKE_CURRENT_SOURCE_DIR}/c # 添加c文件夹作为include目录
)

# 手动指定库的位置
set(G2O_LIBRARIES
    /usr/lib/libg2o_core.so
    /usr/lib/libg2o_stuff.so
    /usr/lib/libg2o_types_slam2d.so
    /usr/lib/libg2o_types_slam3d.so
    /usr/lib/libg2o_solver_cholmod.so
    /usr/lib/libg2o_solver_pcg.so
    /usr/lib/libg2o_solver_csparse.so
)

set(SUITESPARSE_LIBRARIES
    /usr/lib/libsuitesparseconfig.so
    /usr/lib/libcholmod.so
    /usr/lib/libccolamd.so
    /usr/lib/libcolamd.so
    /usr/lib/libcamd.so
    /usr/lib/libamd.so
)

set(OpenCV_LIBS
    /usr/lib/libopencv_core.so
    /usr/lib/libopencv_imgproc.so
    /usr/lib/libopencv_highgui.so
    /usr/lib/libopencv_imgcodecs.so
)

set(Boost_LIBRARIES
    /usr/lib/libboost_system.so
    /usr/lib/libboost_thread.so
    /usr/lib/libboost_graph.so
)

set(FMT_LIBRARIES
    /usr/lib/libfmt.so
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
)
