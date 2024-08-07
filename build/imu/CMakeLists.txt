cmake_minimum_required(VERSION 3.17)
project(imu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# 设置 Boost 的路径
set(BOOST_INCLUDEDIR "/opt/boost-1.85.0-ubuntu-x64/include")
set(BOOST_LIBRARYDIR "/opt/boost-1.85.0-ubuntu-x64/lib")


# 包含目录
include_directories(
    include
    ${BOOST_INCLUDEDIR}
    $ENV{HOME}/dora/apis/c/node # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
    /usr/local/lib
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")

# 链接目录
link_directories(
    ${BOOST_LIBRARYDIR}
)
find_package(Threads REQUIRED)

# 编译可执行文件 test，并链接 lslidar_cx_driver 库
add_executable(test main.cc)
target_link_libraries(test
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
    ${Boost_LIBRARIES}
    boost_thread
    Threads::Threads # 明确链接到 pthread
    rt # 添加 rt 库
    dl # 添加 dl 库
    serial # 添加 serial 库
)