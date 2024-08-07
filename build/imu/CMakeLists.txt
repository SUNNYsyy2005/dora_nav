cmake_minimum_required(VERSION 3.17)
project(imu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# 设置 Boost 的路径
set(BOOST_INCLUDEDIR "/opt/boost-1.85.0-ubuntu-x64/include")
set(BOOST_LIBRARYDIR "/opt/boost-1.85.0-ubuntu-x64/lib")

# 设置 libpcap 的路径
set(PCAP_LIBRARY "-lpcap")
set(PCAP_INCLUDE_DIR "/usr/include")

# 包含目录
include_directories(
    include
    ${BOOST_INCLUDEDIR}
    ${PCAP_INCLUDE_DIR}
    /usr/include/boost/signals2
    $ENV{HOME}/dora/apis/c/node # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
)

# 链接目录
link_directories(
    ${BOOST_LIBRARYDIR}
)


# 找到线程库
find_package(Threads REQUIRED)

# 编译可执行文件 test，并链接 lslidar_cx_driver 库
add_executable(test main.cc)
target_link_libraries(test
    lslidar_cx_driver
    lslidar_cx_input
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
    ${Boost_LIBRARIES}
    boost_thread
    Threads::Threads # 明确链接到 pthread
    rt # 添加 rt 库
    dl # 添加 dl 库
)

# 确保 Boost 库被找到
find_package(Boost 1.85.0 REQUIRED COMPONENTS system thread)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()
