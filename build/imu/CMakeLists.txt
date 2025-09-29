cmake_minimum_required(VERSION 3.17)
project(imu)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)


#find_package(SERIAL REQUIRED serial)

# 包含目录
include_directories(
    include
    ${BOOST_INCLUDEDIR}
    ${SERIAL_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIRS}
    $ENV{HOME}/dora/apis/c/node # dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator
    /usr/local/lib
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")

# 链接目录
link_directories(
    ${BOOST_LIBRARYDIR}
    /usr/local/lib
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
)
