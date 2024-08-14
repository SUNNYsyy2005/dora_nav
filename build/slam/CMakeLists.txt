cmake_minimum_required(VERSION 3.0.2)
project(slam)

# Include directories
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node  # dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator
  ${CMAKE_CURRENT_SOURCE_DIR}  # 添加当前目录包含头文件
  ${CMAKE_CURRENT_SOURCE_DIR}/c
)

# 添加源文件
set(SRC_FILES
  algorithms.cpp
  Map.cpp
  Scan.cpp
  WheeledRobot.cpp
)

# Add executable for 'test'
set(src_test main.cc)
add_executable(test ${src_test} ${SRC_FILES})
target_link_libraries(
  test
  ${CMAKE_CURRENT_SOURCE_DIR}/slambuild/libbreezyslam.so  # 链接本地的libbreezyslam库
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  pthread
  rt
  dl
)

# Add executable for 'log2pgm'
set(src_log2pgm log2pgm.cc)
add_executable(log2pgm ${src_log2pgm} ${SRC_FILES})
target_link_libraries(
  log2pgm
  ${CMAKE_CURRENT_SOURCE_DIR}/slambuild/libbreezyslam.so  # 链接本地的libbreezyslam库
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  pthread
  rt
  dl
)

