cmake_minimum_required(VERSION 3.0.2)
project(slam)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

# Include目录配置
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node  # dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator
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

