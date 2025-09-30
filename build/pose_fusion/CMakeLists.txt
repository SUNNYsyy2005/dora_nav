cmake_minimum_required(VERSION 3.0.2)
project(pose_fusion)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

# 包含目录配置
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${LZ4_INCLUDE_DIR})
include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator
)

# 源文件配置
set(src_pose_fusion main.cc)

# 编译选项
add_compile_options(-std=c++17 -Wall -Wextra)

# 创建可执行文件
add_executable(test ${src_pose_fusion})

# 链接库
target_link_libraries(
  test
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  ${LZ4_LIBRARY}
  ${OPENCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  pthread
  rt
  dl
)
