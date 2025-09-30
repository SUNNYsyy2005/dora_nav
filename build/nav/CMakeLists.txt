cmake_minimum_required(VERSION 3.0.2)
project(nav)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

# 包含目录配置
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${LZ4_INCLUDE_DIR})
include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  #/home/nvidia/dora_project/dora/apis/c/node
  $ENV{HOME}/dora/apis/c/operator
  
)


# Add executable for 'test' (原始版本)
set(src_test main.cc)
add_executable(test ${src_test})
target_link_libraries(
  test
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  ${LZ4_LIBRARY}
  pthread
  rt
  dl
)

# Add executable for 'test_dora' (Dora节点版本)
set(src_dora main_dora.cc)
add_executable(test_dora ${src_dora})
target_link_libraries(
  test_dora
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  ${LZ4_LIBRARY}
  ${OPENCV_LIBRARIES}
  pthread
  rt
  dl
)

