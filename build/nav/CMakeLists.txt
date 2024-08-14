cmake_minimum_required(VERSION 3.0.2)
project(nav)


set(LZ4_INCLUDE_DIR "/usr/include")
set(LZ4_LIBRARY "/usr/lib/liblz4.so")


include_directories(${LZ4_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)



include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  #/home/nvidia/dora_project/dora/apis/c/node
  $ENV{HOME}/dora/apis/c/operator
  
)


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

