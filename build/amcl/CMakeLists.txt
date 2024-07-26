cmake_minimum_required(VERSION 3.0.2)
project(amcl)


add_subdirectory(map)
add_subdirectory(pf)
add_subdirectory(sensors)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)


set(amcl_dependencies )

add_library(amcl ${amcl_map_src} ${amcl_pf_src} ${amcl_sensors_src})
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  #/home/nvidia/dora_project/dora/apis/c/node
  $ENV{HOME}/dora/apis/c/operator
  
)

target_link_libraries(amcl ${amcl_dependencies})
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_compile_definitions(amcl PUBLIC HAVE_DRAND48)
endif ()

set(src_test main.cc)
add_executable(test ${src_test})
target_link_libraries(
  test
  amcl
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
)

