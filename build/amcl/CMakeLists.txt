cmake_minimum_required(VERSION 3.0.2)
project(amcl)

# 包含统一配置文件
include(${CMAKE_SOURCE_DIR}/../../cmake_config.cmake)

add_subdirectory(map)
add_subdirectory(pf)
add_subdirectory(sensors)

# 包含目录配置
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  #/home/nvidia/dora_project/dora/apis/c/node
  $ENV{HOME}/dora/apis/c/operator
  
)

set(amcl_dependencies)

add_library(amcl ${amcl_map_src} ${amcl_pf_src} ${amcl_sensors_src})

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
  pthread
  rt
  dl
  ${OpenCV_LIBS}
)

