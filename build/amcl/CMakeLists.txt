cmake_minimum_required(VERSION 3.0.2)
project(amcl)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amcl
  CATKIN_DEPENDS roscpp std_msgs genmsg
)

add_subdirectory(map)
add_subdirectory(pf)
add_subdirectory(sensors)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(amcl_dependencies )

add_library(amcl ${amcl_map_src} ${amcl_pf_src} ${amcl_sensors_src})
target_link_libraries(amcl ${amcl_dependencies})
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_compile_definitions(amcl PUBLIC HAVE_DRAND48)
endif ()

set(src_test test.cpp)
add_executable(test ${src_test})
target_link_libraries(test amcl ${catkin_LIBRARIES})
