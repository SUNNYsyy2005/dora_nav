set(amcl_sensors_src_
  amcl_laser.cpp
  amcl_odom.cpp
  amcl_sensor.cpp
)


foreach(a IN ITEMS ${amcl_sensors_src_})
  set(amcl_sensors_src_abs ${amcl_sensors_src_abs} ${CMAKE_CURRENT_SOURCE_DIR}/${a})
endforeach()

set(amcl_sensors_src ${amcl_sensors_src_abs} PARENT_SCOPE)





