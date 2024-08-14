set(amcl_map_src_
map.c
map_cspace.cpp
map_draw.c
map_range.c
map_store.c
)



foreach(a IN ITEMS ${amcl_map_src_})
  set(amcl_map_src_abs ${amcl_map_src_abs} ${CMAKE_CURRENT_SOURCE_DIR}/${a})
endforeach()

set(amcl_map_src ${amcl_map_src_abs} PARENT_SCOPE)





		 
