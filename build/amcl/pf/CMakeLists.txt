set(amcl_pf_src_
pf.c
pf_draw.c
pf_kdtree.c
pf_pdf.c
eig3.c
pf_vector.c
)



foreach(a IN ITEMS ${amcl_pf_src_})
  set(amcl_pf_src_abs ${amcl_pf_src_abs} ${CMAKE_CURRENT_SOURCE_DIR}/${a})
endforeach()

set(amcl_pf_src ${amcl_pf_src_abs} PARENT_SCOPE)





