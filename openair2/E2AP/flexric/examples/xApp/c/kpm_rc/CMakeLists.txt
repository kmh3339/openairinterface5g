add_executable(xapp_kpm_rc 
  xapp_kpm_rc.c
  ../../../../src/util/alg_ds/alg/defer.c
  )
 
target_link_libraries(xapp_kpm_rc
                      PUBLIC
                      e42_xapp
                      -pthread
                      -lsctp
                      -ldl
                      )
                      
add_executable(xapp_kpm_rc_prb 
  xapp_kpm_rc_prb.c
  ../../../../src/util/alg_ds/alg/defer.c
  )
 
target_link_libraries(xapp_kpm_rc_prb
                      PUBLIC
                      e42_xapp
                      -pthread
                      -lsctp
                      -ldl
                      )
                      
