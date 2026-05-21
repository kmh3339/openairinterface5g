add_executable(xapp_kpm_moni_yu 
  xapp_kpm_moni_yu.c
  ../../../../src/util/alg_ds/alg/defer.c
  )
 
target_link_libraries(xapp_kpm_moni_yu
                      PUBLIC
                      e42_xapp
                      hiredis
                      -pthread
                      -lsctp
                      -ldl
                      )
                      
add_executable(xapp_kpm_rc_yu 
  xapp_kpm_rc_yu.c
  ../../../../src/util/alg_ds/alg/defer.c
  )
 
target_link_libraries(xapp_kpm_rc_yu
                      PUBLIC
                      e42_xapp
                      -pthread
                      -lsctp
                      -ldl
                      )
                      
