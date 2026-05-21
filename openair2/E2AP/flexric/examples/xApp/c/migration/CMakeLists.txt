add_executable(migration_action
  migration_action.c
  ../common/migration_function.c
)

target_include_directories(migration_action PRIVATE
  ../common
)

target_link_libraries(migration_action
  PUBLIC
  e42_xapp
  -pthread
  -lsctp
  -ldl
)

