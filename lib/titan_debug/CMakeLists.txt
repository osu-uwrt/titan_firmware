add_library(titan_debug INTERFACE)

target_sources(titan_debug INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/debug_server.c
    ${CMAKE_CURRENT_LIST_DIR}/src/remote_cmd_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/src/global_remote_cmds.c
)

target_include_directories(titan_debug INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(titan_debug INTERFACE
    canmore
    hardware_watchdog
)
