add_library(titan_debug INTERFACE)

target_sources(titan_debug INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/titan_debug.c
)

target_include_directories(titan_debug INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(titan_debug INTERFACE
    titan_canmore
)
