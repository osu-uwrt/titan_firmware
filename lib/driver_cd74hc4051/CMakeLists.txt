add_library(driver_cd74hc4051 INTERFACE)

target_sources(driver_cd74hc4051 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/cd74hc4051.c
)

target_include_directories(driver_cd74hc4051 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_cd74hc4051 INTERFACE
    hardware_adc
    titan_logger
)
