add_library(driver_depth INTERFACE)

target_sources(driver_depth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/depth.c
    ${CMAKE_CURRENT_LIST_DIR}/src/ms5837.c
)

target_include_directories(driver_depth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_depth INTERFACE
    driver_async_i2c
    titan_logger
)
