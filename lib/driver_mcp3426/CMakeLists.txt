add_library(driver_mcp3426 INTERFACE)

target_sources(driver_mcp3426 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mcp3426.c
)

target_include_directories(driver_mcp3426 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_mcp3426 INTERFACE
    driver_async_i2c
)
