add_library(driver_sht41 INTERFACE)

target_sources(driver_sht41 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sht41.c
)

target_link_libraries(driver_sht41 INTERFACE driver_async_i2c)

target_include_directories(driver_sht41 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
