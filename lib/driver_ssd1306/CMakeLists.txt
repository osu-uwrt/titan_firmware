add_library(driver_ssd1306 INTERFACE)

target_sources(driver_ssd1306 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306.c
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306_fonts.c
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306_tests.c
)

target_include_directories(driver_ssd1306 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_ssd1306 INTERFACE
    driver_async_i2c
)
