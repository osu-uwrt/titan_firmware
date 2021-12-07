# Add boards folder to PICO_BOARD_HEADER_DIRS 
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/boards)
list(APPEND PICO_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include) # so boards/foo.h can be explicitly included