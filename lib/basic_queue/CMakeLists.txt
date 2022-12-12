cmake_minimum_required(VERSION 3.12)

add_library(basic_queue INTERFACE)
target_include_directories(basic_queue INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
