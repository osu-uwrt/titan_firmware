cmake_minimum_required(VERSION 3.12)

add_library(titan_binary_info INTERFACE)

target_include_directories(titan_binary_info INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
