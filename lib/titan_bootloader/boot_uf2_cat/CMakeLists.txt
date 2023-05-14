cmake_minimum_required(VERSION 3.12)
project(boot_uf2_cat)

add_subdirectory(../../pico-sdk/src/common/boot_uf2 boot_uf2_headers)

add_executable(boot_uf2_cat main.c)
target_link_libraries(boot_uf2_cat boot_uf2_headers)