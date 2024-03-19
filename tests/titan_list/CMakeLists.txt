cmake_minimum_required(VERSION 3.13)

# Define Test Project
project(titan_list_tests)

# Include the library we are testing
add_subdirectory(../../lib/titan_list titan_list)
add_subdirectory(../titan_test titan_test)

# Define Executable
add_executable(titan_list_tests titan_list_tests.c)

# Link libraries
target_link_libraries(titan_list_tests titan_test titan_list)
