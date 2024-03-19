cmake_minimum_required(VERSION 3.13)

# Define Test Project
project(titan_queue_tests)

# Include the library we are testing
add_subdirectory(../../lib/titan_queue titan_lib)
add_subdirectory(../titan_test titan_test)

# Define Executable
add_executable(titan_queue_tests titan_queue_tests.c)

# Link libraries
target_link_libraries(titan_queue_tests titan_test titan_queue)
