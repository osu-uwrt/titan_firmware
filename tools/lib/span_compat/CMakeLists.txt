cmake_minimum_required(VERSION 3.12)

# Define cmake library
add_library(span_cpp INTERFACE)

IF ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "10.0"))
    # Use C++20 span if gcc supports it
    target_compile_features(span_cpp INTERFACE cxx_std_20)
else()
    # Fallback to internal span compatibility library
    target_include_directories(span_cpp INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
endif()
