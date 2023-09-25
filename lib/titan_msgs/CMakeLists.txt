# Generate NanoPB Files
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS RELPATH include include/titan/test.proto)

# Create a library using those files
add_library(titan_msgs INTERFACE ${PROTO_HDRS} ${PROTO_SRCS})

target_include_directories(titan_msgs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${NANOPB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(titan_msgs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/titan_msgs.c
)

target_link_libraries(titan_msgs INTERFACE
    driver_canbus
    titan_list
)
