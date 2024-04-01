set(NANOPB_SRC_ROOT_FOLDER ${REPO_DIR}/lib/nanopb)
set(CMAKE_MODULE_PATH ${REPO_DIR}/lib/nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

# Generate NanoPB Files
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS test.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Create a library using those files
add_library(titan_msgs INTERFACE ${PROTO_HDRS} ${PROTO_SRCS})
target_include_directories(titan_msgs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${NANOPB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(titan_msgs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/titan_msgs.c
    ${PROTO_SRCS}
)
