cmake_minimum_required(VERSION 3.12)

# Copy in DeviceList
set(DEVICE_DEFINITIONS_FILE DeviceList.jsonc )
set(DEVICE_DEFINITIONS_DIR ${REPO_DIR}/tools )
set(DEVICE_DEFINITIONS_GEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/DeviceList.o" )

# Generate object file containing embedded device definitions
add_custom_command(
    OUTPUT ${DEVICE_DEFINITIONS_GEN_OUT}
    COMMAND ${CMAKE_LINKER} --relocatable --format binary --output ${DEVICE_DEFINITIONS_GEN_OUT} ${DEVICE_DEFINITIONS_FILE}
    WORKING_DIRECTORY ${DEVICE_DEFINITIONS_DIR}
    DEPENDS ${DEVICE_DEFINITIONS_DIR}/${DEVICE_DEFINITIONS_FILE}
)
add_custom_target(device_definitions ALL DEPENDS ${DEVICE_DEFINITIONS_GEN_OUT} )

# Define cmake library
add_library(rp2040_flash_itf INTERFACE)

target_sources(rp2040_flash_itf INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/DeviceMap.cpp
)

target_include_directories(rp2040_flash_itf INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

# Ensure that the devicemap object is generated before this target so the linker can find it
add_dependencies(rp2040_flash_itf device_definitions)
target_link_libraries(rp2040_flash_itf INTERFACE ${DEVICE_DEFINITIONS_GEN_OUT})
