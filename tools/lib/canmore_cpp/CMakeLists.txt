cmake_minimum_required(VERSION 3.12)

# Include required libraries
add_subdirectory(${REPO_DIR}/lib/titan_canmore titan_canmore)

# Find threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Define sources
add_library(canmore_cpp INTERFACE)

target_sources(canmore_cpp INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/src/Discovery.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/DebugClient.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/CANDiscovery.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/EthernetDiscovery.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/RegMappedClient.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/RegMappedCANClient.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/RegMappedEthernetClient.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/BootloaderClient.cpp
)

target_include_directories(canmore_cpp INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(canmore_cpp INTERFACE
	titan_canmore
	Threads::Threads

	# Tool Libraries
	rp2040_flash_itf
)
