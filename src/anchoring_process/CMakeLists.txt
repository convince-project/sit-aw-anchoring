cmake_minimum_required(VERSION 3.5.0)
project(anchoring_process)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib)
find_library(typedb-driver-cpp-LIBRARY NAMES typedb-driver-cpp)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(anchoring_process_interfaces REQUIRED)
find_package(anchoring_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/src-gen
	${PROJECT_SOURCE_DIR}/includes
)

add_executable(anchoring_process
	src-gen/anchoring_process_def/anchoring_process.cpp
	src-gen/anchoring_process_def/anchoring_process_main.cpp
	src/anchoring_process_def/anchoring_process_impl.cpp
	src/anchoring_process_def/typedb_client.cpp
	src/anchoring_process_def/utils/parameters_utils.cpp
)
ament_target_dependencies(anchoring_process
	nlohmann_json
	rclcpp
	rclcpp_lifecycle
	rclcpp_action
	anchoring_process_interfaces
 	anchoring_core
 	pluginlib
)


# Start of user code dependencies
target_link_libraries(anchoring_process typedb-driver-cpp)
install(FILES
    lib/libtypedb-driver-cpp.so
    DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY
#	data tql
#	DESTINATION share/${PROJECT_NAME}/
#)
# End of user code

install(TARGETS
	anchoring_process
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
