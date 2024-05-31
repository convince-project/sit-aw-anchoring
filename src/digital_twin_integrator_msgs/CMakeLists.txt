cmake_minimum_required(VERSION 3.5.0)
project(digital_twin_integrator_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(process_msgs REQUIRED)

# Start of user code dependencies
# End of user code

	find_package(rosidl_default_generators REQUIRED)

	# Generate messages, services or actions
	rosidl_generate_interfaces(
		digital_twin_integrator_msgs
		"msg/EntityData.msg"
		"action/GetData.action"
		"action/TestGraspability.action"
		"action/DigitalTwinIntegratorUpdateState.action"
		DEPENDENCIES geometry_msgs process_msgs
	)

		ament_package()
