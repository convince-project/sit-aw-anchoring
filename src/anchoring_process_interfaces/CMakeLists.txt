cmake_minimum_required(VERSION 3.5.0)
project(anchoring_process_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(process_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages, services or actions
rosidl_generate_interfaces(
  anchoring_process_interfaces
  "action/SetOntology.action"
  "action/PopulateInstances.action"
  "action/UpdateState.action"
  DEPENDENCIES process_msgs
)

ament_package()
