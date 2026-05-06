cmake_minimum_required(VERSION 3.8)
project(anchoring_cubesworld_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(anchoring_core REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(anchoring_core plugins.xml)

add_library(anchoring_cubesworld_plugin src/anchoring_cubesworld_plugin.cpp)
target_compile_features(anchoring_cubesworld_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(anchoring_cubesworld_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  anchoring_cubesworld_plugin
  "ament_index_cpp"
  "rclcpp"
  "rclcpp_lifecycle"
  "nlohmann_json"
  "anchoring_core"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(anchoring_cubesworld_plugin PRIVATE "ANCHORING_CUBESWORLD_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS anchoring_cubesworld_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY rules DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  anchoring_cubesworld_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
