cmake_minimum_required(VERSION 3.8)
project(lucy_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  lucy_ros2_control
  SHARED
  hardware/lucy_system.cpp
)

target_compile_features(lucy_ros2_control PUBLIC cxx_std_17)
target_include_directories(lucy_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/lucy_ros2_control>
)
ament_target_dependencies(
  lucy_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lucy_ros2_control.xml)


# Installing
install(
  DIRECTORY hardware/include/
  DESTINATION include/lucy_ros2_control
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/ros2_control_demo_example_2
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/ros2_control_demo_example_2
# )
install(TARGETS lucy_ros2_control
  EXPORT export_lucy_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
