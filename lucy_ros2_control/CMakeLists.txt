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
  src/lucy_system.cpp
  src/joint_config.cpp
)

target_compile_features(lucy_ros2_control PUBLIC cxx_std_17)
target_include_directories(lucy_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
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
  DIRECTORY src/include/
  DESTINATION include/lucy_ros2_control
)

# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/lucy_ros2_control
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/lucy_ros2_control
# )
install(TARGETS lucy_ros2_control
  EXPORT export_lucy_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # cpplint excluded: noise on the inherited ros2_control style; copyright lint
  # expects the standard GPLv3 header block (matches lucy_bringup / package.xml).
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
  )
  # uncrustify/cppcheck are often missing or flaky on RoboStack macOS/Windows runners
  if(APPLE OR WIN32)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_uncrustify
      ament_cmake_cppcheck
    )
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # RoboStack macOS: gtest_vendor sources pick up mismatched gtest headers from
  # $CONDA_PREFIX/include (TestInfo API drift). GTests still build on Linux CI.
  if(NOT APPLE)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_position_limit_clamp test/test_position_limit_clamp.cpp)
    target_include_directories(test_position_limit_clamp PRIVATE
      ${PROJECT_SOURCE_DIR}/src/include
    )

    # Pure on_init helpers: compile the module straight into the test (no ROS node
    # needed); depends on hardware_interface for ComponentInfo / lexical_casts.
    ament_add_gtest(test_joint_config
      test/test_joint_config.cpp
      src/joint_config.cpp
    )
    target_include_directories(test_joint_config PRIVATE
      ${PROJECT_SOURCE_DIR}/src/include
    )
    ament_target_dependencies(test_joint_config hardware_interface)
  endif()
endif()

ament_package()
