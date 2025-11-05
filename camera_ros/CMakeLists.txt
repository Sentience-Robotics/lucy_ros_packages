cmake_minimum_required(VERSION 3.8)
project(camera_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Python classes for the custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetInt.srv"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

# Install Python scripts
install(PROGRAMS
  scripts/camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(FILES
  launch/camera.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
