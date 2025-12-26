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
  scripts/camera_stream_controller.py
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
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Install test files
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  # Install pytest.ini for coverage configuration
  install(FILES
    pytest.ini
    DESTINATION share/${PROJECT_NAME}
  )

  # Add pytest tests with coverage
  ament_add_pytest_test(test_camera_publisher test/test_camera_publisher.py
    TIMEOUT 60
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_camera_stream_controller test/test_camera_stream_controller.py
    TIMEOUT 60
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

# Export the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
