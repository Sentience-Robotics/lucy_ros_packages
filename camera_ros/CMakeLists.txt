cmake_minimum_required(VERSION 3.8)
project(camera_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Python classes for the custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetInt.srv"
)

# Install Python scripts
install(PROGRAMS
  scripts/publish_jpg.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
