cmake_minimum_required(VERSION 3.8)
project(lucy_config_services)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/hardware_config_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
