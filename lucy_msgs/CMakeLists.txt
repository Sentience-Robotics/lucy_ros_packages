cmake_minimum_required(VERSION 3.8)
project(lucy_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RawSensor.msg"
  "action/ConfigurePipeline.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_package()
