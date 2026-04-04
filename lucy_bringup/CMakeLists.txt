cmake_minimum_required(VERSION 3.8)
project(lucy_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install system scripts
install(PROGRAMS
  system_scripts/launch_lucy.sh
  system_scripts/stop_lucy.sh
  system_scripts/check_lucy.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_launch_syntax test/test_launch_syntax.py
    TIMEOUT 120
  )
endif()

ament_package()

