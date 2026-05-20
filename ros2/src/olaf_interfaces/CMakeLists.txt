cmake_minimum_required(VERSION 3.8)
project(olaf_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Declare ROS2 messages
# NOTE: Expression/EarsPose/Gesture/HeartRate archived 2026-05-15 — see
# archive/ros2/olaf_interfaces/msg/README.md and the Phase 2 SCP.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModuleStatus.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
