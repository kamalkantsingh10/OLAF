cmake_minimum_required(VERSION 3.8)
project(olaf_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Expression.msg"
  "msg/Gesture.msg"
  "msg/HeartRate.msg"
  "msg/ModuleStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
