cmake_minimum_required(VERSION 2.8.3)
project(duckietown_msgs)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_generation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(tf REQUIRED)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS})

ament_export_dependencies(
   geometry_msgs
   rclcpp rclpy
   sensor_msgs
   std_msgs
   message_runtime
   visualization_msgs
   )

ament_package()