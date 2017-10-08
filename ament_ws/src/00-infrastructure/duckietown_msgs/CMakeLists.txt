cmake_minimum_required(VERSION 3.5)
project(duckietown_msgs)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(tf_conversions REQUIRED)
# find_package(tf REQUIRED)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CarControl.msg
  msg/CoordinationSignal.msg
  msg/DuckieSensor.msg
  msg/LanePose.msg
#  LEDControl.msg
  msg/LEDDetection.msg
  msg/LEDDetectionArray.msg
  msg/LEDDetectionDebugInfo.msg
  msg/WheelsCmd.msg
  msg/WheelsCmdStamped.msg
  msg/Pose2DStamped.msg
  msg/SignalsDetection.msg
  msg/Twist2DStamped.msg
  msg/AprilTagDetection.msg
  msg/AprilTagDetectionArray.msg
  msg/AprilTagsWithInfos.msg
  msg/TagInfo.msg
  msg/Pixel.msg
  msg/Vector2D.msg
  msg/Segment.msg
  msg/SegmentList.msg
  msg/Rect.msg
  msg/Rects.msg
  msg/SceneSegments.msg
  msg/SourceTargetNodes.msg
  msg/CoordinationClearance.msg
#  IntersectionDetection.msg
#  CoordinationSignal.msg
#  TrafficLightDetection.msg
  msg/VehicleCorners.msg
  msg/VehiclePose.msg
#  VehicleDetection.msg
  msg/FSMState.msg
  msg/BoolStamped.msg
  msg/StopLineReading.msg
  msg/LEDInterpreter.msg
  msg/AntiInstagramHealth.msg
  msg/AntiInstagramTransform.msg
  msg/KinematicsWeights.msg
  msg/KinematicsParameters.msg
  msg/ThetaDotSample.msg
  msg/Vsample.msg
  msg/Trajectory.msg
  msg/ObstacleType.msg
  msg/ObstacleImageDetection.msg
  msg/ObstacleImageDetectionList.msg
  msg/ObstacleProjectedDetection.msg
  msg/ObstacleProjectedDetectionList.msg
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