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
  CarControl.msg
  CoordinationSignal.msg
  DuckieSensor.msg
  LanePose.msg
#  LEDControl.msg
  LEDDetection.msg
  LEDDetectionArray.msg
  LEDDetectionDebugInfo.msg
  WheelsCmd.msg
  WheelsCmdStamped.msg
  Pose2DStamped.msg
  SignalsDetection.msg
  Twist2DStamped.msg
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
  AprilTagsWithInfos.msg
  TagInfo.msg
  Pixel.msg
  Vector2D.msg
  Segment.msg
  SegmentList.msg
  Rect.msg
  Rects.msg
  SceneSegments.msg
  SourceTargetNodes.msg
  CoordinationClearance.msg
#  IntersectionDetection.msg
#  CoordinationSignal.msg
#  TrafficLightDetection.msg
  VehicleCorners.msg
  VehiclePose.msg
#  VehicleDetection.msg
  FSMState.msg
  BoolStamped.msg
  StopLineReading.msg
  LEDInterpreter.msg
  AntiInstagramHealth.msg
  AntiInstagramTransform.msg
  KinematicsWeights.msg
  KinematicsParameters.msg
  ThetaDotSample.msg
  Vsample.msg
  Trajectory.msg
  ObstacleType.msg
  ObstacleImageDetection.msg
  ObstacleImageDetectionList.msg
  ObstacleProjectedDetection.msg
  ObstacleProjectedDetectionList.msg
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