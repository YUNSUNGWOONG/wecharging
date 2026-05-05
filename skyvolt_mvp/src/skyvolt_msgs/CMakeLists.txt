cmake_minimum_required(VERSION 3.8)
project(skyvolt_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RfidDetection.msg
  msg/PhotoeyeDetection.msg
  msg/TrackState.msg
  msg/DockingState.msg
  msg/ChargingJob.msg
  msg/FleetStatus.msg
  srv/AssignJob.srv
  action/TransportPile.action
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
