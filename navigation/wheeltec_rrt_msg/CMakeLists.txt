cmake_minimum_required(VERSION 3.5)
project(wheeltec_rrt_msg)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ChangePosition.action"
  "action/FindColouredBox.action"
  "action/PickColouredBox.action"
  "srv/DeleteShape.srv"
  "msg/PointArray.msg"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

ament_package()
