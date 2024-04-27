cmake_minimum_required(VERSION 3.5)
project(wheeltec_robot_rrt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wheeltec_rrt_msg REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)

add_library(find_coloured_box SHARED src/bt_plugins/find_coloured_box.cpp)
list(APPEND plugin_libs find_coloured_box)

add_library(approach_coloured_box SHARED src/bt_plugins/approach_coloured_box.cpp)
list(APPEND plugin_libs approach_coloured_box)

add_library(pick_coloured_box SHARED src/bt_plugins/pick_coloured_box.cpp)
list(APPEND plugin_libs pick_coloured_box)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} behaviortree_cpp_v3 nav2_msgs nav2_behavior_tree wheeltec_rrt_msg)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_executable(robot_picker src/robot_picker.cpp)
ament_target_dependencies(robot_picker rclcpp nav2_util nav2_behavior_tree)

add_executable(find_robot_action_server src/find_robot_action_server.cpp)
ament_target_dependencies(find_robot_action_server rclcpp rclcpp_action nav2_msgs geometry_msgs wheeltec_rrt_msg)

add_executable(pick_robot_action_server src/pick_robot_action_server.cpp)
ament_target_dependencies(pick_robot_action_server rclcpp rclcpp_action wheeltec_rrt_msg)

add_executable(wait_for_fin src/rrt_exploration/wait_for_fin.cpp)
ament_target_dependencies(wait_for_fin rclcpp geometry_msgs nav_msgs visualization_msgs)


install(TARGETS  
    robot_picker 
    find_robot_action_server 
    pick_robot_action_server
    wait_for_fin
    DESTINATION lib/${PROJECT_NAME}
)

include_directories(src/rrt_exploration/include)

add_library(utils SHARED src/rrt_exploration/utils.cpp)
ament_target_dependencies(utils rclcpp nav2_msgs visualization_msgs)

add_library(mtrand SHARED src/rrt_exploration/mtrand.cpp)

add_library(mean_shift SHARED src/rrt_exploration/mean_shift.cpp)

add_library(robot SHARED src/rrt_exploration/robot.cpp)
ament_target_dependencies(robot rclcpp tf2_ros geometry_msgs nav2_msgs)

add_executable(local_rrt src/rrt_exploration/local_rrt.cpp)
target_link_libraries(local_rrt utils mtrand)


ament_target_dependencies(local_rrt 
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    )


add_executable(global_rrt src/rrt_exploration/global_rrt.cpp)
target_link_libraries(global_rrt utils mtrand)

ament_target_dependencies(global_rrt 
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    )

add_executable(filter src/rrt_exploration/filter.cpp)
target_link_libraries(filter utils mean_shift)

ament_target_dependencies(filter 
    rclcpp
    geometry_msgs
    visualization_msgs
    wheeltec_rrt_msg
    )

add_executable(assigner src/rrt_exploration/assigner.cpp)
target_link_libraries(assigner utils robot)

ament_target_dependencies(assigner 
    rclcpp
    wheeltec_rrt_msg
    )

add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)
ament_target_dependencies(robot_pose_publisher 
    rclcpp
    sensor_msgs
    tf2_ros
    geometry_msgs
    )

install(TARGETS 
  local_rrt
  global_rrt
  filter
  assigner
  robot_pose_publisher
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  behaviour_trees
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
    utils mtrand mean_shift robot
   DESTINATION lib
)

install(PROGRAMS
  scripts/boundary_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
