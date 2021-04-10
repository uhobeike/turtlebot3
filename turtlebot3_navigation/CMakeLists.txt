################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_navigation)
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  tf
  actionlib_msgs
  costmap_dynamic_object_detecter
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(
  DIRECTORY action
  FILES
  WaypointNav.action
)

generate_messages(
  DEPENDENCIES
   actionlib_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS 
   roscpp 
   std_msgs
   actionlib_msgs
   costmap_dynamic_object_detecter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################
add_executable(waypoint_navigation src/waypoint_navigation.cpp)
target_link_libraries(waypoint_navigation ${catkin_LIBRARIES})

add_executable(waypoint_rviz src/waypoint_rviz_set.cpp)
target_link_libraries(waypoint_rviz ${catkin_LIBRARIES})

add_executable(start_order_waypoint_navigation src/start_order_waypoint_navigation.cpp)
target_link_libraries(start_order_waypoint_navigation ${catkin_LIBRARIES})

add_library(WaypointNav
  src/WaypointNav.cpp
)
add_dependencies(WaypointNav 
  ${TARGET_NAME} 
  ${PROJECT_NAME}_generate_messages_cpp
)
add_executable(WaypointNav_node
  src/WaypointNav_node.cpp
)
target_link_libraries(
  WaypointNav_node
  WaypointNav
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
