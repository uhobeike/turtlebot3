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
  ObstacleDetecte.action
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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################
##waypoint_rviz 
add_executable(waypoint_rviz 
  src/waypoint_rviz_set.cpp
)
target_link_libraries(waypoint_rviz 
  ${catkin_LIBRARIES}
)

##WaypointNav_AWS
add_library(WaypointNav_AWS
  src/WaypointNav_AWS.cpp
)
add_dependencies(WaypointNav_AWS 
  ${TARGET_NAME} 
  ${PROJECT_NAME}_generate_messages_cpp
)
add_executable(WaypointNav_AWS_node
  src/WaypointNav_AWS_node.cpp
)
target_link_libraries(
  WaypointNav_AWS_node
  WaypointNav_AWS
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
