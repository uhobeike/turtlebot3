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
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS 
   roscpp 
   std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################
#waypoint_rviz
add_executable(waypoint_rviz src/waypoint_rviz_set.cpp)
target_link_libraries(waypoint_rviz ${catkin_LIBRARIES})

##WaypointNav
add_library(WaypointNav
  src/WaypointNav.cpp
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
