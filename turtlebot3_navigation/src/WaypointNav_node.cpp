#include <ros/ros.h>

#include "turtlebot3_navigation/WaypointNav.hpp"

using namespace::std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "WaypointNav");
	ros::NodeHandle nh;

	waypoint_nav::WaypointNav wv(nh, ros::this_node::getName(), argv[1]);
	ROS_INFO("%s: Please ' rostopic pub -1 /WaypointNav/goal turtlebot3_navigation/WaypointNavActionGoal ' ",
			 ros::this_node::getName().c_str());
	
	ros::spin();
	return 0;
}