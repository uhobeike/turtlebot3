#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include "turtlebot3_navigation/WaypointNav.hpp"

#include <fstream> 
#include <sstream>
#include <cmath>

using namespace::std;

namespace  WaypointNav {

WaypointNav::WaypointNav(ros::NodeHandle& nodeHandle, std::string node_name, std::string file_name) :
                        nh_(nodeHandle),
                        as_(nodeHandle, node_name, boost::bind(&WaypointNav::ExecuteCb, this, _1), false),
                        ac_("move_base", true),
                        node_name_(node_name_),
                        csv_fname_(file_name), waypoint_csv_index_(0), waypoint_rviz_index_(0), waypoint_index_(0),
                        waypoint_csv_(0), amcl_pose_(4, 0),
                        waypoint_area_threshold_(0.5), waypoint_area_check_(0.0),
                        NextWaypointMode_(true), GoalWaypointMode_(false), GoalReachedMode_(false), GoalReachedFlag_(false)
{
    PubSub_Init();
    MoveBaseClient_Init();

    WaypointCsvRead();
    WaypointRvizVisualization();
} 

WaypointNav::~WaypointNav() {}

void WaypointNav::PubSub_Init()
{
    sub_amcl_pose_ = nh_.subscribe("amcl_pose", 1, &WaypointNav::AmclPoseCb, this);
    sub_movebase_goal_ = nh_.subscribe("move_base/status", 1, &WaypointNav::GoalReachedCb, this);

    way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
}

void WaypointNav::MoveBaseClient_Init()
{
    while (!ac_.waitForServer(ros::Duration(30.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("MoveBase server comes up");
}

void WaypointNav::WaypointCsvRead()
{
    ifstream f_r(csv_fname_.c_str(), std::ios::in);
    if (f_r.fail()){
        ROS_ERROR("std::ifstream could not open %s.", csv_fname_.c_str());
        exit(-1);
    }
    string line, word;
    while (getline(f_r, line)){
        istringstream stream(line);
        while (getline(stream, word, ',')){
            waypoint_csv_[waypoint_csv_index_].push_back(word);
        }

        waypoint_csv_.resize(++waypoint_csv_index_);
    }
}

void WaypointNav::WaypointRvizVisualization()
{
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    uint8_t vec_cnt_index (0);
    for (auto it_t = waypoint_csv_.begin(); it_t != waypoint_csv_.end(); ++it_t){
        vec_cnt_index = 0;
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it){
            switch (vec_cnt_index){
                case 1: 
                    pose.position.x = stod(*it);
                    continue;
                case 2: 
                    pose.position.y = stod(*it);
                    continue;
                
                    pose.position.z = 0.2;
                
                case 3: 
                    pose.orientation.z = stod(*it);
                    continue;
                case 4: 
                    pose.orientation.w = stod(*it);
                    break;
            }
            vec_cnt_index++;
        }
        pose_array.poses.push_back(pose);
    }

    way_pose_array_.publish(pose_array);
}

void WaypointNav::WaypointInfoManagement()
{
    if (waypoint_csv_[waypoint_index_].size() >= 0 && waypoint_csv_[waypoint_index_].size() <= 3)
        NextWaypointMode_ = true;
    else if (waypoint_csv_[waypoint_index_][4] == "Goal")
        GoalWaypointMode_ = true;
    else if (waypoint_csv_[waypoint_index_][4] == "GoalReach")
        GoalReachedMode_ = true;

    if (waypoint_index_ == waypoint_csv_index_ && GoalReachedFlag_) 
        FinalGoalFlag_ = true;

    if (FinalGoalFlag_){
        ROS_INFO("%s: Final Goal Reached", node_name_.c_str());
        ROS_INFO("%s: Please 'aaaaa' ", node_name_.c_str());
    }
}

bool WaypointNav::WaypointAreaCheck()
{
    waypoint_area_check_ = 
        sqrt(pow( stod(waypoint_csv_[waypoint_index_][0]) - amcl_pose_[0], 2) 
            + pow( stod(waypoint_csv_[waypoint_index_][1]) - amcl_pose_[1], 2));
    
    if (waypoint_area_check_ <= waypoint_area_threshold_){
        ROS_INFO("%s: WAY_POINT PASSING", node_name_.c_str());
        ROS_INFO("%s: NEXT MOVE PLAN", node_name_.c_str());

        waypoint_index_++;
        return true;
    }
    return false;
}

bool WaypointNav::GoalReachCheck()
{
    if (GoalReachedFlag_){
        ROS_INFO("%s: Goal Reached", node_name_.c_str());
        ROS_INFO("%s: Restart", node_name_.c_str());
        
        waypoint_index_++;
        return true;
    }
    return false;
}

void WaypointNav::WaypointSet(move_base_msgs::MoveBaseGoal& goal)
{
    goal.target_pose.pose.position.x    = stod(waypoint_csv_[waypoint_index_][0]);
    goal.target_pose.pose.position.y    = stod(waypoint_csv_[waypoint_index_][1]);
    goal.target_pose.pose.orientation.z = stod(waypoint_csv_[waypoint_index_][2]);
    goal.target_pose.pose.orientation.w = stod(waypoint_csv_[waypoint_index_][3]);
    goal.target_pose.header.stamp       = ros::Time::now();

    ac_.sendGoal(goal);
    ModeFlagOff();
}

void WaypointNav::ModeFlagOff()
{
    NextWaypointMode_ = 0;
    GoalWaypointMode_ = 0;
    GoalReachedMode_  = 0;
    GoalReachedFlag_  = 0;
}

void WaypointNav::Run()
{
    goal_.target_pose.header.frame_id = "map";    
    WaypointSet(goal_);

    ros::Rate loop_rate(5);
    while (ros::ok()){
        WaypointInfoManagement();
        ros::spinOnce();

        if (NextWaypointMode_ || GoalWaypointMode_){
            if (WaypointAreaCheck())
                WaypointSet(goal_);
        }
        if (GoalReachedMode_){
            if (GoalReachCheck())
                WaypointSet(goal_);
        }
        loop_rate.sleep();
    }
}

void WaypointNav::AmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    amcl_pose_.at(0) = msg.pose.pose.position.x; 
    amcl_pose_.at(1) = msg.pose.pose.position.y;
    amcl_pose_.at(2) = msg.pose.pose.orientation.z;
    amcl_pose_.at(3) = msg.pose.pose.orientation.w;
}

void WaypointNav::GoalReachedCb(const actionlib_msgs::GoalStatusArray& status)
{
    if (!status.status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status.status_list[0];

        if (goalStatus.status == 3 || GoalReachedFlag_ == 0) 
            GoalReachedFlag_ = 1;
    }
}

void WaypointNav::ExecuteCb(const turtlebot3_navigation::WaypointNavGoalConstPtr& goal){
    if (goal->strat)
        Run();
    else if (goal->stop){
        ROS_INFO("%s: Shutdown now ('o')/ bye bye~~~", node_name_.c_str());
        ros::shutdown();
    }
}

} /* namespace */