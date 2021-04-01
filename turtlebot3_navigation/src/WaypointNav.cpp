#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include "turtlebot3_navigation/WaypointNav.hpp"

#include <algorithm>
#include <fstream> 
#include <sstream>
#include <cmath>

using namespace::std;

namespace  waypoint_nav {

WaypointNav::WaypointNav(ros::NodeHandle& nodeHandle, std::string node_name, std::string file_name) :
                        nh_(nodeHandle),
                        as_(nodeHandle, node_name, boost::bind(&WaypointNav::ExecuteCb, this, _1), false),
                        ac_("move_base", true),
                        node_name_(node_name),
                        csv_fname_(file_name), waypoint_csv_index_(1), waypoint_index_(0),
                        waypoint_csv_(1, vector<string>(0)), amcl_pose_(4, 0),
                        waypoint_area_threshold_(0.5), waypoint_area_check_(0.0),
                        NextWaypointMode_(true), GoalWaypointMode_(false), GoalReachedMode_(false), 
                        GoalReachedFlag_(false), FinalGoalFlag_(false)
{
    PubSub_Init();
    ActionClient_Init();

    WaypointCsvRead();
    WaypointRvizVisualization();

    as_.start();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::PubSub_Init()
{
    sub_amcl_pose_ = nh_.subscribe("amcl_pose", 1, &WaypointNav::AmclPoseCb, this);
    sub_movebase_goal_ = nh_.subscribe("move_base/status", 1, &WaypointNav::GoalReachedCb, this);

    way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
    way_area_array_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_area", 1, true);
}

void WaypointNav::ActionClient_Init()
{
    while (!ac_.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        exit(0);
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
            waypoint_csv_[waypoint_csv_index_ -1].push_back(word);
        }
        waypoint_csv_.resize(++waypoint_csv_index_);
    }
    /*Index adjustment__________________________*/
    waypoint_csv_.resize(--waypoint_csv_index_);
    --waypoint_csv_index_;
    /*__________________________________________*/
}

void WaypointNav::WaypointRvizVisualization()
{
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    visualization_msgs::MarkerArray waypoint_area;
    waypoint_area.markers.resize(waypoint_csv_index_+1);
    uint8_t vec_cnt_index (0);
    for (auto it_t = waypoint_csv_.begin(); it_t != waypoint_csv_.end(); ++it_t){
        vec_cnt_index = 0;
        WaypointMarkerArraySet(waypoint_area, distance(waypoint_csv_.begin(), it_t), waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size());
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it){
            switch (vec_cnt_index){
                case 0: 
                    pose.position.x = stod(*it);
                    if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
                        waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x = stod(*it);
                    vec_cnt_index++;
                    continue;
                case 1: 
                    pose.position.y = stod(*it);
                    if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
                        waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y = stod(*it);
                    vec_cnt_index++;
                    continue;

                    pose.position.z = 0.2;
                    if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
                    waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z = 0.1;
                
                case 2: 
                    pose.orientation.z = stod(*it);
                    vec_cnt_index++; 
                    continue;
                case 3: 
                    pose.orientation.w = stod(*it);
                    vec_cnt_index++;
                    break;
            }
        }
        pose_array.poses.push_back(pose);
    }
    pose_array.header.stamp = ros::Time::now(); 
    pose_array.header.frame_id = "map";

    way_pose_array_.publish(pose_array);
    way_area_array_.publish(waypoint_area);
}

void WaypointNav::WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area, uint8_t index, uint8_t size)
{
    waypoint_area.markers[index].header.frame_id = "map";
    waypoint_area.markers[index].header.stamp = ros::Time::now();
    waypoint_area.markers[index].id = index;
    waypoint_area.markers[index].type = visualization_msgs::Marker::CYLINDER;
    waypoint_area.markers[index].action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 cylinder;
    cylinder.x = waypoint_area_threshold_;
    cylinder.y = waypoint_area_threshold_;
    cylinder.z = 0.03;
    waypoint_area.markers[index].scale = cylinder;
    if (size == 4)
        waypoint_area.markers[index].color.a = 0.1f;
    else 
        waypoint_area.markers[index].color.a = 0.000001f;
    waypoint_area.markers[index].color.b = 1.0f;
    waypoint_area.markers[index].color.g = 0.0f;
    waypoint_area.markers[index].color.r = 0.0f;
    waypoint_area.markers[index].pose.orientation.z = 0;
    waypoint_area.markers[index].pose.orientation.w = 1;
}

void WaypointNav::WaypointInfoManagement()
{
    if (waypoint_csv_[waypoint_index_].size() >= 0 && waypoint_csv_[waypoint_index_].size() <= 4){
        ModeFlagOff();
        NextWaypointMode_ = true;
    }
    else if (waypoint_csv_[waypoint_index_][4] == "Goal"){
        GoalWaypointMode_ = true;
        if (waypoint_index_ == waypoint_csv_index_ && GoalReachedFlag_ && WaypointAreaCheck()) 
            FinalGoalFlag_ = true;
        if (waypoint_index_ == waypoint_csv_index_)
            ModeFlagOff();
        if (FinalGoalFlag_){
            ROS_INFO("%s: Final Goal Reached", node_name_.c_str());
            ROS_INFO("%s: Please ' Ctl + c ' ",node_name_.c_str());
        }
    }
    else if (waypoint_csv_[waypoint_index_][4] == "GoalReach"){
        ModeFlagOff();
        GoalReachedMode_ = true;
    }
}

bool WaypointNav::WaypointAreaCheck()
{   
    if (NextWaypointMode_){
        waypoint_area_check_ = 
            sqrt(pow( stod(waypoint_csv_[waypoint_index_][0]) - amcl_pose_[0], 2) 
                + pow( stod(waypoint_csv_[waypoint_index_][1]) - amcl_pose_[1], 2));

        if (waypoint_area_check_ <= waypoint_area_threshold_){
            ROS_INFO("%s: WayPoint Passing", node_name_.c_str());
            ROS_INFO("%s: Next Move Plan", node_name_.c_str());

            waypoint_index_++;
            return true;
        }
    }
    else if (!NextWaypointMode_){
        waypoint_area_check_ = 
            sqrt(pow( stod(waypoint_csv_[waypoint_index_][0]) - amcl_pose_[0], 2) 
                + pow( stod(waypoint_csv_[waypoint_index_][1]) - amcl_pose_[1], 2));

        if (waypoint_area_check_ <= waypoint_area_threshold_){
            ROS_INFO("%s: Invade WayPoint Area ", node_name_.c_str());

            return true;
        }
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
}

void WaypointNav::ModeFlagOff()
{
    NextWaypointMode_ = false;
    GoalWaypointMode_ = false;
    GoalReachedMode_  = false;
    GoalReachedFlag_  = false;
}

void WaypointNav::ModeDebug()
{
    cout << "___________________\n"
         << "NextWaypointMode:"   << NextWaypointMode_ << "\n"
         << "GoalWaypointMode: "  << GoalWaypointMode_ << "\n"
         << "GoalReachedMode : "  << GoalReachedMode_  << "\n"
         << "GoalReachedFlag_:"   << GoalReachedFlag_  << "\n"
         << "~~~~~~~~~~~~~~~~~~~\n"
         << "WaypointIndex   :"   << waypoint_index_   << "\n"
         << "___________________\n";
}

void WaypointNav::Run()
{
    goal_.target_pose.header.frame_id = "map"; 
    WaypointSet(goal_);

    ros::Rate loop_rate(5);
    while (ros::ok()){
        ModeDebug();
        if (NextWaypointMode_){
            if (WaypointAreaCheck())
                WaypointSet(goal_);
        }
        else if (GoalWaypointMode_)
                WaypointSet(goal_);
        else if (GoalReachedMode_){
            if (WaypointAreaCheck() && GoalReachCheck())
                WaypointSet(goal_);
        }
        WaypointInfoManagement();
        ros::spinOnce();
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

        if (goalStatus.status == 3 && GoalReachedFlag_ == false)
            GoalReachedFlag_ = true;
    }
}

void WaypointNav::ExecuteCb(const turtlebot3_navigation::WaypointNavGoalConstPtr& goal)
{
    if (goal->start)
        Run();
    else if (goal->stop){
        ROS_INFO("%s: Shutdown now ('o')/ bye bye~~~", node_name_.c_str());
        ros::shutdown();
    }
}

} /* namespace */