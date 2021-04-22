/*
*Copyright 2021, uhobeike.
*
*Licensed under the Apache License, Version 2.0 (the "License");
*you may not use this file except in compliance with the License.
*You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
*Unless required by applicable law or agreed to in writing, software
*distributed under the License is distributed on an "AS IS" BASIS,
*WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*See the License for the specific language governing permissions and
*limitations under the License.
*/

#ifndef WAYPOINT_NAV_
#define WAYPOINT_NAV_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <thread>

using namespace::std;

namespace waypoint_nav {

class WaypointNav
{
public:
    WaypointNav(ros::NodeHandle& nodeHandle, std::string name, std::string file_name);
    virtual ~WaypointNav();

    void AmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void GoalReachedCb(const actionlib_msgs::GoalStatusArray& status);
    void GoalCommandCb(const std_msgs::String& msg);
    void AwsCb(const std_msgs::String& msg);

    void ActionClient_Init();
    void PubSub_Init();

    void WaypointCsvRead();
    void WaypointRvizVisualization();
    void WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area, 
                                visualization_msgs::MarkerArray& waypoint_number_txt,
                                uint8_t index, uint8_t siz);
    void StrategyCheck(vector<string>& waypoint_csv_);
    void DividedByStrategyArray(geometry_msgs::PoseArray& pose_array, visualization_msgs::MarkerArray& waypoint_area, 
                                visualization_msgs::MarkerArray& waypoint_number_txt);

    void WaypointInfoManagement();
    bool WaypointAreaCheck();
    bool GoalReachCheck();
    bool ObjectDetectCheck();

    void WaypointSet(move_base_msgs::MoveBaseGoal& current);
    void WaypointNextSet(move_base_msgs::MoveBaseGoal& next);
    void WaypointPrevSet(move_base_msgs::MoveBaseGoal& prev);
    void WaypointInitSet(move_base_msgs::MoveBaseGoal& init);
    void WaypointCourseSelectSet(move_base_msgs::MoveBaseGoal& course, uint& waypoint_index);

    void ModeFlagOff();
    void Run();

    void ModeFlagDebug();
    void ModeFlagDebugAWS();

private:
    ros::NodeHandle& nh_;

    ros::Subscriber sub_amcl_pose_, sub_movebase_goal_, sub_goal_command_, sub_aws_;
    ros::Publisher ini_pose_, way_pose_array_, way_area_array_, way_number_txt_array_, 
                   aws_debug_, ros_to_awsiot_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

    string node_name_;

    string csv_fname_;
    uint waypoint_csv_index_;
    double strategy_rviz_;
    vector <geometry_msgs::PoseArray> pose_array_vtr_;
    geometry_msgs::PoseArray pose_array_;
    geometry_msgs::PoseArray pose_array_divide_;
    vector<visualization_msgs::MarkerArray> waypoint_area_vtr_;
    visualization_msgs::MarkerArray waypoint_area_;
    visualization_msgs::MarkerArray waypoint_area_divide_;
    visualization_msgs::MarkerArray waypoint_area_delete_;
    vector<visualization_msgs::MarkerArray> waypoint_number_txt_vtr_;
    visualization_msgs::MarkerArray waypoint_number_txt_;
    visualization_msgs::MarkerArray waypoint_number_txt_divide_;
    visualization_msgs::MarkerArray waypoint_number_txt_delete_;

    uint waypoint_index_;
    vector<vector<string>> waypoint_csv_;
    vector<double> amcl_pose_;

    float waypoint_area_threshold_;
    float waypoint_area_check_;

    move_base_msgs::MoveBaseGoal goal_;
    thread th_;

    //Mode
    bool NextWaypointMode_;
    bool FinalGoalWaypointMode_;
    bool ReStartWaypointMode_;
    bool GoalReachedMode_;
    bool ForcedNextWaypointMode_;
    bool ForcedPrevWaypointMode_;
    bool ReturnToInitialPositionMode_;
    bool FreeSelectWaypointMode_;
    bool FreeSelectWaypointAWSMode_;

    //Flag
    bool FinalGoalFlag_;
    bool ReStartFlag_; 
    bool MsgReceiveFlag_;
    bool GoalReachedFlag_;
    bool ActionRestartFlag_;
    bool ActionCancelFlag_;
    bool FreeSelectWaypointFlag_;

    //Value
    uint waypoint_index_awsiot_;
};

} /* namespace */
#endif