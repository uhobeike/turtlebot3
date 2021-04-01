#ifndef WAYPOINT_NAV_
#define WAYPOINT_NAV_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <turtlebot3_navigation/WaypointNavAction.h>

#include <vector>

using namespace::std;

namespace waypoint_nav {

class WaypointNav
{
public:
    WaypointNav(ros::NodeHandle& nodeHandle, std::string name, std::string file_name);
    virtual ~WaypointNav();

    void GoalCommandCb(const std_msgs::String& msg);
    void AmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void GoalReachedCb(const actionlib_msgs::GoalStatusArray& status);
    void ExecuteCb(const turtlebot3_navigation::WaypointNavGoalConstPtr& goal);

    void ActionClient_Init();
    void PubSub_Init();

    void WaypointCsvRead();
    void WaypointRvizVisualization();
    void WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area, uint8_t index, uint8_t siz);
    void WaypointInfoManagement();
    bool WaypointAreaCheck();
    bool GoalReachCheck();

    void WaypointSet(move_base_msgs::MoveBaseGoal& next);

    void ModeFlagOff();
    void Run();

    void ModeDebug();

private:
    ros::NodeHandle& nh_;

    ros::Subscriber sub_goal_command_, sub_amcl_pose_, sub_movebase_goal_;
    ros::Publisher ini_pose_, way_pose_array_, way_area_array_;
    actionlib::SimpleActionServer<turtlebot3_navigation::WaypointNavAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    string node_name_;

    string csv_fname_;
    int waypoint_csv_index_;
    int waypoint_index_;
    vector<vector<string>> waypoint_csv_;
    vector<double> amcl_pose_;  

    float waypoint_area_threshold_;
    float waypoint_area_check_;

    move_base_msgs::MoveBaseGoal goal_;

    bool NextWaypointMode_;
    bool GoalWaypointMode_;
    bool GoalReachedMode_;
    bool GoalReachedFlag_;
    bool FinalGoalFlag_;
    bool RestartFlag_; 

};

} /* namespace */
#endif