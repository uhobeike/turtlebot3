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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot3_navigation/ObstacleDetecteAction.h>

using namespace::std;

class ObstacleDetecte
{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_scan_;
    actionlib::SimpleActionServer<turtlebot3_navigation::ObstacleDetecteAction> as_;
    std::string action_name_;
    
    sensor_msgs::LaserScan latest_scan_, save_scam_;
    int16_t LeftSearchAngle_;
    int16_t RightSearchAngle_;
    vector<int16_t> SearchArray_;

    turtlebot3_navigation::ObstacleDetecteFeedback feedback_;
    turtlebot3_navigation::ObstacleDetecteResult result_;

    _Float32 ObstacleDetecteThreshold_;
    uint8_t NumberOfDetections_;

public: 
    ObstacleDetecte(std::string name) :
                              as_(nh_, name, boost::bind(&ObstacleDetecte::executeCB, this, _1),false),
                              action_name_(name),
                              LeftSearchAngle_(15), RightSearchAngle_(15),
                              ObstacleDetecteThreshold_(0.4), NumberOfDetections_(15)
    {
        sub_scan_ = nh_.subscribe("scan", 1, &ObstacleDetecte::ScanDistanceCB, this);
        as_.start();
    }
    ~ObstacleDetecte() {}

    _Float32 DegreeToRadian(int16_t& degree)
    {
        double radian  = degree * M_PI / 180;
        return radian;
    }

    vector<int16_t> LeftSearchArray(int16_t& degree)
    {
        vector<int16_t> LeftSearchIndexArray;
        double LeftSearchRadian = DegreeToRadian(degree);
        
        for (double i(0); i<LeftSearchRadian; i+=latest_scan_.angle_increment)
            LeftSearchIndexArray.push_back(1);
        return LeftSearchIndexArray;
    }

    vector<int16_t> RightSearchArray(int16_t& degree)
    {
        vector<int16_t> RightSearchIndexArray;
        double RightSearchRadian = DegreeToRadian(degree);
        for (double i(0); i<RightSearchRadian; i+=latest_scan_.angle_increment)
            RightSearchIndexArray.push_back(1);
        return RightSearchIndexArray;
    }

    vector<int16_t> MergeSearchArray(int16_t& left_degree, int16_t& right_degree)
    {
        vector<int16_t> SearchArray;
        SearchArray.push_back(360 - LeftSearchArray(left_degree).size());
        SearchArray.push_back(RightSearchArray(right_degree).size());
        return SearchArray;
    }

    bool ScanObstacle()
    {
        cout << "__________________________" << "\n";

        NumberOfDetections_ = 0;
        for (int i(SearchArray_.at(0)); i != SearchArray_.at(1); ++i){
            if (latest_scan_.ranges[i] < latest_scan_.range_min ||
                latest_scan_.ranges[i] > latest_scan_.range_max ||
                std::isnan(latest_scan_.ranges[i])) {}
            else {
                _Float32 scan_diff = abs(latest_scan_.ranges[i] - save_scam_.ranges[i]);
                cout << scan_diff << ", ";
                if (scan_diff > ObstacleDetecteThreshold_)
                    NumberOfDetections_++;
                if (NumberOfDetections_ == 15){
                    cout << "\n";
                    cout << "__________________________" << "\n";
                    return true;
                }
            }

            if (i == 359)
                i = 0;
        }
        cout << "\n";
        cout << "__________________________" << "\n";

        return false;
    }

    void ScanDistanceCB(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        latest_scan_ = *msg;
    }

    void executeCB(const turtlebot3_navigation::ObstacleDetecteGoalConstPtr &goal)
    {
        if (goal->order){
            bool success = false;
            result_.observation_results = false;
            ROS_INFO("%s: Start ObstacleDetecte processing", action_name_.c_str());

            while (latest_scan_.ranges.empty()){
                ROS_INFO("%s: Wait scan_msg callback", action_name_.c_str());
                ros::Duration duration(0.1);
                duration.sleep();
            }

            save_scam_ = latest_scan_;
            SearchArray_ = MergeSearchArray(LeftSearchAngle_, RightSearchAngle_);

            ros::Rate loop_rate(10);
            while (ros::ok()){
                ros::spinOnce();

                if (latest_scan_.ranges.size() > 0){                    
                    if (ScanObstacle()){
                        success = true;
                        break;
                    }
                }
                loop_rate.sleep();
            }

            if (success){
                result_.observation_results = true;
                ros::Duration duration(1.5);
                duration.sleep();
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }  
        else if (!goal->order)
            ros::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obs_detecte");

    ObstacleDetecte ObstacleDetecte("obs_detecte");

    ros::spin();
    return 0;
}
