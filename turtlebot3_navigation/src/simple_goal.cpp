#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  double x;
  double y;
  double yaw;
};

int main(int argc, char** argv){
  MyPose way_point[] = {2.0, 1.0, 0.5 * M_PI};

  ros::init(argc, argv, "simple_goal");

  // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。
  // アクションサーバーが立ち上がっていないとだめ。
  // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。
  MoveBaseClient ac("move_base", true);
  // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
  // この例では５秒間待つ(ブロックされる)
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("The server comes up");
  move_base_msgs::MoveBaseGoal goal;
  // base_link座標系（ロボット座標系)
  goal.target_pose.header.frame_id = "map";
  // 現在時刻                                                                       
  goal.target_pose.header.stamp = ros::Time::now();


  int i = 0;
  while (ros::ok()) {
    // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
    goal.target_pose.pose.position.x =  way_point[i].x;
    goal.target_pose.pose.position.y =  way_point[i].y;
    goal.target_pose.pose.orientation.w = 1; 

    ROS_INFO("Sending goal: No.%d", i+1);
    // サーバーにgoalを送信
    ac.sendGoal(goal);

    // 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
    bool succeeded = ac.waitForResult(ros::Duration(30.0));

    // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
    actionlib::SimpleClientGoalState state = ac.getState();

    if(succeeded) {
      ROS_INFO("Succeeded: No.%d (%s)",i+1, state.toString().c_str());
    }
    else {
      ROS_INFO("Failed: No.%d (%s)",i+1, state.toString().c_str());
    }
    i++;
  }
  return 0;
}