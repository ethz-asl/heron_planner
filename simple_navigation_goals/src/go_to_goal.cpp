#include "simple_navigation_goals/go_to_goal.h"

GoToGoal::GoToGoal(ros::NodeHandle& nh) : nh_ { nh } {

  //tell the action client that we want to spin a thread by default
  // ac("move_base", true);

  //wait for the action server to come up
  while(!ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  GoToGoal::sendGoalAction()
}
bool GoToGoal::sendGoalAction(){

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac_.sendGoal(goal);

  ac_.waitForResult();

  if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
