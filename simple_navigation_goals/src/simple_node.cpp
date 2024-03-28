#include <ros/ros.h>
#include "simple_navigation_goals/go_to_goal.h"

int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "simple_navigation_goals");

  // Create a NodeHandle object
  ros::NodeHandle nh;

  // Start goal action
  GoToGoal goalAction(nh);

  // Print an info message
  ROS_INFO("This is a simple ROS node.");

  // Spin to keep the node running
  ros::spin();

  return 0;
}
