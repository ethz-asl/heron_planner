#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoToGoal {
  public:
    GoToGoal(ros::NodeHandle& nh);

  private:
    NodeHandle& nh_;
    MoveBaseClient ac_;

    bool sendGoalAction();
    
}
