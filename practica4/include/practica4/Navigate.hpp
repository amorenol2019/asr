#ifndef PRACTICA4__NAVIGATE_HPP__
#define PRACTICA4__NAVIGATE_HPP__

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
class Navigate
{
public:
  Navigate(bool need_arg,std::string arg="none");
  void sendNavigationGoal(void);
  void set_coordinates();

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  std::string destination_ = "none";

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;

  float x_;
  float y_;
  move_base_msgs::MoveBaseGoal goal_;

  ros::NodeHandle nh_;
};

} //practica4

#endif // PRACTICA4__NAVIGATE_HPP__
