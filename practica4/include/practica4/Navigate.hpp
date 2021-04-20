#ifndef PRACTICA4__NAVIGATE_HPP__
#define PRACTICA4__NAVIGATE_HPP__

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <bica/Component.h>
#include "std_msgs/Bool.h"


namespace practica4
{
class Navigate : public bica::Component
{
public:
  Navigate(bool need_arg);
  void sendNavigationGoal();
  void set_coordinates();
  void step();

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;

  float x_;
  float y_;
  std::string destination_ = "none";
  move_base_msgs::MoveBaseGoal goal_;

private:
  void destinationCb(const std_msgs::String::ConstPtr& msg);
  void doneCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  ros::NodeHandle nh_;
  ros::Publisher finish_pub_;
  ros::Subscriber dest_sub_;

};

} //practica4

#endif // PRACTICA4__NAVIGATE_HPP__
