#ifndef PRACTICA5__GO_POINT_HPP__
#define PRACTICA5__GO_POINT_HPP__

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>

namespace practica5
{
class Go_point : public BT::ActionNodeBase
{
public:
  Go_point(bool need_param_); // explicit ??
  void sendNavigationGoal(void);
  void set_coordinates();

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void halt();
  BT::NodeStatus tick();

  std::string destination_ = "carreta"; // cambiar a "none"

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;

  float x_;
  float y_;
  bool first_time_;
  move_base_msgs::MoveBaseGoal goal_;

  ros::NodeHandle nh_;

};

} // practica5

#endif // PRACTICA5__GO_POINT_HPP__
