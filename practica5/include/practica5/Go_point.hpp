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
  Go_point(const std::string& name); //, const std::string& destination); // explicit ??
  void sendNavigationGoal(void);
  void set_coordinates(move_base_msgs::MoveBaseGoal& goal);

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void halt();
  BT::NodeStatus tick();

private:
  std::string destination_ = "carreta"; // cambiar a "none"

  float x_;
  float y_;
  bool first_time_;
  double goal_x_;
  double goal_y_;
  move_base_msgs::MoveBaseGoal goal_;

  ros::NodeHandle nh_;

};

} // practica5

#endif // PRACTICA5__GO_POINT_HPP__
