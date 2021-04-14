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
  void doneCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  move_base_msgs::MoveBaseGoal goal_;
  bool finished_ = false;
};

} //practica4

#endif // PRACTICA4__NAVIGATE_HPP__
