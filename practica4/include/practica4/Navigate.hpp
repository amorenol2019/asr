#ifndef PRACTICA4__NAVIGATE_HPP__
#define PRACTICA4__NAVIGATE_HPP__

#include "bica/Component.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
class Navigate : public bica::Component
{
public:
  Navigate(ros::NodeHandle& nh);
  void step();

  void doneCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void sendNavigationGoal(void);

  std::string destination_;

  float x_ = - 0.5; // = 0.0;
  float y_ = 8.5; // = 0.0;

  move_base_msgs::MoveBaseGoal goal_;
  bool finished_;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;
};

}//practica4

#endif // PRACTICA4__NAVIGATE_HPP__
