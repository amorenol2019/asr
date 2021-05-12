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
  Go_point(const std::string& name, const BT::NodeConfiguration& config);

  void sendNavigationGoal(void);
  void set_coordinates();

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  //void doneCb(const actionlib::SimpleClientGoalState& state,
    //  const move_base_msgs::MoveBaseResultConstPtr& result);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("goal")};
  }

private:
  ros::NodeHandle nh_;
  std::string destination_ = "none";

  move_base_msgs::MoveBaseGoal goal_;
};

} // practica5

#endif // PRACTICA5__GO_POINT_HPP__
