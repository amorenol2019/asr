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
  Navigate(ros::NodeHandle& nh);

  void doneCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void sendNavigationGoal(void);

  std::string destination_;
  std::string carreta_ = "carreta";
  std::string cajas_ = "cajas";
  std::string contenedor_ = "contenedor";
  std::string derecha_superior_ = "derecha_superior";

  float x_;
  float y_;

  move_base_msgs::MoveBaseGoal goal_;
  bool finished_;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;
};

} //practica4

#endif // PRACTICA4__NAVIGATE_HPP__
