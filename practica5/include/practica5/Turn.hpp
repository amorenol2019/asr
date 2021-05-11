#ifndef PRACTICA5__TURN_HPP__
#define PRACTICA5__TURN_HPP__

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>

namespace practica5
{
  class Turn : public BT::ActionNodeBase
{
public:
  Turn(const std::string& name); // explicit?
  void halt();
  BT::NodeStatus tick();

private:
  ros::NodeHandle nh_;
  geometry_msgs::Twist vel;
  ros::Publisher vel_pub_;
  ros::Time beggining_time;

  const float VELOCITY = 0.1;
  float TURNING_TIME = 1.0;

};

} // practica5

#endif // PRACTICA5__TURN_HPP__
