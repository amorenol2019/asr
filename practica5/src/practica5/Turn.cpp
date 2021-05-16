#include "practica5/Turn.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace practica5
{
  Turn::Turn(const std::string& name): BT::ActionNodeBase(name, {}) , nh_("~"),detected_(false)
  {
    detect_sub_ = nh_.subscribe("/detected", 10, &Turn::detectCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  }

  void Turn::detectCb(const std_msgs::Bool::ConstPtr& msg)
  {
    detected_ = msg->data;
  }

  void Turn::halt()
  {
    ROS_INFO("Object detected; ");
  }

  BT::NodeStatus Turn::tick()
  {
    vel.angular.z = VELOCITY;

    if(!detected_)
    {
      vel_pub_.publish(vel);
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      vel.angular.z = 0;
      vel_pub_.publish(vel);
      return BT::NodeStatus::SUCCESS;
    }

  }

} // practica5
