#include "practica5/Turn.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace practica5
{
  Turn::Turn(): BT::ActionNodeBase(????)
  {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    beggining_time = ros::Time::now();
  }

  void Turn::halt()
  {
    ROS_INFO("He girado un pokin");
  }

  BT::NodeStatus Turn::tick()
  {
    vel.angular.z = VELOCITY;
    vel_pub_.publish(vel);
    if( (ros::Time::now() - beggining_time) < TURNING_TIME)
    {
      return BT::NodeStatus::RUNNING;
    }
    if(ros::Time::now() - beggining_time >= TURNING_TIME)
    {
      vel.angular.z= 0;
      return BT::NodeStatus::SUCCESS;
    }
  }

} // practica5
