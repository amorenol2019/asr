#include "practica5/Turn.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace practica5
{
  Turn::Turn(const std::string& name): BT::ActionNodeBase(name, {}) , nh_("~")
  {
    tf_sub_ = nh_.subscribe("/created", 10, &Turn::tfCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    beggining_time = ros::Time::now();
  }

  void Turn::tfCb(const std_msgs::Bool::ConstPtr& msg)
  {
    created_ = msg->data;
  }

  void Turn::halt()
  {
    ROS_INFO("He girado un pokin");
  }

  BT::NodeStatus Turn::tick()
  {
    vel.angular.z = VELOCITY;

    /*
    if((ros::Time::now() - beggining_time).toSec() < TURNING_TIME)
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
    */

    //   ----------------PARA GIRAR CUANDO NO HAYA CREADO LA TF
    if (created_)
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
