#include "practica3/Practica3Impl.hpp"

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

namespace practica3
{
Practica3Impl::Practica3Impl()
{
  object_pub_ = nh_.advertise<std_msgs::String>("/object", 1);
  state_ts_ = ros::Time::now();
}

bool Practica3Impl::ToBall_2_ToBlueGoal()
{
  ROS_INFO("\ntransicion\n");
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::ToBlueGoal_2_ToYellGoal()
{
  ROS_INFO("\ntransicion\n");
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::ToYellGoal_2_Turn()
{
  ROS_INFO("\ntransicion\n");
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::Turn_2_ToBall()
{
  ROS_INFO("\ntransicion\n");
  return (ros::Time::now() - state_ts_).toSec() > TURNING_TIME;
}


void Practica3Impl::ToYellGoal_code_iterative()
{
  ROS_INFO("\nYellow\n");
  object_pub_.publish("yellow");
}

void Practica3Impl::ToBlueGoal_code_iterative()
{
  ROS_INFO("\nBLue\n");
  object_pub_.publish("blue");
}

void Practica3Impl::ToBall_code_iterative()
{
  ROS_INFO("\nball\n");
  object_pub_.publish("ball");
}

void Practica3Impl::Turn_code_iterative()
{
  ROS_INFO("\nturn\n");
  object_pub_.publish("Turn");
}
}
