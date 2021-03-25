#include "practica3/Practica3Impl.hpp"

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "std_msgs/Int64.h"

namespace practica3
{
Practica3Impl::Practica3Impl()
{
  object_pub_ = nh_.advertise<std_msgs::Int64>("/object", 10);
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
  msg_.data = 3; // "yellow";
  object_pub_.publish(msg_);
}

void Practica3Impl::ToBlueGoal_code_iterative()
{
  ROS_INFO("\nBLue\n");
  msg_.data = 2; // "blue";
  object_pub_.publish(msg_);
}

void Practica3Impl::ToBall_code_iterative()
{
  ROS_INFO("\nball\n");
  msg_.data = 1; //"ball";
  object_pub_.publish(msg_);
}

void Practica3Impl::Turn_code_iterative()
{
  ROS_INFO("\nturn\n");
  msg_.data = 4; //"ball";
  object_pub_.publish(msg_);
}

} // practica3
