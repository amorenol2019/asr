#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

namespace practica3
{
Forward::Forward() : v_turning_(0.4)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  dist_sub_ = nh_.subscribe<std_msgs::Float64>("/distance", 10, &Forward::distanceCb, this);
  position_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/position", 10, &Forward::positionCb, this);
  angle_sub_ = nh_.subscribe<std_msgs::Float64>("/angle", 10, &Forward::angleCb, this);
}

void Forward::distanceCb(const std_msgs::Float64::ConstPtr& msg)
{
  distance_ = msg->data;
}

void Forward::positionCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  x_ = msg->data[0];
  y_ = msg->data[1];
  width_ = msg->data[2];
}

void Forward::angleCb(const std_msgs::Float64::ConstPtr& msg)
{
  angle_2obj_ = msg->data;
}

void Forward::orient_2object()
{
  if(x_ < width_ / 2 + 50 && x_ > width_ / 2 - 50)
  {
    if(x_ > width_ / 2 + 20)
    {
      cmd_.angular.z = - v_turning_ + 0.3;
    }
    else if (x_ < width_ / 2 - 20)
    {
      cmd_.angular.z = v_turning_ - 0.3;
    }
    else
    {
      cmd_.angular.z = 0;
    }
  }
  else
  {
    if(x_ > width_ / 2 + 50)
    {
      cmd_.angular.z = - v_turning_ + 0.1;
    }
    else
    {
      cmd_.angular.z = v_turning_; // - 0.1;
    }
  }
}

void Forward::step()
{
  if(!isActive() || vel_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  if(distance_ == 0) // No ve nada centrado
  {
    //if(angle_2obj_ != 400)
    //{
    //  cmd_.angular.z = v_turning_ * angle_2obj_;
    //}
    //else
    //{
    orient_2object();
    //}
  }
  else
  {
    if(distance_ < 0.8) // Ha llegado
    {
      cmd_.linear.x = 0.0;
      cmd_.angular.z = 0.0;
    }
    else // Se acerca
    {
      cmd_.linear.x = 0.5;
      if(distance_ < 3)
      {
        cmd_.linear.x = 0.2;
      }
      orient_2object();
    }
  }

  vel_pub_.publish(cmd_);
}

} // practica3
