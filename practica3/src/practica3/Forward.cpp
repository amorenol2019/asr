#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <std_msgs/Float32MultiArray.h>

namespace practica3
{
Forward::Forward()
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
{ // devuelve 1 si el objeto esta centrado en la imagen

  //int centered = 0;
  if(x_ < width_ / 2 + 50 && x_ > width_ / 2 - 50)
  {
    v_turning_ = 0.1;

    if(x_ > width_ / 2 + 20)
    {
      cmd_.angular.z = -v_turning_;
    } else if (x_ < width_ / 2 - 20)
    {
      cmd_.angular.z = v_turning_;
    } else
    {
      cmd_.angular.z = 0;
      //centered = 1;
    }
  } else
  {
    v_turning_ = 0.3;

    if(x_ > width_ / 2 + 50)
    {
      cmd_.angular.z = -v_turning_;
    } else
    {
      cmd_.angular.z = v_turning_;
    }
  }
  //vel_pub_.publish(cmd_);
}

void Forward::step()
{
  if(!isActive() || vel_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  if(angle_2obj_ > 0) // revisar
  {
    cmd_.angular.z = v_turning_;
  }
  else
  {
    cmd_.angular.z = -v_turning_;
  }

  if(distance_ > 0.2)
  {
    ROS_INFO("distance > 1");

    velocity_ = distance_ * 0.3;
    cmd_.linear.x = velocity_;
    orient_2object();
  }
  else
  {
    ROS_INFO("distance < 1");
    cmd_.linear.x = 0;
  }

  vel_pub_.publish(cmd_);
}

} // practica3
