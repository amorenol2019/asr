#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Float32.h>

namespace practica3
{
Forward::Forward()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  dist_sub_ = nh_.subscribe<std_msgs::Float32>("/distance", 10, &Forward::distanceCb, this);
  position_sub_ = nh_.subscribe<std_msgs::Int64MultiArray>("position",10,&Forward::positionCb, this);
  angle_sub_ = nh_.subscribe<std_msgs::Float32>("/angle",10,&Forward::angleCb, this);

}

void Forward::distanceCb(const std_msgs::Float32::ConstPtr& msg)
{
  distance_ = msg->data;
}

void Forward::positionCb(const std_msgs::Float32::ConstPtr& msg)
{
  x_ = msg->data[0];
  y_ = msg->data[1];
  width_ = msg->data[2];
}

void Forward::angleCb(const std_msgs::Float32::ConstPtr& msg)
{
  angle_2obj_ = msg->data; 
}

int Perception::orient_2object()
{ // devuelve 1 si el objeto esta centrado en la imagen

  int centered = 0;
  if(x_ < width_ / 2 + 50 && x_ > width_ / 2 - 50)
  {
    v_turning_ = 0.05;

    if(x_ > width_ / 2 + 20)
    {
      cmd_.angular.z = -v_turning_;
    } else if (x_ < width_ / 2 - 20)
    {
      cmd_.angular.z = v_turning_;
    } else
    {
      cmd_.angular.z = 0;
      centered = 1;
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
  vel_pub_.publish(cmd_);

  return centered;
}

void Forward::step()
{
  if(!isActive() || vel_pub_.getNumSubscribers() == 0){
    return;
  }

  if(distance_ > 0.05)
  {
    velocity_ = distance_ * 0.5;
    cmd_.angular.z = 0;
    cmd_.linear.x = velocity_;
    vel_pub_.publish(cmd_);
  }
}

} // practica3
