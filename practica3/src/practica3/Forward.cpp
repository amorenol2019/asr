#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Float32.h>

namespace practica3
{
Forward::Forward()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    dist_sub_ = nh_.subscribe<std_msgs::Float32>("/distance", 1, &Forward::distanceCb, this);
}

void Forward::distanceCb(const std_msgs::Float32::ConstPtr& msg)
{
  distance_ = msg->data;
}

void Forward::step()
{
  //if(!isActive() || vel_pub_.getNumSubscribers() == 0){ // Ahorrar procesamiento innecesario
    //return;
  //}
  ROS_INFO("distance_: %f", distance_);

  // Hacer pruebas para ajustarlo
  if(distance_ > 1.0)
  {
    velocity_ = distance_ * 0.1;
  }
  else
  {
    velocity_ = 0.01;
  }

  cmd_.linear.x = velocity_;
  vel_pub_.publish(cmd_);
}

} // practica3
