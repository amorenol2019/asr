#include "practica3/Turn.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace practica3
{

Turn::Turn()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); // Tamaño de cola 1 o 10?
}

void
Turn::step()
{
  if(!isActive()){
    return;
  }
  vel.angular.z = 0.3;
  vel_pub_.publish(vel);
}

} // practica3
