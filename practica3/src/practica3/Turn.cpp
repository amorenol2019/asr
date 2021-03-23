#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "practica3/Turn.hpp"

namespace practica3
{
  Turn::Turn()
  {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void Turn::step()
  {
    geometry_msgs::Twist vel;

    vel.angular.z = 0.3;

    vel_pub_.publish(vel);
  }
}
