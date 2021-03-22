#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "Turn.hpp"

namespace practica3
{
  Turn::Turn()
  {
    vel_pub_ = pub_detect_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void Turn::step()
  {
    geometry_msgs::Twist vel;

    vel.angular.z = TURNING_TIME;

    vel_pub_.publish(vel);
  }
}
