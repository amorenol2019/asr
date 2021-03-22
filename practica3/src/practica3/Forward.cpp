#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "Forward.hpp"

namespace practica3
{
  Forward::Forward()
  {
    vel_pub_ = pub_detect_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void Forward::step()
  {
    geometry_msgs::Twist vel;

    vel.linear.x = ;//lo que sea

    vel_pub_.publish(vel);
  }
}
