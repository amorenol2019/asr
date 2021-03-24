#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"


namespace practica3
{

Forward::Forward()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); // Tamaño de cola 1 o 10?
}

void
Forward::step()
{
  if(!isActive()){ // Componente de BICA
    return;
  }

  geometry_msgs::Twist vel;
  vel.linear.x = 0.3;
  vel_pub_.publish(vel);
}

} // practica3
