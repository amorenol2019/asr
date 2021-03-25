#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"


namespace practica3
{

Forward::Forward()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); // Tamaño de cola 1 o 10?
    //dist_sub_ = nh_.subscribe<std_msg::int>("/distance",1, &Forward::distanceCb, this);
}
//void Forward::distanceCb(const std_msgs::int::ConstPtr& msg)
//{
//  distance_ = msg;
//}

void
Forward::step()
{
  if(!isActive()){ // Componente de BICA
    return;
  }

  //implementar una funcion que haga que la velocidad disminuya en funcion a la distancia a la que este

  vel.linear.x = 0.3;
  vel_pub_.publish(vel);
}

} // practica3
