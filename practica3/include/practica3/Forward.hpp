#ifndef PRACTICA3__FORWARD_HPP__
#define PRACTICA3__FORWARD_HPP__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace practica3
{
class Forward : public bica::Component
{
public:
  Forward();

  void step();

private:
  ros::Publisher vel_pub_;
  ros::NodeHandle nh_;
};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
