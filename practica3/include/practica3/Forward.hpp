#ifndef PRACTICA3__FORWARD_HPP__
#define PRACTICA3__FORWARD_HPP__

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace practica3
{
class Forward : public bica::Component
{
public:
  Forward();

  void step();

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
