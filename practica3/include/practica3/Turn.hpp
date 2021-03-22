#ifndef PRACTICA3__TURN_HPP__
#define PRACTICA3__TURN_HPP__

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace practica3
{
class Turn : public bica::Component
{
public:
  Turn();

  void detect();
  void step();

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  const double TURNING_TIME = 0.5;//por ejemplo
};

} // practica3

#endif // PRACTICA3__TURN_HPP__
