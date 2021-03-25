#ifndef PRACTICA3__FORWARD_HPP__
#define PRACTICA3__FORWARD_HPP__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <std_msgs/Float32.h>

namespace practica3
{
class Forward : public bica::Component
{
public:
  Forward();
  void step();

private:
  void distanceCb(const std_msgs::Float32::ConstPtr& msg);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber dist_sub_;
  geometry_msgs::Twist vel;

  float distance_;
  int velocity_;

};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
