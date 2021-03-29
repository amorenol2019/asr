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
  void positionCb(const std_msgs::Float32::ConstPtr& msg);
  void angleCb(const std_msgs::Float32::ConstPtr& msg);
  int orient_2object();

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber dist_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber angle_sub_;

  geometry_msgs::Twist cmd_;

  float distance_;
  float velocity_;

  int x_;
  int y_;
  int width_;

  float v_turning_;
  float angle_2obj_;
};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
