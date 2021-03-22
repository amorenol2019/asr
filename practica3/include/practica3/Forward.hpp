#ifndef PRACTICA3__FORWARD_HPP__
#define PRACTICA3__FORWARD_HPP__

#include "bica/Component.h"

#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include "ros/ros.h"

namespace practica3
{
class Forward : public bica::Component
{
public:
  Forward();

  void detect(const sensor_msgs::Image::ConstPtr& msg);
  void step();

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
