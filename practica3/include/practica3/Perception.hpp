#ifndef PRACTICA3__PERCEPTION_HPP__
#define PRACTICA3__PERCEPTION_HPP__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

namespace practica3
{
class Perception : public bica::Component
{
public:
  Perception();

  void step();

private:
  // Analiza lo que le llega de la camara, lo filtra y calcula distancia aproximada:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);
  std_msgs::Bool orient_2object(const int x,const int y);

  ros::NodeHandle nh_;
  ros::Publisher object_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // Rangos H:
  int h_min;
  int h_max;

  const int BALL_HMIN = 93;
  const int BALL_HMAX = 148;
  const int BLUE_HMIN = 16;
  const int BLUE_HMAX = 45;
  const int YELLOW_HMIN = 86;
  const int YELLOW_HMAX = 91;

  // Rangos S:
  const int S_MIN = 0;
  const int S_MAX = 360;
  // Rangos V:
  const int V_MIN = 0;
  const int V_MAX = 360;

  std::string object_;
  int distance_;

  const int TURNING_V = 0.1;

};

} // practica3

#endif // PRACTICA3__FORWARD_HPP__
