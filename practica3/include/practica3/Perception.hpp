#ifndef PRACTICA3__PERCEPTION_HPP__
#define PRACTICA3__PERCEPTION_HPP__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class Perception : public bica::Component
{
public:
  Perception();

  void step();

private:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);
  void objectCb(const std_msgs::Float32::ConstPtr& msg);
  void create_transform(float x, float y ,std::string object);
  int orient_2object(const int x,const int y);

  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::string object_;
  int distance_;
  int width_;
  double angle_;
  const int TURNING_V = 0.1;

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

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::TransformListener listener_;
};

#endif // PRACTICA3__FORWARD_HPP__
