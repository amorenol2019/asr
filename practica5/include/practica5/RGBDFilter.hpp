#ifndef PRACTICA5__RGBDFILTER_HPP__
#define PRACTICA5__RGBDFILTER_HPP__

#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>

namespace practica5
{
class RGBDFilter : public BT::ActionNodeBase
{
public:
  RGBDFilter(const std::string& name); //, std::string obj); // explicit?
  void halt();
  BT::NodeStatus tick();

private:
  void boxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
  void create_transform(const float x, const float y, const float z);

  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber box_sub_;

  int ctr_image_x;
  int ctr_image_y;
  int MIN_PROB = 0.7;

  bool detected_ = false;
  bool created_ = false;

  std::string object_;

  tf2_ros::StaticTransformBroadcaster br_;
  tf::TransformListener listener_;

};

} // practica5

#endif // PRACTICA5__GO_POINT_HPP__
