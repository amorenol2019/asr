#include <string>
#include <vector>
#include <iostream>

#include "practica5/RGBDFilter.hpp"

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

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace practica5
{
  RGBDFilter::RGBDFilter(std::string dest, std::string obj): BT::ActionNodeBase(?????) : destination_(dest), object_(obj), ctr_image_x(0.0), ctr_image_y(0.0)
  {
    box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &RGBDFilter::boxCB, this);
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
  }

  void RGBDFilter::boxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    int ar_length = sizeof(msg->bounding_boxes);
    detected_ = false;

    for(int i = 0 ; i < ar_length ; i++)
    {
      if(msg->bounding_boxes[i].probability >= MIN_PROB && msg->bounding_boxes[i].Class == object_)
      {
        ctr_image_x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax) / 2;
        ctr_image_y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax) / 2;
        detected_ = true;
      }
    }
    ROS_INFO("(%c, %c)", ctr_image_x, ctr_image_y);
  }

  void RGBDFilter::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    sensor_msgs::PointCloud2 cloud;

    if (!detected_)
    {
      return;
    }

    try
    {
      pcl_ros::transformPointCloud("/map", *cloud_in, cloud, listener_);
      auto pcrgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      pcl::fromROSMsg(cloud, *pcrgb);

      auto point = pcrgb->at(ctr_image_x, ctr_image_y);

      ROS_INFO("(%f, %f, %f)", point.x, point.y, point.z);
      create_transform(point.x, point.y, point.z);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }
  }

  void RGBDFilter::create_transform(const float x, const float y, const float z)
  {
    geometry_msgs::TransformStamped odom2object_msg;
    odom2object_msg.transform.translation.x = x;
    odom2object_msg.transform.translation.y = y;
    odom2object_msg.transform.translation.z = z;
    odom2object_msg.transform.rotation.x = 0.0;
    odom2object_msg.transform.rotation.y = 0.0;
    odom2object_msg.transform.rotation.z = 0.0;
    odom2object_msg.transform.rotation.w = 1.0;
    odom2object_msg.header.frame_id = "map";
    odom2object_msg.child_frame_id = object_;
    odom2object_msg.header.stamp = ros::Time::now();

    br_.sendTransform(odom2object_msg);
  }

  void RGBDFilter::halt() {

  }

  BT::NodeStatus RGBDFilter::tick(){

  }


} //namespace practica5


//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "rgbd");
//  if(argc < 3)
//  {
//     std::cerr << "usage: rosrun practica5_1 nodo_rgbd <destination> <object>" << std::endl; // cerr
//     return -1;
//  }

//  RGBDFilter rf(argv[1],argv[2]);
//  ros::spin();
//  return 0;
//}
