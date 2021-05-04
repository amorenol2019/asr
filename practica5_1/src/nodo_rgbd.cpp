
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class RGBDFilter
{
public:
  RGBDFilter(std::string dest, std::string obj): destination_(dest) , object_(obj)
  {
    box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &RGBDFilter::boxCB, this);
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
  }

  void boxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    int ar_length = sizeof(msg->bounding_boxes); // 24

    for(int i = 0 ; i < ar_length ; i++)
    {
      if (msg->bounding_boxes[i].probability > 0.7 && msg->bounding_boxes[i].Class == object_)
      {
        ctr_image_x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax) / 2;
        ctr_image_y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax) / 2;
      }
    }
    std::cout << "(" << ctr_image_x << ", " << ctr_image_y << ")" << std::endl;
  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    auto pcrgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*cloud_in, *pcrgb);

    auto point = pcrgb->at(ctr_image_x, ctr_image_y);

    // Las coordenadas respecto a que son?
    std::cout << "(" << point.x << ", " << point.y << "; " << point.z << ")" << std::endl;
  }

  void create_transform(const float x, const float y, const std::string name)
  {
    geometry_msgs::TransformStamped odom2bf_msg;
    try{
      odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
    } catch (std::exception & e)
    {
      return;
    }

    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);

    tf2::Stamped<tf2::Transform> bf2object;
    bf2object.setOrigin(tf2::Vector3(x * 1.0, y * 1.0, 0));
    bf2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    tf2::Transform odom2object = odom2bf * bf2object;

    geometry_msgs::TransformStamped odom2object_msg;
    odom2object_msg.header.frame_id = object_;
    odom2object_msg.child_frame_id = name;
    odom2object_msg.header.stamp = ros::Time::now();
    odom2object_msg.transform = tf2::toMsg(odom2object);

    br_.sendTransform(odom2object_msg);
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber box_sub_;

  int ctr_image_x = 0.0;
  int ctr_image_y = 0.0;

  std::string destination_;
  std::string object_;

  tf2_ros::Buffer buffer_;
  tf2_ros::StaticTransformBroadcaster br_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd");

  RGBDFilter rf(argv[1],argv[2]);
  ros::spin();
  return 0;
}
