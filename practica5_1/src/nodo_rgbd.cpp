
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class RGBDFilter
{
public:
  RGBDFilter()
  {
    box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &RGBDFilter::boxCB, this);
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
  }
  void boxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
     int ar_length = sizeof(msg->bounding_boxes); //24s

    for ( int i = 0 ; i < ar_length ; i++)
    {
      if (msg->bounding_boxes[i].probability > 0.7 && msg->bounding_boxes[i].probability < 1)
      {
            ctr_image_x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax) / 2;
            ctr_image_y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax) / 2;

            std::cout << "probability: " << msg->bounding_boxes[i].probability << std::endl;
            std::cout << "xmin: " << msg->bounding_boxes[i].xmin << "xmax: " <<msg->bounding_boxes[i].xmax<< std::endl;
            std::cout << "ctr x: " << ctr_image_x << std::endl;

      }
    }
    std::cout << "(" << ctr_image_x << ", " << ctr_image_y <<")"<< std::endl;
  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    auto pcrgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*cloud_in, *pcrgb);

    //int c_w = cloud_in->width / 2;
    //int c_h = cloud_in->height / 2;

    auto point = pcrgb->at(ctr_image_x, ctr_image_y);
    //?
    std::cout << "(" << point.x << ", " << point.y << "; " << point.z << ")" << std::endl;
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber box_sub_;

  int ctr_image_x=0.0;
  int ctr_image_y=0.0;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd");
  RGBDFilter rf;
  ros::spin();
  return 0;
}
