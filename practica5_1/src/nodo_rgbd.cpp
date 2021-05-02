
#include <string>
#include <vector>

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

class RGBDFilter
{
public:
  RGBDFilter()
  {
    cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &RGBDFilter::cloudCB, this);
  }
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    auto pcrgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*cloud_in, *pcrgb);

    int c_w = cloud_in->width / 2;
    int c_h = cloud_in->height / 2;

    auto point = pcrgb->at(c_w, c_h);

    std::cout << "(" << point.x << ", " << point.y << "; " << point.z << ")" << std::endl;
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd");
  RGBDFilter rf;
  ros::spin();
  return 0;
}
