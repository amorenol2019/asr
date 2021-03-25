#include "practica3/Practica3Impl.hpp"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "practica3");
  ros::NodeHandle n;

  practica3::Practica3Impl practica3;

  ros::Rate loop_rate(10);

  int count = 0;

  while (practica3.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
