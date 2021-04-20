#include "practica4/SequenceImpl.hpp"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "practica4");
  ros::NodeHandle n;

  practica4::SequenceImpl practica4;

  ros::Rate loop_rate(10);

  int count = 0;

  while (practica4.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
