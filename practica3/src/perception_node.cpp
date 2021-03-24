#include "practica3/Perception.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception");
  ros::NodeHandle n;

  Perception perception;

  ros::Rate loop_rate(10);

  int count = 0;

  while (perception.ok())
  {
    perception.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
