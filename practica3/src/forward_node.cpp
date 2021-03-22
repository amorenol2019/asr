#include "practica3/Forward.hpp"

#include "ros/ros.h"

int main (int argc, cahr **argv)
{
  ros::init(argc, argv, "forward");
  ros::NodeHandle n;

  practica3::Forward forward;

  ros::Rate loop_rate(20);

  int count = 0;

  while(Forward.ok())
  {
    forward.step();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
