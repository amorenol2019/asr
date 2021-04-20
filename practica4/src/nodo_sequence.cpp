#include <ros/ros.h>
#include "practica4/Navigate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  // practica4::Navigate navigator(false);

  practica4::Navigate navigator(false);

  ros::Rate loop_rate(10);

  int count = 0;

  while(navigator.ok())
  {
    navigator.step();

    ros::spinOnce();
    loop_rate.sleep();
  }


  /*
  std::vector<std::string> arr_destinations = {"carreta", "cajas", "contenedor", "esquina"};

  // Send a goal to the robot to move to the selected position
  for (int i = 0; i < arr_destinations.size(); i++)
  {
    navigator.destination_ = arr_destinations[i];
    navigator.set_coordinates();
  }
    */

  return 0;
}
