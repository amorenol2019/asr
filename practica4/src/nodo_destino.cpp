#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practica4/Navigate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");

  if(argc < 2){
    ROS_INFO("There is no parameter.");
    return 0;
  }

  practica4::Navigate navigator(true);

  navigator.destination_ = argv[1];
  navigator.set_coordinates();

  ROS_INFO("argc:%d  argv[0]:%s\n", argc, argv[1]);

  navigator.sendNavigationGoal();

  navigator.destination_ = "none";

  return 0;
}
