#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practica4/Navigate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  practica4::Navigate navigator(false);

  while(!navigator.ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<std::string> arr_destinations = {"frigo", "grifo", "cocina", "despensa"};
  navigator.goal_.target_pose.header.frame_id = "map";

  // Send a goal to the robot to move to the selected position
  for (int i = 0; i < arr_destinations.size(); i++)
  {
    navigator.destination_ = arr_destinations[i];
    navigator.set_coordinates();

    navigator.goal_.target_pose.header.stamp = ros::Time::now();
    navigator.goal_.target_pose.pose.position.x = navigator.x_;
    navigator.goal_.target_pose.pose.position.y = navigator.y_;
    navigator.goal_.target_pose.pose.orientation.w = 1.0;

    navigator.sendNavigationGoal();

    if(navigator.ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot arrived to %s\n", navigator.destination_.c_str());
    else
      ROS_INFO("The base failed to move for some reason");
  }

  return 0;
}
