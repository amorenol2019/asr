#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practica4/Navigate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  practica4::Navigate navigator(true,argv[1]);

ROS_INFO("argc:%d  argv[0]:%s\n",argc,argv[1]);
  if(navigator.destination_ == "none"){
    ROS_INFO("There is no parameter.");
    return 0;
  }

  while(!navigator.ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Send a goal to the robot to move to the selected position
  navigator.goal_.target_pose.header.frame_id = "map";
  navigator.goal_.target_pose.header.stamp = ros::Time::now();

  navigator.goal_.target_pose.pose.position.x = navigator.x_;
  navigator.goal_.target_pose.pose.position.y = navigator.y_;
  navigator.goal_.target_pose.pose.orientation.w = 1.0;

  navigator.sendNavigationGoal();

  if(navigator.ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot arrived to %s\n", navigator.destination_.c_str());
  else
    ROS_INFO("The base failed to move for some reason");

  navigator.destination_ = "none";

  return 0;
}
