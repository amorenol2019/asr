#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practica4/Navigate.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  practica4::Navigate navigator(n);

  //wait for the action server to come up
  while(!navigator.ac_.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
  }

  //we'll send a goal to the robot to move 1 meter forward
  navigator.goal_.target_pose.header.frame_id = "base_link";
  navigator.goal_.target_pose.header.stamp = ros::Time::now();

  navigator.goal_.target_pose.pose.position.x = navigator.x_;
  navigator.goal_.target_pose.pose.position.y = navigator.y_;
  navigator.goal_.target_pose.pose.orientation.w = 1.0;

  navigator.sendNavigationGoal();

  if(navigator.ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
   }
