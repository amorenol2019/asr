#include "bica/Component.h"
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practica4/Navigate.hpp"

namespace practica4
{
  void step()
  {
  if(!isActive() || distance_pub_.getNumSubscribers() == 0 || angle_pub_.getNumSubscribers() == 0){
    return;
  }
////?????????????? como hacemos el callback sin una clase ni variables glbales ????
  ros::Subscriber dest_sub_ = nh_.subscribe("/destination", 10, &Perception::destinationCb, this);
  practica4::Navigate navigator(true,////destino del callback??);

  while(!navigator.ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

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

}//practica4
