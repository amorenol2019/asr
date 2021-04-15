#include <ros/ros.h>
#include <practica4/Navigate.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  practica4::Navigate navigator; //??

  ros::init(argc, argv, "navigate");

  MoveBaseClient ac("move_base", false); // cambiar

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base acation server to come up");
  }

  navigator.goal_.target_pose.header.frame_id = "map";
  navigator.goal_.target_pose.header.stamp = ros::Time::now();

  // Posicion a la que ir
  navigator.goal_.target_pose.pose.position.x = 1.0;
  navigator.goal_.target_pose.pose.position.y = -3.0;
  navigator.goal_.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(navigator.goal_, navigator.doneCb, MoveBaseClient::SimpleActiveCallback(),  navigator.feedbackCb);

  /*
  ros::Rate rate(1);
  while(ros::ok() && !navigator.finished_) // cambiar segunda condicion
  {
    ROS_INFO("Esperando");
    rate.sleep();
  }
  */
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move foward 1 meter for some reason");

  return 0;
}
