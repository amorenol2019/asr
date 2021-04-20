#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"
#include <bica/Component.h>

namespace practica4
{
Navigate::Navigate(bool need_arg) : nh_("~"), ac_("move_base", true)
{
  if(!need_arg){
    dest_sub_ = nh_.subscribe<std_msgs::String>("/destination", 10, &Navigate::destinationCb, this);
  }

  finish_pub_ = nh_.advertise<std_msgs::Bool>("/finish", 10);
}

void Navigate::destinationCb(const std_msgs::String::ConstPtr& msg)
{
  destination_ = msg->data;
  set_coordinates();
}

void Navigate::set_coordinates()
{
  if(destination_ == "carreta"){
    ROS_INFO("HHH");
    //nh_.getParam("X_CARRETA",x_);
    //nh_.getParam("Y_CARRETA",y_);
    x_ = -0.5;
    y_ = 9.0;
  }
  else if(destination_ == "cajas"){
    //nh_.getParam("X_CAJAS",x_);
    //nh_.getParam("Y_CAJAS",y_);
    x_ = -3.5;
    y_ = -2.5;
  }
  else if(destination_ == "contenedor"){
    //nh_.getParam("X_CONTENEDOR",x_);
    //nh_.getParam("Y_CONTENEDOR",y_);
    x_ = 2.5;
    y_ = -7.0;
  }
  else if(destination_ == "esquina"){
    //nh_.getParam("X_ESQUINA",x_);
    //nh_.getParam("Y_ESQUINA",y_);
    x_ = 4.0;
    y_ = -8.5;
  }

  ROS_INFO("Destination: %s x:%f y:%f", destination_.c_str(), x_, y_);
}

void Navigate::doneCb(const actionlib::SimpleClientGoalState& state,
  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  std_msgs::Bool msg;
  msg.data = true;
  finish_pub_.publish(msg);
  if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot arrived to %s\n", destination_.c_str());
  else
    ROS_INFO("The base failed to move for some reason");
}

void Navigate::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  double goal_x = goal_.target_pose.pose.position.x;
  double goal_y = goal_.target_pose.pose.position.y;
  double current_x = feedback->base_position.pose.position.x;
  double current_y = feedback->base_position.pose.position.y;

  double diff_x = goal_x - current_x;
  double diff_y = goal_y - current_y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  ROS_INFO("Distance to %s = %lf", destination_.c_str(), distance);
}

void Navigate::sendNavigationGoal()
{
  while(!ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  goal_.target_pose.header.frame_id = "map";

  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position.x = x_;
  goal_.target_pose.pose.position.y = y_;
  goal_.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac_.sendGoal(goal_,
    boost::bind(&Navigate::doneCb, this, _1, _2),
    MoveBaseClient::SimpleActiveCallback(),
    boost::bind(&Navigate::feedbackCb, this, _1));
    ac_.waitForResult(); // esto entonces habria que quitarlo de aqui !? car: creo que no
}

void Navigate::step()
{
  if(!isActive() || finish_pub_.getNumSubscribers() == 0){
    return;
  }
  sendNavigationGoal();
}

} //practica4
