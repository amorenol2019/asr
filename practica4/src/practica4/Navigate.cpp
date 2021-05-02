#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
Navigate::Navigate(bool need_param_) : nh_("~"), ac_("move_base", true)
{
  if(need_param_){
    nh_.getParam("destination", destination_);
    set_coordinates();
  }
}

void Navigate::set_coordinates(){
  if(destination_ == "frigo"){
    x_ = -0.42;
    y_ = -3.04;
  }
  else if(destination_ == "grifo"){
    x_ = -0.39;
    y_ = 1.61;
  }
  else if(destination_ == "cocina"){
    x_ = 2.21;
    y_ = -0.96;
  }
  else if(destination_ == "despensa"){
    x_ = 3.02;
    y_ = 0.97;
  }
  ROS_INFO("Destination: %s\n", destination_.c_str());
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

void Navigate::sendNavigationGoal(void)
{
  ROS_INFO("Sending goal");
  ac_.sendGoal(goal_, NULL, MoveBaseClient::SimpleActiveCallback(),
     boost::bind(&Navigate::feedbackCb, this, _1));
  ac_.waitForResult();
}

} //practica4
