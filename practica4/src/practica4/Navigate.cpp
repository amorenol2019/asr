#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
Navigate::Navigate(ros::NodeHandle& nh) : ac_("move_base", true) // true?
{
  nh.getParam("destination", destination_);
  if(destination_ == carreta_){
    ROS_INFO("Estado: %s\n", destination_.c_str());
    x_ = -0.5;
    y_ = 8.5;
  }
  else if(destination_ == cajas_){
    //ROS_INFO("Estado: %s\n", state_.c_str());
    x_ = -3.5;
    y_ = -2.5;
  }
  else if(destination_ == contenedor_){
    x_ = 1.5;
    y_ = -7;
  }
  else if (destination_ == derecha_superior_){
    x_ = 5.0;
    y_ = -0.5;
  }
  else{
    ROS_INFO("No parameter received");
    x_ = 2.0;
    y_ = 1.0;
  }

}

void Navigate::doneCb(const actionlib::SimpleClientGoalState& state,
  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  finished_ = true;
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
  ac_.sendGoal(goal_,
      boost::bind(&Navigate::doneCb, this, _1, _2),
      MoveBaseClient::SimpleActiveCallback(),
      boost::bind(&Navigate::feedbackCb, this, _1));
  ac_.waitForResult();
}

} //practica4
