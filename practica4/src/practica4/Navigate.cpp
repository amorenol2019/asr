#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
Navigate::Navigate(ros::NodeHandle& nh) : ac_("move_base", true) // true?
{
  nh.getParam("destination", destination_);
  // x_ = destination_[1];
  // y_ = destination_[2];

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
