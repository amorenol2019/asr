#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{                                        //tell the action client that we want to spin a thread by default
Navigate::Navigate() : ac_("move_base", true) {} //x(0.0),y(0.0)

void Navigate::doneCb(const actionlib::SimpleClientGoalState& state,
  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Terminado!!");
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

  double dist = sqrt(diff_x * diff_x + diff_y * diff_y);

  ROS_INFO("Distance to goal = %lf", dist);
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

}//practica4
