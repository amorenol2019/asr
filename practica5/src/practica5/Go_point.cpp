#include "practica5/Go_point.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace practica5
{ // herencia
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  Go_point::Go_point(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config), nh_("~")
  {
    /*
    ,  const std::string& destination)
    , destination_(destination)


  if(need_param_){
    nh_.getParam("destination", destination_);
    set_coordinates();
  }
  */
    first_time_ = true;
  }


  void Go_point::set_coordinates(move_base_msgs::MoveBaseGoal& goal){
    if(destination_ == "carreta"){
      goal_.target_pose.pose.position.x = -0.5;
      goal_.target_pose.pose.position.y = 9.0;
      goal_.target_pose.pose.orientation.w = 0.05;
    }
    else if(destination_ == "cajas"){
      goal_.target_pose.pose.position.x = -3.5;
      goal_.target_pose.pose.position.y = -2.5;
      goal_.target_pose.pose.orientation.w = 0.05;
    }
    else if(destination_ == "esquina"){
      goal_.target_pose.pose.position.x = 4.0;
      goal_.target_pose.pose.position.y = -8.5;
      goal_.target_pose.pose.orientation.w = 0.05;
    }
    else if(destination_ == "contenedor"){
      goal_.target_pose.pose.position.x = 2.5;
      goal_.target_pose.pose.position.y = -7.0;
      goal_.target_pose.pose.orientation.w = 0.05;
    }
    else
    {
      ROS_INFO("Destination invalid");
    }
    ROS_INFO("Destination: %s\n", destination_.c_str());
  }

  void Go_point::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    double current_x = feedback->base_position.pose.position.x;
    double current_y = feedback->base_position.pose.position.y;

    double diff_x = goal_x_ - current_x;
    double diff_y = goal_y_ - current_y;

    double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
    ROS_INFO("Distance to %s = %lf", destination_.c_str(), distance);
  }

  void Go_point::halt() {
    ROS_INFO("GoPoint halt");
  }

  BT::NodeStatus Go_point::tick(){
    ROS_INFO("GoPoint tick");

    std::string destination_ = getInput<std::string>("destination").value();

    MoveBaseClient ac("move_base",true);
    if (first_time_)
    {
      while (!ac.waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      goal_.target_pose.header.frame_id = "map";
      set_coordinates(goal_);
      goal_.target_pose.header.stamp = ros::Time::now();

      ROS_INFO("Sending goal");
      ac.sendGoal(goal_, NULL, MoveBaseClient::SimpleActiveCallback(),
        boost::bind(&Go_point::feedbackCb, this, _1));
      first_time_ = false;
    }

    // ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
      ROS_INFO("Mission is being accomplished");
      return BT::NodeStatus::RUNNING;
    }
    else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, mission accomplished");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("[Error] mission could not be accomplished");
      return BT::NodeStatus::FAILURE;
    }
  }
} // practica5
