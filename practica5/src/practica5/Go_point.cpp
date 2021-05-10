#include "practica5/Go_point.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica5
{
  Go_point::Go_point(bool need_param_) : nh_("~"), ac_("move_base", true)
  {
    /*
  if(need_param_){
    nh_.getParam("destination", destination_);
    set_coordinates();
  }
  */
  set_coordinates();
  }

  void Go_point::set_coordinates(){
    if(destination_ == "carreta"){
      x_ = -0.5;
      y_ = 9.0;
    }
    else if(destination_ == "cajas"){
      x_ = -3.5;
      y_ = -2.5;
    }
    else if(destination_ == "esquina"){
      x_ = 4.0;
      y_ = -8.5;
    }
    else if(destination_ == "contenedor"){
      x_ = 2.5;
      y_ = -7.0;
    }
    ROS_INFO("Destination: %s\n", destination_.c_str());
  }

  void Go_point::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
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

  void Go_point::sendNavigationGoal(void)
  {
    ROS_INFO("Sending goal");
    ac_.sendGoal(goal_, NULL, MoveBaseClient::SimpleActiveCallback(),
       boost::bind(&Navigate::feedbackCb, this, _1));
    ac_.waitForResult();
  }

  void Go_point::halt() {

  }

  BT::NodeStatus Go_point::tick(){

  }

} // practica5
