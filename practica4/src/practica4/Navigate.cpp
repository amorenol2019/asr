#include "practica4/Navigate.hpp"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
Navigate::Navigate(bool need_arg,std::string arg) : nh_("~"), ac_("move_base", true)
{
  if(need_arg){
    destination_ = arg;
    set_coordinates();
  }
}

void Navigate::set_coordinates(){

  if(destination_ == "carreta"){
    nh_.getParam("X_CARRETA",x_);
    nh_.getParam("Y_CARRETA",y_);
  }
  else if(destination_ == "cajas"){
    nh_.getParam("X_CAJAS",x_);
    nh_.getParam("Y_CAJAS",y_);
  }
  else if(destination_ == "contenedor"){
    nh_.getParam("X_CONTENEDOR",x_);
    nh_.getParam("Y_CONTENEDOR",y_);
  }
  else if(destination_ == "esquina"){
    nh_.getParam("X_ESQUINA",x_);
    nh_.getParam("Y_ESQUINA",y_);
  }

  ROS_INFO("Destination: %s x:%f y:%f", destination_.c_str(),x_,y_);
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
  ac_.waitForResult(); //esto entonces habria que quitarlo de aqui !?
}

} //practica4
