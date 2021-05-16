#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>
#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "practica5/Go_object.hpp"

namespace practica5
{
  Go_object::Go_object(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config), nh_("~"), buffer_(), listener_(buffer_)
  {
    detect_sub_ = nh_.subscribe("/detected", 10, &Go_object::detectCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  }

  void Go_object::detectCb(const std_msgs::Bool::ConstPtr& msg)
  {
    detected_ = msg->data;
  }

  void Go_object::look4_TF(const std::string name , float *angle , float *distance)
  {
      geometry_msgs::TransformStamped bf2obj_msg;
      try {
        bf2obj_msg = buffer_.lookupTransform("base_footprint", name, ros::Time(0));
      }
      catch (std::exception & e)
      {
        *angle = IMPOSIBLE_ANGLE;
        *distance = -100;
        return;
      }
      *angle = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
      *distance = bf2obj_msg.transform.translation.x;
  }

  bool Go_object::centre_2object(float angle , float distance)
  {
    bool near = false;
    bool centered = false;
    bool arrived = false;

    //si la tf no esta creada se acerca al objecto
    if(angle == IMPOSIBLE_ANGLE)
    {
      vel_.linear.x= LINEAR_VEL;
      vel_pub_.publish(vel_);
      return false;
    }

    //si la tf esta creada centra el objecto
    if( fabs(angle) <= 0.3)
    {
      vel_.angular.z = 0;
      centered = true;
    }
    else if(angle > 0.3)
    {
      vel_.angular.z = ANGULAR_VEL;
    }
    else if(angle < -0.3)
    {
      vel_.angular.z = -ANGULAR_VEL;
    }

    //si la tf esta creada se acerca al objecto
    //movimiento lineal
    if(distance > MIN_DISTANCE)
    {
      vel_.linear.x = LINEAR_VEL;
    }
    else if(distance <= MIN_DISTANCE )
    {
      vel_.linear.x = 0;
      near = true;
    }

    if(near && centered)
    {
      arrived = true;
    }

    vel_pub_.publish(vel_);
    
    return arrived;
  }

  BT::NodeStatus Go_object::tick()
  {
      if(!detected_)
      {
        return BT::NodeStatus::FAILURE;
      }

      std::string object = getInput<std::string>("target").value();

      float angle , distance;
      look4_TF( object , &angle , &distance);
      bool arrived = centre_2object( angle  ,distance );

      if(arrived)
      {
        ROS_INFO("HOORRAY , HE LLEGADO AL OBJECTO");
        return BT::NodeStatus::RUNNING;
      }
      else
      {
        return BT::NodeStatus::RUNNING;
      }
  }
  void Go_object::halt()
  {
    ROS_INFO("halt Go _object");
  }

} // practica5
