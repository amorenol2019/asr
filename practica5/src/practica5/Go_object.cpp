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
  : BT::ActionNodeBase(name, config), nh_("~"), buffer_(), arrived_(false), listener_(buffer_), already_seen(false)
  {
    detect_sub_ = nh_.subscribe("/detected", 1, &Go_object::detectCB, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  }

  void Go_object::detectCB(const std_msgs::Bool::ConstPtr& msg)
  {
    detected_ = msg->data;
  }

  void Go_object::centre_2object()
  {
    bool centered = false;
    bool near = false;

    //movimiento angular
    if(detected_)
    {
      if(!already_seen)
      {
        seen_ = ros::Time::now();
        already_seen = true;
      }
      float speed = speed_4angle(angle_);
      vel.angular.z = speed;
      if(speed == 0.0)
      {
        centered = true;
      }
    }
    else {
      vel.angular.z = 0.1;
    }

    //movimiento lineal
    if(distance_ > 1)
    {
      vel.linear.x = LINEAR_VEL;
    }
    else if(distance_ <= 1.5 || !detected_)
    {
      vel.linear.x = 0;
      near = true;
    }

    if(centered && near)
    {
      arrived_ = true;
      seen_ = ros::Time::now();
      vel.linear.x = 0;
      vel.angular.z = 0;
    }
    vel_pub_.publish(vel);
  }

  float Go_object::speed_4angle(float angle)
  {
    float speed;
    if(angle_ == 200)
    {
      speed = 0.00001;
    }
    else if(0.3 >= fabs(angle_))
    {
      speed = 0.0;
    }
    else if(angle_ > 0.3)
    {
      speed = ANGULAR_VEL;
    }
    else if(angle < 0.3)
    {
      speed = -ANGULAR_VEL;
    }
    return speed;
  }

  void Go_object::look4_TF(const std::string name)
  {
    geometry_msgs::TransformStamped bf2obj_msg;
    try {
      bf2obj_msg = buffer_.lookupTransform("base_footprint", name, ros::Time(0));
    }
    catch (std::exception & e)
    {
      angle_ = 200; // angulo imposible
      distance_ = -100;
      return;
    }
    angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
    distance_ = bf2obj_msg.transform.translation.x;
  }

  BT::NodeStatus Go_object::tick()
  {
    object_ = getInput<std::string>("target").value();
    look4_TF(object_);
    if(arrived_)
    {
      angle_ = 200;
    }
    centre_2object();

    if(arrived_ && detected_)
    {
      ROS_INFO("HOORRAY , HE CENTRADO EN OBJETO");
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  void Go_object::halt()
  {
    ROS_INFO("HOORRAY , HE CENTRADO EN OBJETO");
  }
} // practica5
