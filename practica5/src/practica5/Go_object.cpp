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
  : BT::ActionNodeBase(name, config), nh_("~") ,buffer_(), arrived_(false), listener_(buffer_)
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

    look4_TF(object_);

    ROS_INFO_STREAM(detected_);
    //movimiento angular
    if (0.8 >= fabs(angle_) && detected_)
    {
      ROS_INFO("(%lf)", angle_);
      ROS_INFO("ANGULO INTERMEDIO");
      vel.angular.z = 0;
      centered = true;
    }
    else if(angle_ > 0.8 && detected_) //IZQUIERDA
    {
      ROS_INFO("(%lf)", angle_);
      ROS_INFO("ANGULO > 0.8");
      vel.angular.z = ANGULAR_VEL;
    }
    else
    {
      ROS_INFO("(%lf)", angle_);
      ROS_INFO("ANGULO < -0.8");
      vel.angular.z = -ANGULAR_VEL;
    }

    //movimiento lineal
    if(distance_ > 1 && detected_)
    {
      ROS_INFO("(%lf)", distance_);
      ROS_INFO("DISTANCIA > 1");
      vel.linear.x = LINEAR_VEL;
    }
    else
    {
      ROS_INFO("DISTANCIA < 1");
      ROS_INFO("(%lf)", distance_);
      vel.linear.x = 0;
      near = true;
    }
    vel_pub_.publish(vel);

    if(centered && near)
    {
      arrived_ = true;
    }
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
      return;
    }
    // angulo del robot respecto al objecto
    angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
    // tambien quiero obtener la distancia hasta el objecto
    distance_ = bf2obj_msg.transform.translation.x; //nose si esto esta bien;  car: creo que si
  }

  BT::NodeStatus Go_object::tick()
  {
    arrived_ = false;
    object_ = getInput<std::string>("target").value();
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
