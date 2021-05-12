#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>
#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "practica5/Go_object.hpp"

namespace practica5
{
  Go_object::Go_object(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config), nh_("~") ,buffer_(), arrived_(false)
  {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  }

  void Go_object::centre_2object()
  {
    bool centered = false;
    bool near = false;

    look4_TF(object_);

    if(angle_ > 0) //esta hacia su izq?
    {
      vel.angular.z = ANGULAR_VEL;
    }
    else
    {
      vel.angular.z = -ANGULAR_VEL;
    }
    //habra que poner un rango en el que consideremos que esta centrado en funcion del angulo (ir probando)
    //cuando este dentro de ese rango centered = true

    if(distance_ > 1)
    {
      vel.linear.x = LINEAR_VEL;
    }
    else
    {
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
      angle_ = 400; // angulo imposible
      return;
    }
    // angulo del robot respecto al objecto
    angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
    // tambien quiero obtener la distancia hasta el objecto
    distance_ = bf2obj_msg.transform.translation.x; //nose si esto esta bien;  car: creo que si
  }

  BT::NodeStatus Go_object::tick()
  {
    object_ = getInput<std::string>("target").value();
    centre_2object();

    if(arrived_)
    {
      return BT::NodeStatus::SUCCESS;
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
