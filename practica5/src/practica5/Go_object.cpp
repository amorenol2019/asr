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
  //typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  Go_object::Go_object(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config), nh_("~") ,buffer_(), arrived_(false), listener_(buffer_),already_seen(false)
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
    ROS_INFO("angle : %lf",angle_);
    if(detected_)// si lo esta viendo
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
    else
    {
      vel.angular.z = 0.1;
      /*if(angle_== 200)// si no hay tf
      {
        vel.angular.z = 0.3;
      }
      else
      {
        vel.angular.z = speed_4angle(angle_);
      }
      */
    }

    //movimiento lineal
    if( distance_ > 1 )//|| ( (ros::Time::now() - seen_).toSec() > 3.0 &&(ros::Time::now() - seen_).toSec() < 15.0 && distance_ == -100) )
    {
      ROS_INFO("distancia > 1 O mucho tiempo desde que o vi");
      vel.linear.x = LINEAR_VEL;
    }
    else if(distance_ <= 1.5 || !detected_)
    {
      ROS_INFO("DISTANCIA < 1 O !detected_");
      ROS_INFO("(%lf)", distance_);
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
      ROS_INFO("0:no TF");
      speed = 0.00001;
    }
    else if(0.3 >= fabs(angle_))
    {
      ROS_INFO("I: cnetrado ");
      speed = 0.0;
    }
    else if(angle_ > 0.3)
    {
      ROS_INFO("M: > 0.8");
      speed = ANGULAR_VEL;
    }
    else if(angle < 0.3)
    {
      ROS_INFO("m: < 0.8");
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
    // angulo del robot respecto al objecto
    angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
    // tambien quiero obtener la distancia hasta el objecto
    distance_ = bf2obj_msg.transform.translation.x; //nose si esto esta bien;  car: creo que si
  }

  BT::NodeStatus Go_object::tick()
  {
    object_ = getInput<std::string>("target").value();
    look4_TF(object_);
    if(arrived_)
    {
      angle_ =200;
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

  /*
  bool Go_object::going2object()
  {
      geometry_msgs::TransformStamped bf2object_msg;

      try
      {
          bf2object_msg = buffer_.lookupTransform("base_footprint", object_, ros::Time(0));
      }
      catch (std::exception & e)
      {
          return false;
      }

      goal_.target_pose.header.frame_id = "base_footprint";
      goal_.target_pose.pose.orientation = bf2object_msg.transform.rotation;
      goal_.target_pose.pose.position.x = bf2object_msg.transform.translation.x - 1; //Idea de cuando está a menos 1
      goal_.target_pose.pose.position.y = bf2object_msg.transform.translation.y;   //O aqui
      goal_.target_pose.pose.position.z = bf2object_msg.transform.translation.z;

      return true;
  }

  BT::NodeStatus Go_object::tick()
  {
      object_ = getInput<std::string>("target").value();
      MoveBaseClient ac("move_base",true);
      if (!(going2object() ))
      {
          return BT::NodeStatus::FAILURE;
      }

      while (!ac.waitForServer(ros::Duration(5.0)))
      {
          ROS_INFO("Waiting for the move_base action server to come up");
      }
      goal_.target_pose.header.stamp = ros::Time::now();

      ROS_INFO("Sending goal");
      ac.sendGoal(goal_);
      ac.waitForResult();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
  */

  void Go_object::halt()
  {
    ROS_INFO("HOORRAY , HE CENTRADO EN OBJETO");
  }
} // practica5
