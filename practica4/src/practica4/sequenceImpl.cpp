#include "ros/ros.h"
#include "practica4/sequenceImpl.hpp"
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"
#include <bica/Component.h>

namespace practica4
{
  sequenceImpl::sequenceImpl()
  {
    dest_pub_ = nh_.advertise<std_msgs::String>("/destination", 10);
    finish_sub_ = nh_.subscribe<std_msgs::Bool>("/finish", 10, &sequenceImpl::stateCb, this);
  }

  void sequenceImpl::stateCb(const std_msgs::Bool::ConstPtr& msg)
  {
    finished_ = msg->data;
  }

  void sequenceImpl::carreta_code_once()
  {
    std_msgs::String msg;
    msg.data = "carreta";
    dest_pub_.publish(msg);
  }
  void sequenceImpl::esquina_code_once()
  {
    std_msgs::String msg;
    msg.data = "esquina";
    dest_pub_.publish(msg);
  }
  void sequenceImpl::cajas_code_once()
  {
    std_msgs::String msg;
    msg.data = "cajas";
    dest_pub_.publish(msg);
  }
  void sequenceImpl::contenedor_code_once()
  {
    std_msgs::String msg;
    msg.data = "contenedor";
    dest_pub_.publish(msg);
  }

  bool sequenceImpl::esquina_2_carreta()
  {
    ROS_INFO("Voy a carreta");
    return finished_;
  }
  bool sequenceImpl::contenedor_2_esquina()
  {
    ROS_INFO("Voy a esquina");
    return finished_;
  }
  bool sequenceImpl::cajas_2_contenedor()
  {
    ROS_INFO("Voy a contenedor");
    return finished_;
  }
  bool sequenceImpl::carreta_2_cajas()
  {
    ROS_INFO("Voy a cajas");
    return finished_;
  }

}//practica4
