#include "ros/ros.h"
#include "practica4/SequenceImpl.hpp"
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"
#include <bica/Component.h>

namespace practica4
{
  SequenceImpl::SequenceImpl()
  {
    dest_pub_ = nh_.advertise<std_msgs::String>("/destination", 10);
    finish_sub_ = nh_.subscribe<std_msgs::Bool>("/finish", 10, &SequenceImpl::stateCb, this);
  }

  void SequenceImpl::stateCb(const std_msgs::Bool::ConstPtr& msg)
  {
    finished_ = msg->data;
  }
  /*
  void SequenceImpl::carreta_code_once()
  {
    msg_.data = "carreta";
    dest_pub_.publish(msg_);
  }
  void SequenceImpl::esquina_code_once()
  {
    msg_.data = "esquina";
    dest_pub_.publish(msg_);
  }
  void SequenceImpl::cajas_code_once()
  {
    msg_.data = "cajas";
    dest_pub_.publish(msg_);
  }
  void SequenceImpl::contenedor_code_once()
  {
    msg_.data = "contenedor";
    dest_pub_.publish(msg_);
  }
  */
  bool SequenceImpl::esquina_2_carreta()
  {
    ROS_INFO("Voy a carreta");
    msg_.data = "carreta";
    dest_pub_.publish(msg_);
    return finished_;
  }
  bool SequenceImpl::contenedor_2_esquina()
  {
    ROS_INFO("Voy a esquina");
    msg_.data = "esquina";
    dest_pub_.publish(msg_);
    return finished_;
  }
  bool SequenceImpl::cajas_2_contenedor()
  {
    ROS_INFO("Voy a contenedor");
    msg_.data = "contenedor";
    dest_pub_.publish(msg_);
    return finished_;
  }
  bool SequenceImpl::carreta_2_cajas()
  {
    ROS_INFO("Voy a cajas");
    msg_.data = "cajas";
    dest_pub_.publish(msg_);
    return finished_;
  }

} //practica4
