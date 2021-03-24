#include "practica3/Practica3Impl.hpp"

#include "std_msgs/Bool.h"
#include "ros/ros.h"

namespace practica3
{
Practica3Impl::Practica3Impl()
{
  object_sub_ = nh_.subscribe("object", 10, &Practica3Impl::object_callback, this);
  state_ts_ = ros::Time::now();
}

void
Practica3Impl::object_callback(const std_msgs::Bool::ConstPtr msg)
{
  // Segun lo que recibamos: distancia, objeto...
  // if msg es de la bola:    msg->data o algo asi
  object_ = "ball";
  // else if msg es de la p azul
  // object_ = "blue";
  // else if msg es de la p amarilla
  // object_ = "yellow";
}

bool Practica3Impl::ToBall_2_ToBlueGoal()
{
  return (ros::Time::now() - state_ts_).toSec > FORWARD_TIME;
}

bool Practica3Impl::ToBlueGoal_2_ToYellGoal()
{
  return (ros::Time::now() - state_ts_).toSec > FORWARD_TIME;
}

bool Practica3Impl::ToYellGoal_2_Turn()
{
  return (ros::Time::now() - state_ts_).toSec > FORWARD_TIME;
}

bool Practica3Impl::Turn_2_ToBall() 
{
  return (ros::Time::now() - state_ts_).toSec > TURNING_TIME;
}

} // practica3
