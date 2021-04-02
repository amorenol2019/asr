#include "practica3/Practica3Impl.hpp"

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "std_msgs/Int64.h"

namespace practica3
{
Practica3Impl::Practica3Impl()
{
  state_ts_ = ros::Time::now();
}

bool Practica3Impl::ToBall_2_ToBlueGoal()
{
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::ToBlueGoal_2_ToYellGoal()
{
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::ToYellGoal_2_Turn()
{
  return (ros::Time::now() - state_ts_).toSec() > FORWARD_TIME;
}

bool Practica3Impl::Turn_2_ToBall()
{
  return (ros::Time::now() - state_ts_).toSec() > TURNING_TIME;
}

} // practica3
