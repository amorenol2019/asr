#include <ros/ros.h>
#include "practica3/Practica3Impl.hpp"

namespace practica_3impl
{
  bool Practica3Impl::ToBall_2_ToBlueGoal()
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
}
