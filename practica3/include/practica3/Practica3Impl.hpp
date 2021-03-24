#ifndef PRACTICA3__PRACTICA_3_IMPL_HPP__
#define PRACTICA3__PRACTICA_3_IMPL_HPP__

#include "practica3.h"
#include "std_msgs/Bool.h"

#include "ros/ros.h"

namespace practica3
{
class Practica3Impl : public practica3
{
public:
    Practica3Impl() ;

  bool ToBall_2_ToBlueGoal() ;
  bool ToBlueGoal_2_ToYellGoal();
  bool ToYellGoal_2_Turn() ;
  bool Turn_2_ToBall() ;

private:
  void object_callback(const std_msgs::Bool::ConstPtr msg);

  ros::NodeHandle nh_;
  ros::Subscriber object_sub_; // Suscriptor a lo que produce Perception

  const float FORWARD_TIME = 30.0;
  const float TURNING_TIME = 5.0;

  std::string object_;

};

}; // practica3

#endif // PRACTICA3__PRACTICA_3_IMPL_HPP__
