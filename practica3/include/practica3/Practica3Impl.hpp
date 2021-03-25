#ifndef PRACTICA3__PRACTICA_3_IMPL_HPP__
#define PRACTICA3__PRACTICA_3_IMPL_HPP__

#include <bica/Component.h>
#include "practica3.h"
#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"

namespace practica3
{
class Practica3Impl : public bica::practica3
{
public:
  Practica3Impl() ;

  bool ToBall_2_ToBlueGoal() ;
  bool ToBlueGoal_2_ToYellGoal();
  bool ToYellGoal_2_Turn() ;
  bool Turn_2_ToBall() ;

  void ToYellGoal_code_iterative() ;
  void Turn_code_iterative();
  void ToBlueGoal_code_iterative();
  void ToBall_code_iterative();

private:
  //void object_callback(const std_msgs::Bool::ConstPtr msg);

  ros::NodeHandle nh_;
  ros::Publisher object_pub_;

  std_msgs::Int64 msg_; //String msg_;

  const float FORWARD_TIME = 30.0;
  const float TURNING_TIME = 5.0;

};

} // practica3

#endif // PRACTICA3__PRACTICA_3_IMPL_HPP__
