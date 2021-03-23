#ifndef PRACTICA3IMPL_H_
#define PRACTICA3IMPL_H_

#include <ros/ros.h>
#include "practica_3impl"

namespace practica_3impl
{
class Practica3Impl : public practica_3impl
{
public:
  Practica3Impl() {}

  bool ToBlueGoal_2_ToYellGoal();
  bool ToBall_2_ToBlueGoal() ;
  bool ToYellGoal_2_Turn() ;
  bool Turn_2_ToBall() ;

private:
  const float FORWARD_TIME = 30.0;
  const float TURNING_TIME = 5.0;

};

}

#endif /* PRACTICA3IMPL_H_ */
