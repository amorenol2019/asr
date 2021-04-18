#ifndef PRACTICA4__PRACTICA_4_IMPL_HPP__
#define PRACTICA4__PRACTICA_4_IMPL_HPP__

#include <bica/Component.h>
#include "practica4.h"
#include "ros/ros.h"

namespace practica4
{
class Practica4Impl : public bica::practica4
{
public:
  Practica4Impl();

  bool Last_2_First();
  bool Second_2_Third();
  bool First_2_Second();
  bool Third_2_Last();

private:

};

} // practica4

#endif // PRACTICA4__PRACTICA_4_IMPL_HPP__
