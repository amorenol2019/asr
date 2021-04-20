#ifndef PRACTICA4__SEQUENCED_DESTINATION_IMPL_HPP__
#define PRACTICA4__SEQUENCED_DESTINATION_IMPL_HPP__

#include <bica/Component.h>
#include "sequenced_destination.h"
#include "ros/ros.h"

namespace practica4
{
  class sequenced_destinationImpl : public bica::practica3
{
  sequenced_destinationImpl();

  void carreta_code_iterative();
  void esquina_code_iterative();
  void caja_code_iterative();
  void contenedor_code_iterative();

  bool esquina_2_contenedor();
  bool contenedor_2_caja();
  bool caja_2_carreta();
  bool carreta_2_esquina();

private:
  std::bool reached_;
  ros::Publisher dest_pub_;
};

}//practica4

#endif // PRACTICA3__SEQUENCED__DESTINATION_IMPL_HPP__
