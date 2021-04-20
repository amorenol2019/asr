#include "ros/ros.h"
#include "practica4/sequenced_destinationImpl.hpp"

namespace practica4
{
  sequenced_destinationImpl()
  {
    dest_pub_ = nh_.advertise<std_msgs::String>("/destination", 10);
  }

  void carreta_code_iterative
  {
    dest_pub_.publish("carreta");
  }
  void esquina_code_iterative()
  {
    dest_pub_.publish("esquina");
  }
  void caja_code_iterative()
  {
    dest_pub_.publish("caja");
  }
  void contenedor_code_iterative()
  {
    dest_pub_.publish("contenedor");
  }

  bool esquina_2_contenedor()
  {

  }
  bool contenedor_2_caja()
  {

  }
  bool caja_2_carreta()
  {

  }
  bool carreta_2_esquina()
  {
    
  }

}//practica4
