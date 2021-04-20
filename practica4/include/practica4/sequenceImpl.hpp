#ifndef PRACTICA4__SEQUENCE_IMPL_HPP__
#define PRACTICA4__SEQUENCE_IMPL_HPP__

#include <bica/Component.h>
#include "sequence.h"
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"

namespace practica4
{
class sequenceImpl : public bica::sequence
{
  sequenceImpl();

  void carreta_code_once();
  void esquina_code_once();
  void cajas_code_once();
  void contenedor_code_once();

  bool esquina_2_carreta();
  bool contenedor_2_esquina();
  bool cajas_2_contenedor();
  bool carreta_2_cajas();

private:
  void stateCb(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Publisher dest_pub_;
  ros::Subscriber finish_sub_;

  bool finished_;
};

}//practica4

#endif // PRACTICA3__SEQUENCE_IMPL_HPP__
