#include "practica3/Forward.hpp"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"


namespace practica3
{

Forward::Forward()
{
}

void
Forward::step()
{
  if(!isActive()){ // Componente de BICA
    return;
  }
}

} // practica3
