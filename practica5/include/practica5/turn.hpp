
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace practica5
{
  class Turn
{
public:
  Turn(void) ;
  void halt();
  BT::NodeStatus tick();

private:
  ros::NodeHandle nh_;
  geometry_msgs::Twist vel;
  ros::Time beggining_time;

  const float VELOCITY = 0.1;
  float TURNING_TIME = 1.0;

};

}//namespace practica5
