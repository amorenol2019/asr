#ifndef PRACTICA4__NAVIGATE_HPP__
#define PRACTICA4__NAVIGATE_HPP__

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace practica4
{
class Navigate
{
public:
  Navigate();
  void doneCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  int main(int argc, char** argv);

  float x_;
  float y_;

  move_base_msgs::MoveBaseGoal goal;
  bool finished_;

  //x e y inicializadas a 0
  //indican el punto al que se navegará


};

}//practica4

#endif // PRACTICA4__NAVIGATE_HPP__
