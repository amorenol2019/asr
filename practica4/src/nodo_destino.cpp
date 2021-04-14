#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typdef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// quitar variables globales:
bool finished = false;
move_base_msgs::MoveBaseGoal goal;

void doneCb(const actionlib::SimpleClientGoalState& state,
  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Terminado!!");
  finished = true;
}

void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  double goal_x = goal.target_pose.pose.position.x;
  double goal_y = goal.target_pose.pose.position.y;
  double current_x = feedback->base_position.pose.position.x;
  double current_y = feedback->base_position.pose.position.y;

  double diff_x = goal_x - current_x;
  double diff_y = goal_y - current_y;

  double dist = sqrt(diff_x * diff_x + diff_y * diff_y);

  ROS_INFO("Distance to goal = %lf", dist);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  MoveBaseClient ac("move_base", true); // cambiar

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base acation server to come up");
  }

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Posicion a la que ir
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 3.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal, doneCb);

  ros::Rate rate(1);
  while(ros::ok() && !finished) // cambiar segunda condicion
  {
    ROS_INFO("Esperando");
    rate.sleep();
  }

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move foward 1 meter for some reason");

  return 0;
}
