// Copyright 2021 Almudena, Blanca, Carlota y Jaime
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

class Bump
{
public:
  Bump()
  {
    is_pressed_ = false;
    //Se suscribe al bumper
    sub_bumper_ = n_.subscribe("/mobile_base/events/bumper", 1, &Bump::messageCallback, this);

    //Publica en el topic de los comandos de velocidad
    robot_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

  }

  void messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    is_pressed_ = msg->state == msg->PRESSED;
  }

  void move()
  {
    geometry_msgs::Twist vel;
    if(is_pressed_) vel.linear.x = 0.0;
    else vel.linear.x = 0.2;

    robot_vel_.publish(vel);
  }

private:
  bool is_pressed_;

  ros::NodeHandle n_;
  ros::Subscriber sub_bumper_;
  ros::Publisher robot_vel_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "bump");

  Bump bump;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    bump.move();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
