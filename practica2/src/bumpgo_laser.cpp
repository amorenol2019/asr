// Copyright 2019 Intelligent Robotics Lab
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

#include "practica2/Detector.h"
#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0

int main(int argc, char **argv)
{
  practica2::Detector detector;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    detector.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
