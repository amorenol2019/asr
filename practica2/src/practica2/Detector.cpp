
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

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

namespace practica2
{
  Detector(): state_(GOING_FORWARD), pressed_(false)
  {
    sub_detect = n_.subscribe("/scan",1,&Detector::detectorCallback, this);
    pub_detect = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void detectorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    float front =  msg->ranges[msg->ranges.size()/2];
    float right = msg->ranges[msg->ranges.size()/2 - msg->ranges.size()/10];
    float left = msg->ranges[msg->ranges.size()/2 - msg->ranges.size()/10];

    if (front <= min_distance_)
    {
      pressedFront_ = true;
    }
    if (right <= min_distance_)
    {
      pressedRight_ = true;
    }
    if (left <= min_distance_)
    {
      pressedLeft_ = true;
    }
  }

  void step()
  {
    geometry_msgs::Twist cmd;

    switch (state_)
    {
    case GOING_FORWARD:

      if (pressedFront_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }
      break;

    case GOING_BACK:

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        if (pressedLeft_)
        {
          state_ = TURNING_RIGHT;
          ROS_INFO("GOING_BACK -> TURNING_RIGHT");
        }
        else
        {
          state = TURNING_LEFT;
          ROS_INFO("GOING_BACK -> TURNING_LEFT");
        }
      }
      break;

    case TURNING_LEFT:

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }
  }

}
