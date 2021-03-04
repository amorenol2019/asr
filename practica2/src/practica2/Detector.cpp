
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

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray"
#include "visualization_msgs/Marker"

namespace practica2
{
  Detector::Detector(): state_(GOING_FORWARD), pressedFront_(false)
  {
    sub_detect_ = n_.subscribe("/scan",1,&Detector::detectorCallback, this);
    pub_detect_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_marker_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_msgs/Markers",1);
  }

  void Detector::detectorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    float front =  msg->ranges[msg->ranges.size()/2];
    float right = msg->ranges[msg->ranges.size()/2 - msg->ranges.size()/10];
    float left = msg->ranges[msg->ranges.size()/2 + msg->ranges.size()/10];

    if (front <= MIN_DISTANCE)
    {
      pressedFront_ = true;
    }
    if (right <= MIN_DISTANCE)
    {
      pressedRight_ = true;
    }
    if (left <= MIN_DISTANCE)
    {
      pressedLeft_ = true;
    }
  }

  void Detector::step()
  {
    geometry_msgs::Twist cmd;
    //todo esto de los markers podria ir en una funcion (?)
    visualization_msgs::Marker marker_front;
    visualization_msgs::Marker marker_right;
    visualization_msgs::Marker marker_left;
    visualization_msgs::MarkerArray marker_arr;

    marker_arr = visualize(marker_front,marker_right,marker_left);
    pub_marker_.publish( marker_arr );


    switch (state_)
    {
    case GOING_FORWARD:

      cmd.linear.x = 0.2;

      if (pressedFront_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }
      break;

    case GOING_BACK:

      cmd.linear.x = -0.2;

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
          state_ = TURNING_LEFT;
          ROS_INFO("GOING_BACK -> TURNING_LEFT");
        }
      }
      break;

    case TURNING_LEFT:

      cmd.angular.z = 0.2;

      if ((ros::Time::now() - turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

      cmd.angular.z = -0.2;

      if ((ros::Time::now() - turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }
  }

} // namespace practica2