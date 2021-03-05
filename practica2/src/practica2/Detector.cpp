
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
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

namespace practica2
{
  Detector::Detector(): state_(GOING_FORWARD), pressedFront_(false)
  {
    sub_detect_ = n_.subscribe("/scan", 1, &Detector::detectorCallback, this);
    pub_detect_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_marker_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_msgs/Markers", 1);
  }

  void Detector::detectorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    float posiciones = ANGLE / msg->angle_increment; // Diferencia de posiciones en 'ranges' correspondientes a pi/5

    pressedFront_ = msg->ranges[msg->ranges.size() / 2] < MIN_DISTANCE;
    pressedRight_ = msg->ranges[msg->ranges.size() / 2 - posiciones] < MIN_DISTANCE;
    pressedLeft_ = msg->ranges[msg->ranges.size() / 2 + posiciones] < MIN_DISTANCE;
  }

  void Detector::step()
  {
    geometry_msgs::Twist cmd;

    srand(time(NULL));
    turning_time_ = MIN_TURNING_TIME + rand()%(MAX_TURNING_TIME - MIN_TURNING_TIME);

    switch (state_)
    {
    case GOING_FORWARD:

      cmd.linear.x = VELOCITY;

      if (pressedFront_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }
      break;

    case GOING_BACK:

      cmd.linear.x = - VELOCITY;
      pressedFront_ = false;

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

      cmd.angular.z = VELOCITY;
      pressedRight_ = false;

      if ((ros::Time::now() - turn_ts_).toSec() > turning_time_ )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

      cmd.angular.z = - VELOCITY;
      pressedLeft_ = false;

      if ((ros::Time::now() - turn_ts_).toSec() > turning_time_ )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_RIGHT -> GOING_FORWARD");
      }
      break;
    }
    pub_detect_.publish(cmd);
  }

  void Detector::visualize()
  {
    visualization_msgs::Marker marker_front;
    visualization_msgs::Marker marker_right;
    visualization_msgs::Marker marker_left;
    visualization_msgs::MarkerArray marker_arr;

    marker_front.header.frame_id = "base_link"; // el frame debe estar dentro del robot
    marker_front.header.stamp = ros::Time();
    marker_front.ns = "my_namespace";
    marker_front.id = 0;
    marker_front.type = visualization_msgs::Marker::SPHERE;
    marker_front.action = visualization_msgs::Marker::ADD;
    marker_front.pose.position.x = 1;
    marker_front.pose.position.y = 0;
    marker_front.pose.position.z = 0;
    marker_front.pose.orientation.x = 0.0;
    marker_front.pose.orientation.y = 0.0;
    marker_front.pose.orientation.z = 0.0;
    marker_front.pose.orientation.w = 1.0;
    marker_front.scale.x = 0.25;
    marker_front.scale.y = 0.25;
    marker_front.scale.z = 0.25;
    marker_front.color.a = 1.0;
    marker_front.color.r = 0.0;
    marker_front.color.g = 1.0;
    marker_front.color.b = 0.0;
    marker_front.lifetime = ros::Duration(1.0);

    marker_left = marker_front;
    marker_left.id = 1;
    marker_left.pose.position.x = cos(ANGLE);
    marker_left.pose.position.y = sin(ANGLE);

    marker_right = marker_left;
    marker_right.id = 2;
    marker_right.pose.position.y = - sin(ANGLE);

    if (pressedFront_)
    {
      marker_front.color.g = 0.0;
      marker_front.color.r = 1.0;
    }
    if(pressedRight_)
    {
      marker_right.color.g = 0.0;
      marker_right.color.r = 1.0;
    }
    if(pressedLeft_)
    {
      marker_left.color.g = 0.0;
      marker_left.color.r = 1.0;
    }

    marker_arr.markers.push_back(marker_front);
    marker_arr.markers.push_back(marker_left);
    marker_arr.markers.push_back(marker_right);

    pub_marker_.publish(marker_arr);
  }

} // namespace practica2
