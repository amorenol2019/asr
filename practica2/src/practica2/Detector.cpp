
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
    sub_detect_ = n_.subscribe("/scan",1,&Detector::detectorCallback, this);
    pub_detect_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_marker_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_msgs/Markers",1);
  }

  void Detector::detectorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    pressedFront_ =  msg->ranges[msg->ranges.size()/2] <= MIN_DISTANCE;
    float posiciones = (M_PI/5) / msg->angle_increment;
    pressedRight_ = msg->ranges[msg->ranges.size()/2 - posiciones] <= MIN_DISTANCE;
    pressedLeft_ = msg->ranges[msg->ranges.size()/2 + posiciones] <= MIN_DISTANCE;
  }

  void Detector::step()
  {
    geometry_msgs::Twist cmd;

    srand(time(NULL));

    turning_time_ = rand()%(MAX_TURNING_TIME -  MIN_TURNING_TIME);
    turning_time_ += MIN_TURNING_TIME;

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

      cmd.angular.z = 0.2;
      pressedRight_ = false;

      if ((ros::Time::now() - turn_ts_).toSec() > turning_time_ )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

      cmd.angular.z = -0.2;
      pressedLeft_ = true;

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
    marker_front.pose.position.y = 1;
    marker_front.pose.position.z = 1;
    marker_front.pose.orientation.x = 0.0;
    marker_front.pose.orientation.y = 0.0;
    marker_front.pose.orientation.z = 0.0;
    marker_front.pose.orientation.w = 1.0;
    marker_front.scale.x = 0.5;
    marker_front.scale.y = 0.5;
    marker_front.scale.z = 0.5;
    marker_front.color.a = 1.0; // Don't forget to set the alpha!
    marker_front.color.r = 0.0;
    marker_front.color.g = 1.0;
    marker_front.color.b = 0.0;
    marker_front.lifetime = ros::Duration(1.0);

    marker_left = marker_front;
    marker_left.id = 1;
    //hay que cambiar la posicion de estos dos Markers
    //marker_left.x = nueva x  pi/5 = 36º -> 90-36=54º:
    marker_left.pose.position.x = sin(54);
    //marker_left.y = nueva y
    marker_left.pose.position.y = cos(54);
    marker_right = marker_left;
    marker_right.id = 2;
    //marker_right.y = nueva y
    marker_right.pose.position.y =-cos(54);

    marker_arr.markers.push_back(marker_front);
    marker_arr.markers.push_back(marker_left);
    marker_arr.markers.push_back(marker_right);

    pub_marker_.publish(marker_arr);
  }

} // namespace practica2
