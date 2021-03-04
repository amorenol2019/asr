
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

      cmd.linear.x =0.2;

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

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

      cmd.angular.z = -0.2;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }
  }

  MarkerArray Detector::visualize(Marker marker_front,Marker marker_right,Marker marker_left)
  {
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
    marker_front_front.pose.orientation.y = 0.0;
    marker_front.pose.orientation.z = 0.0;
    marker_front.pose.orientation.w = 1.0;
    marker_front.scale.x = 1;
    marker_front.scale.y = 0.1;
    marker_front.scale.z = 0.1;
    marker_front.color.a = 1.0; // Don't forget to set the alpha!
    marker_front.color.r = 0.0;
    marker_front.color.g = 1.0;
    marker_front.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker_front.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    marker_left = marker_front;
//hay qu ecambiar la posicion de estos dos Markers
    //marker_left.x = nueva x  pi/5 = 36º -> 90-36=54º:
    marker_left.x = sin(54);
    //marker_left.y = nueva y
    marker_left.y = cos(54);
    marker_right = marker_left;
    //marker_right.y = nueva y
    marker_right.y =-cos(54);

    marker_arr.Markers.pushback(marker_front);
    marker_arr.Markers.pushback(marker_left);
    marker_arr.Markers.pushback(marker_right);

    return marker_arr;
  }

}
