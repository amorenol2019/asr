
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

#ifndef PRACTICA2_DETECTOR_H
#define PRACTICA2_DETECTOR_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


namespace practica2
{

  class Detector
  {
    public:
      Detector();
      void detectorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
      void step();
      void visualize();

    private:
      ros::NodeHandle n_;
      static const int GOING_FORWARD   = 0;
      static const int GOING_BACK      = 1;
      static const int TURNING_LEFT    = 2;
      static const int TURNING_RIGHT   = 3;
      static const int MAX_TURNING_TIME = 5.0;
      static const int BACKING_TIME = 3.0;
      const int MIN_TURNING_TIME = 2.5;
      const float MIN_DISTANCE = 0.5;

      int state_;
      int turning_time_;

      bool pressedFront_;
      bool pressedRight_;
      bool pressedLeft_;

      ros::Time press_ts_;
      ros::Time turn_ts_;

      ros::Subscriber sub_detect_;
      ros::Publisher pub_detect_;
      ros::Publisher pub_marker_;
  };

}  // namespace practica2

#endif  // practica2
