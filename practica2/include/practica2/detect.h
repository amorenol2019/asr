
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

#ifndef PRACTICA2_DETECT_H
#define PRACTICA2_DETECT_H

namespace practica2 // Para que sirve?
{

class Detect
{
  public:
    Detect(): state_(GOING_FORWARD), pressed_(false);
    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void step();

  private:
    ros::NodeHandle n_;
    static const int GOING_FORWARD   = 0;
    static const int GOING_BACK   = 1;
    static const int TURNING     = 2;

    int state_;

    bool pressed_;

    ros::Time press_ts_;
    ros::Time turn_ts_;

    ros::Subscriber sub_bumber_;
    ros::Publisher pub_vel_;
};

}  // namespace practica2

#endif  // practica2
