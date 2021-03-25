#include "practica3/Perception.hpp"

#include "geometry_msgs/Twist.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "ros/ros.h"
#include <string>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace practica3
{
Perception::Perception(): it_(nh_), buffer_() , listener_(buffer_)
{
  image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &Perception::imageCb, this);
  object_sub_ = nh_.subscribe("/object", 1, &Perception::objectCb, this);

  object_pub_ = nh_.advertise<std_msgs::Float32>("/distance", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void Perception::objectCb(const std_msgs::Int64::ConstPtr& msg)
{
  object_ = msg->data; //str->data;
}

void Perception::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if(object_ == 1) //ball
  {
    h_min = BALL_HMIN;
    h_max = BALL_HMAX;
  }
  else if(object_ == 2) //blue
  {
    h_min = BLUE_HMIN;
    h_max = BLUE_HMAX;
  }
  else if(object_ == 3) //yellow
  {
    h_min = YELLOW_HMIN;
    h_max = YELLOW_HMAX;
  }

  // Crear copia de la imagen
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // Convertir a HSV:
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

  width_ = cv_ptr->image.cols;
  int height = cv_ptr->image.rows;
  int step = cv_ptr->image.step;
  int channels = 3;

  x = 0;
  y = 0;
  counter = 0;

  for (int i = 0; i < height; i++ ){
    for (int j = 0; j < width_; j++ ){
      int posdata = i * step + j * channels;

      if((hsv.data[posdata] >= h_min) &&
        (hsv.data[posdata] <= h_max) &&
        (hsv.data[posdata + 1] >= S_MIN) &&
        (hsv.data[posdata + 1] <= S_MAX) &&
        (hsv.data[posdata + 2] >= V_MIN) &&
        (hsv.data[posdata + 2] <= V_MAX))
      {
        x += j;
        y += i;
        counter++;
      }
    }
  }

  if(x == 0 && y == 0)
  {
        cmd_.angular.z = 0.3;
        vel_pub_.publish(cmd_);
  }

  if(counter > 0)
  {
    ROS_INFO("x : %d , y: %d \n",x / counter,y / counter);
    ROS_INFO("counter1111: %d \n",counter);
  }

}

int Perception::orient_2object(const int x, const int y)
{ // devuelve 1 si el objeto esta centrado en la imagen

  int centered = 0;

  if(x > width_ / 2 +10)
  {
    ROS_INFO("esta en DER\n");
    cmd_.angular.z = -TURNING_V_4orientation;
  }
  else if (x < width_ / 2 -10)
  {
    ROS_INFO("esta en IZQ\n");
    cmd_.angular.z = TURNING_V_4orientation;

  }
  else
  {
    ROS_INFO("esta en CENT\n");
    cmd_.angular.z = 0;
    centered = 1;
  }
  vel_pub_.publish(cmd_);

  return centered;
}

//crea una transformada estatica desde base_footprint hasta el objeto con coordenadas x,y,z y nombre object
void
Perception::create_transform(const float x, const float y, const int object)
{
  geometry_msgs::TransformStamped odom2bf_msg;
  try{
    odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
  }   catch (std::exception & e)
  {
    ROS_INFO("odom2bf_msg not found");
    return ;
  }

  tf2::Stamped<tf2::Transform> odom2bf;
  tf2::fromMsg(odom2bf_msg, odom2bf);

  tf2::Stamped<tf2::Transform> bf2object;
  bf2object.setOrigin(tf2::Vector3(x * 1.0, y * 1.0, 0));
  bf2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform odom2object = odom2bf * bf2object;

  geometry_msgs::TransformStamped odom2object_msg ;
  odom2object_msg.header.frame_id = "odom";
  odom2object_msg.child_frame_id = "ball";//object;
  odom2object_msg.header.stamp = ros::Time::now();
  odom2object_msg.transform = tf2::toMsg(odom2object);

  br_.sendTransform(odom2object_msg);

  //geometry_msgs::TransformStamped bf2obj_msg;
  //try {
    //  bf2obj_msg = buffer_.lookupTransform( "base_footprint", "object", ros::Time(0));
  //} catch (std::exception & e)
  //{
    //ROS_INFO("bf2obj not found");
    //return;
  //}

  //angulo del robot respecto a la pelota
  //angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
}

void
Perception::step()
{
  //if(!isActive() || object_pub_.getNumSubscribers() == 0){
    //return;
    //ROS_INFO("NOT ACTIVE");
  //}

  distance_ = 0.0;

  if(counter > 0 && width_ > 0)
  {
    ROS_INFO("!!!!counter222: %d \n",counter);
    ROS_INFO("counter>0\n");

    if(orient_2object(x / counter, y / counter) == 1)
    {
      ROS_INFO("ORIENTADO");

      if(counter < 40)
      {
        distance_ = 6.0;
      }
      else if(counter < 55)
      {
        distance_ = 5.0;
      }
      else if(counter < 80)
      {
        distance_ = 4.0;
      }
      else if(counter < 125)
      {
        distance_ = 3.0;
      }
      else if(counter < 500)
      {
        distance_ = 2.0;
      }
      else if(counter < 940)
      {
        distance_ = 1.0;
      }
      printf("Número de píxeles: %d\n", counter);
      ROS_INFO("Object at %d, %d\n", x / counter, y / counter);

    }
    else {
      ROS_INFO("No centrado \n");
    }
  }
  else {
    ROS_INFO("No object found\n");
  }
  ROS_INFO("distance_: %f\n",distance_);

  std_msgs::Float32 msg;
  msg.data = distance_;
  object_pub_.publish(msg);

  //create_transform(distance_, Y_CENTRED, object_);
}

} // practica3
