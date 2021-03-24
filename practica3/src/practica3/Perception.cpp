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
//#include "std_msgs/Bool"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

Perception::Perception(): it_(nh_), buffer_() , listener_(buffer_)
{
  image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &Perception::imageCb, this);
  object_sub_ = nh_.subscribe("/object", 1, &Perception::objectCb, this);
}

void Perception::objectCb(const std_msgs::Float32::ConstPtr& msg)
{
  object_ = "ball";
  //msg;
}

void Perception::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if(object_ == "ball")
  {
    h_min = BALL_HMIN;
    h_max = BALL_HMAX;
  }
  else if(object_ == "blue")
  {
    h_min = BLUE_HMIN;
    h_max = BLUE_HMAX;
  }
  else if(object_ == "yellow")
  {
    h_min = YELLOW_HMIN;
    h_max = YELLOW_HMAX;
  }

  // Crear copias de la imagen
  cv_bridge::CvImagePtr cv_ptr; //, cv_imageout;

  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // cv_imageout = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // Convertir a HSV:
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

  width_ = cv_ptr->image.cols;
  int height = cv_ptr->image.rows;
  int step = cv_ptr->image.step;
  int channels = 3;  // RGB

  int x = 0;
  int y = 0;
  int counter = 0;

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

  if(counter > 0)
  {
    if(orient_2object(x / counter, y / counter) == 1)
    {
      if(counter < 40)
      {
        distance_ = 6;
      }
      else if(counter < 55)
      {
        distance_ = 5;
      }
      else if(counter < 80)
      {
        distance_ = 4;
	    }
      else if(counter < 125)
      {
        distance_ = 3;
      }
      else if(counter < 500)
      {
        distance_ = 2;
      }
      else if(counter < 940)
      {
        distance_ = 1;
      }

      printf("Número de píxeles: %d\n", counter);
      ROS_INFO("Object at %d %d", x / counter, y / counter);
    }
    else {
      ROS_INFO("No centrado");
    }
  }
  else {
    ROS_INFO("No object found");
  }

}

int Perception::orient_2object(const int x ,const int y) //devuelve true si el objeto esta centrado en x
{
  int centered = 0;

  geometry_msgs::Twist cmd;

  if( x > width_ / 2 )
    { cmd.angular.z = - TURNING_V; }    //gira hacia la derecha
  else if (x < width_ / 2)
    { cmd.angular.z = TURNING_V; }
  else
    {
      cmd.angular.z = 0;
      centered = 1;
    }

  return centered;
}

//crea una transformada estatica desde base_footprint hasta el objeto con coordenadas x,y,z y nombre object
void Perception::create_transform(float x, float y ,std::string object)
{
  geometry_msgs::TransformStamped odom2bf_msg;
  odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));

  tf2::Stamped<tf2::Transform> odom2bf;
  tf2::fromMsg(odom2bf_msg, odom2bf);

  tf2::Stamped<tf2::Transform> bf2object;
  bf2object.setOrigin(tf2::Vector3(x*1.0, y*1.0, 0));
  bf2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform odom2object = odom2bf * bf2object;

  geometry_msgs::TransformStamped odom2object_msg ;
  odom2object_msg.header.frame_id = "odom";
  odom2object_msg.child_frame_id = object;
  odom2object_msg.header.stamp = ros::Time::now();
  odom2object_msg.transform = tf2::toMsg(odom2object);

  br_.sendTransform(odom2object_msg);
}

void
Perception::step()
{
  if(!isActive()) {
    return;
  }

  create_transform(distance_, 0, object_);
}
