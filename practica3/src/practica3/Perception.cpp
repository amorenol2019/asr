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
#include <std_msgs/Float64.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace practica3
{
Perception::Perception(): it_(nh_), buffer_() , listener_(buffer_)
{
  image_sub_ = it_.subscribe("/hsv/image_filtered", 10, &Perception::imageCb, this);
  object_sub_ = nh_.subscribe("/object", 10, &Perception::objectCb, this);

  distance_pub_ = nh_.advertise<std_msgs::Float32>("/distance", 10);
  angle_pub_ = nh_.advertise<std_msgs::Float64>("/angle", 10);
  position_pub_ = nh_.advertise<std_msgs::Int64MultiArray>("position",10);

  //vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10); este no  lo necesitamos porque en el perception no nos vamos a mover
}

void Perception::objectCb(const std_msgs::Int64::ConstPtr& msg)
{
  object_ = msg->data;
}

void Perception::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if(object_ == 1) // ball
  {
    name_ = "ball";

    h_min = BALL_HMIN;
    h_max = BALL_HMAX;
    s_min = S_MIN;
    v_min = V_MIN;
  }
  else if(object_ == 2) // blue
  {
    name_ = "blue";

    h_min = BLUE_HMIN;
    h_max = BLUE_HMAX;
    s_min = BLUE_SMIN;
    v_min = V_MIN;
  }
  else if(object_ == 3) // yellow
  {
    name_ = "yellow";

    h_min = YELLOW_HMIN;
    h_max = YELLOW_HMAX;
    s_min = YELLOW_SMIN;
    v_min = YELLOW_VMIN;
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
        (hsv.data[posdata + 1] >= s_min) &&
        (hsv.data[posdata + 1] <= S_MAX) &&
        (hsv.data[posdata + 2] >= v_min) &&
        (hsv.data[posdata + 2] <= V_MAX))
      {
        x += j;
        y += i;
        counter++;
      }
    }
  }
  std_msgs::Int64MultiArray array;
  array.data.clear();
  array.data.push_back(x);
  array.data.push_back(y);
  array.data.push_back(width);

  position_pub_.publish(array); //publica la posicion x,y
}

//crea una transformada estatica desde base_footprint hasta el objeto con coordenadas x,y,z y nombre object
void
Perception::create_transform(const float x, const float y, const std::string name)
{
  geometry_msgs::TransformStamped odom2bf_msg;
  try{
    odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
  } catch (std::exception & e)
  {
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
  odom2object_msg.child_frame_id = name;
  odom2object_msg.header.stamp = ros::Time::now();
  odom2object_msg.transform = tf2::toMsg(odom2object);

  br_.sendTransform(odom2object_msg);
}

void
Perception::look4_TF(const std::string name)
{
  float angle;

  geometry_msgs::TransformStamped bf2obj_msg;
  try {
      bf2obj_msg = buffer_.lookupTransform("base_footprint", name, ros::Time(0));
  }
  catch (std::exception & e)
  {
    ??; //si no se encuantran transformadas se sale de la funcion con una velocidad arbitraria
  }

  //angulo del robot respecto a la pelota
  angle = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);

  std_msgs::Float64 msg2;
  msg2.data = angle;
  angle_pub_.publish(msg2);
}

void
Perception::step()
{
  if(!isActive() || object_pub_.getNumSubscribers() == 0){
    return;
  }

  distance_ = 0.0;

  if (counter == 0)
  {
    look4_TF(name_);
  }

  else //aqui no se que poner porque realmente no se si se deberia calcular aqui la distancia y el orient2object esta en el forward
  {
    if(orient_2object(x / counter, y / counter) == 1)
    {
      if(object_ == 1)
      {
        distance_ = 10.52 - 1.44 * logf(counter);
      }
      else if(object_ == 2 || object_ == 3)
      {
        distance_ = 16.09 - 1.45 * logf(counter);
      }
    }
  }

  std_msgs::Float32 msg;
  msg.data = distance_;
  object_pub_.publish(msg);

  create_transform(distance_, 0, name_);
}

} // practica3
