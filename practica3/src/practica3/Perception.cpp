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
#include <std_msgs/Float32MultiArray.h>

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
  state_sub_ = nh_.subscribe("/practica3/state", 10, &Perception::stateCb, this);

  distance_pub_ = nh_.advertise<std_msgs::Float64>("/distance", 10);
  angle_pub_ = nh_.advertise<std_msgs::Float64>("/angle", 10);
  position_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/position",10);
}

void Perception::stateCb(const std_msgs::String::ConstPtr& msg)
{
  state_ = msg->data;
}

void Perception::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if(state_.compare("ToBall"))
  {
    name_ = "ball";

    h_min = BALL_HMIN;
    h_max = BALL_HMAX;
    s_min = S_MIN;
    v_min = V_MIN;
  }
  else if(state_.compare("ToBlueGoal"))
  {
    name_ = "blue";

    h_min = BLUE_HMIN;
    h_max = BLUE_HMAX;
    s_min = BLUE_SMIN;
    v_min = V_MIN;
  }
  else if(state_.compare("ToYellGoal"))
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

  x_ = 0;
  y_ = 0;
  counter_ = 0;

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
        x_ += j;
        y_ += i;
        counter_++;
      }
    }
  }
  std_msgs::Float32MultiArray array;
  array.data.clear();
  array.data.push_back(x_);
  array.data.push_back(y_);
  array.data.push_back(width_);

  position_pub_.publish(array); //publica la posicion x,y
}

//crea una transformada estatica desde base_footprint hasta el objeto con coordenadas x,y,z
void
Perception::create_transform(const float x, const float y, const std::string name)
{
  geometry_msgs::TransformStamped odom2bf_msg;
  try{
    odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
  } catch (std::exception & e)
  {
    return;
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
  geometry_msgs::TransformStamped bf2obj_msg;
  try {
      bf2obj_msg = buffer_.lookupTransform("base_footprint", name, ros::Time(0));
  }
  catch (std::exception & e)
  {
    ROS_INFO("No se ha encontrado transformada"); //si no se encuantran transformadas se sale de la funcion con una velocidad arbitraria
    return;
  }

  tf_founded_ = true;
  //angulo del robot respecto a la pelota
  angle_ = atan2(bf2obj_msg.transform.translation.y, bf2obj_msg.transform.translation.x);
}

void
Perception::step()
{
  if(!isActive() || distance_pub_.getNumSubscribers() == 0 || angle_pub_.getNumSubscribers() == 0){
    return;
  }

  distance_ = 0.0;
  tf_founded_ = false;

  if(counter_ == 0)
  {
    look4_TF(name_); // da valor a angle si encuentra la transformada
    if(tf_founded_ == false)
    {
      angle_ = -1; // valor aleatorio
    }
  }
  else // calcula distancia con el número de pixeles
  {
    if(state_.compare("To Ball"))
    {
      distance_ = 10.52 - 1.44 * logf(counter_);
    }
    else if(state_.compare("ToBlueGoal") || state_.compare("ToYellGoal"))
    {
      distance_ = 16.09 - 1.45 * logf(counter_);
    }
  }

  if(distance_ != 0 && distance_ < 0.2)
  {
    create_transform(distance_, 0, name_);
  }

  std_msgs::Float64 msg, msg2;
  msg.data = angle_;
  angle_pub_.publish(msg);

  msg2.data = distance_;
  distance_pub_.publish(msg2);

}

} // practica3
