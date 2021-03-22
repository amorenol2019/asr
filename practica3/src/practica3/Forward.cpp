#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "Forward.hpp"

namespace practica3
{
  Forward::Forward()
  {
    vel_pub_ = pub_detect_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &Forward::detect, this);
  }

  void Forward::detect(const sensor_msgs::Image::ConstPtr& msg)
  {
    cv_bridge::CvImagePtr copy, result;
    // Copiamos en 'copy' la imagen que se recibe
    copy = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    int height = copy->image.rows;
    int width = copy->image.cols;
    int step = copy->image.step;
    int channels = 3;  // RGB

    // Contamos filas y columnas != de negro
    int x = 0;
    int y = 0;
    int counter = 0;

    for (int i = 0; i < height; i++ ){
      for (int j = 0; j < width; j++ ){
        int posdata = i * step + j * channels;

        if(copy->image.data[posdata] != 0 && copy->image.data[posdata + 1] != 0 && copy->image.data[posdata + 2] != 0){
          x += j;
          y += i;
          counter++;
        }
      }
    }

    // Bola:
    if(counter > 0){
          ROS_INFO("Ball at %d %d", x / counter, y / counter);
          //create_transform(x,y,z,"ball")
    } else{
      ROS_INFO("No ball found");
    }

    // Porterías:
    if(counter > 0){
          ROS_INFO("Goal at %d %d", x / 2, y / 2);
    } else{
      ROS_INFO("No goal found");
    }

  }

//crea una transformada estatica desde base_footprint hasta el objeto con coordenadas x,y,z y nombre object
  void create_transform(int x, int y ,int z,string object)
  {
    tf2_ros::StaticTransformBroadcaster br; //queremos que las transformadas sean estaticas

    tf2::Stamped<tf2::Transform> bf2object;
    bf2object.frame_id_ = "base_footprint"; //las transformadas iran desde el robot hacia los objetos
    bf2object.stamp_ = ros::Time::now();

    bf2object.setOrigin(tf2::Vector3(x, y, z));

    tf2::Quaternion q;
    q.setRPY(ax, ay, az);
    bf2object.setRotation(q);

    geometry_msgs::TransformStamped bf2object_msg = tf2::toMsg(bf2object);
    bf2object_msg.child_frame_id = object ; //habra que sustituir object dependiendo de cada objeto a detectar
    br.sendTransform(bf2object_msg);

  }


  void Forward::step()
  {
    geometry_msgs::Twist vel;

    vel.linear.x = ;

    vel_pub_.publish(vel);
  }
}
