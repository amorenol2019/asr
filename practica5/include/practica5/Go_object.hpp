#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace practica5
{
  class Go_object : public BT::ActionNodeBase
  {
  public:
    Go_object(const std::string& name, const BT::NodeConfiguration& config);

    void centre_2object();
    void look4_TF(const std::string name);

    void halt();
    BT::NodeStatus tick();
    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("object")};
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::Twist vel;
    ros::Publisher vel_pub_;

    std::string object_ = "none";
    bool arrived_;

    float angle_;
    float distance_;
    tf2_ros::Buffer buffer_;

    float ANGULAR_VEL = 0.1;
    float LINEAR_VEL = 0.2;

  };
}//namespace practica5
