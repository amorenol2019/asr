#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace practica5
{
  class Go_object : public BT::ActionNodeBase
  {
  public:
    Go_object(const std::string& name, const BT::NodeConfiguration& config);

    void detectCB(const std_msgs::Bool::ConstPtr& msg);

    void halt();
    BT::NodeStatus tick();
    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("target")};
    }

  private:
    void centre_2object();
    void look4_TF(const std::string name);
    float speed_4angle(float angle);

    ros::NodeHandle nh_;
    geometry_msgs::Twist vel;
    ros::Publisher vel_pub_;
    ros::Subscriber detect_sub_;
    ros::Time seen_;

    bool arrived_;
    bool detected_ ;
    bool already_seen;

    std::string object_ = "none";

    float ANGULAR_VEL = 0.2;
    float LINEAR_VEL = 0.2;
    float angle_;
    float distance_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener  listener_;
  };
} // namespace practica5
