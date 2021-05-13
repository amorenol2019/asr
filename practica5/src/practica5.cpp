#include "practica5/Go_point.hpp"
#include "practica5/Turn.hpp"
#include "practica5/Go_object.hpp"

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    std::cerr << "usage: rosrun practica5_1 nodo_rgbd <destination> <object>" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "practica5");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<practica5::Go_point>("Go_point");
  //factory.registerNodeType<practica5::Turn>("Turn");
  factory.registerNodeType<practica5::Go_object>("Go_object");

  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::string>("destination", argv[1]);
  blackboard->set<std::string>("object", argv[2]);

  std::string pkgpath = ros::package::getPath("practica5");
  std::string xml_file = pkgpath + "/behavior_trees_xml/tree_3.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  ros::Rate loop_rate(5);
  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
