cmake_minimum_required(VERSION 3.0.2)
project(practica4)

find_package(catkin REQUIRED COMPONENTS
  kobuki_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS kobuki_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp(
  src/nodo_destino.cpp
  include/practica4/Navigate.hpp
)


add_executable(nodo_destino src/nodo_destino.cpp)
target_link_libraries(nodo_destino ${catkin_LIBRARIES})
