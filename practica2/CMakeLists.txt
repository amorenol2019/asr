cmake_minimum_required(VERSION 3.0.2)
project(practica2)

find_package(catkin REQUIRED COMPONENTS
  practica2 //es necesrio?
  roscpp
	geometry_msgs
  sensor_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(bumpgo_laser.cpp
src/bumpgo_laser.cpp
src/practica2/detetor.cpp)


target_link_libraries(bumpgo_laser ${catkin_LIBRARIES})

roslint_cpp(
  src/nodo_behavior_trees.cpp
  include/practica2/Detector.h
)
