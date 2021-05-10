cmake_minimum_required(VERSION 3.0.2)
project(practica5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros
  behaviortree_cpp_v3
)

catkin_package(
CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros
  behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(RGBDFilter src/practica5/RGBDFilter.cpp)
target_link_libraries(RGBDFilter ${catkin_LIBRARIES})

add_executable(Go_point src/practica5/Go_point.cpp)
target_link_libraries(Go_point ${catkin_LIBRARIES})

add_executable(practica5 src/practica5.cpp)
target_link_libraries(practica5 ${catkin_LIBRARIES})
