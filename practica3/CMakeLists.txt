cmake_minimum_required(VERSION 3.0.2)
project(practica3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  bica
  tf2
  roscpp
  image_transport
  cv_bridge
  std_msgs
  roslint
  kobuki_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp bica std_msgs tf2 tf2_ros kobuki_msgs image_transport cv_bridge tf2_geometry_msgs geometry_msgs

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/Perception.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(perception_node src/perception_node.cpp src/practica3/Perception.cpp)
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_node ${catkin_LIBRARIES})
