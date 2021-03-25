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
  fsm
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/Perception.cpp
   src/${PROJECT_NAME}/Forward.cpp
   src/${PROJECT_NAME}/Turn.cpp
   src/${PROJECT_NAME}/Practica3Impl.cpp
   fsm/practica3.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(perception_node src/perception_node.cpp src/practica3/Perception.cpp)
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_node ${catkin_LIBRARIES})

add_executable(forward_node src/forward_node.cpp src/practica3/Forward.cpp)
add_dependencies(forward_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(forward_node ${catkin_LIBRARIES})

add_executable(turn_node src/turn_node.cpp src/practica3/Turn.cpp)
add_dependencies(turn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_node ${catkin_LIBRARIES})

add_executable(practica3_node src/practica3_node.cpp src/practica3/Practica3Impl.cpp)
add_dependencies(practica3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(practica3_node ${catkin_LIBRARIES})
