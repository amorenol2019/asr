cmake_minimum_required(VERSION 3.0.2)
project(practica1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	kobuki_msgs
	geometry_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(nodo_primero src/nodo_primero.cpp)
target_link_libraries(nodo_primero ${catkin_LIBRARIES})
