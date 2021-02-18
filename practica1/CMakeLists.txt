cmake_minimum_required(VERSION 3.0.2)
project(practica1)

find_package(Boost REQUIRED COMPONENTS
	roscpp
	std_msgs
	roslint
	sensor_msgs
	kobuki_msgs
	rospy
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)

include directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(nodo_primero src/nodo_primero.cpp)
target_link_libraries(nodo_primero ${catkin_LIBRARIES})

roslint_cpp(
	src/nodo_primero.cpp
)
