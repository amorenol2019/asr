cmake_minimum_required(VERSION 3.0.2)
project(practica4)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  move_base_msgs
  actionlib
	bica
	std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp  move_base_msgs actionlib bica std_msgs
)

include_directories(
  include
	fsm
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/Navigate.cpp
	 src/${PROJECT_NAME}/sequenceImpl.cpp
	 fsm/sequence.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(nodo_destino src/nodo_destino.cpp  src/practica4/Navigate.cpp)
add_dependencies(nodo_destino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodo_destino ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(nodo_sequence src/nodo_sequence.cpp  src/practica4/Navigate.cpp)
add_dependencies(nodo_sequence ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodo_sequence ${PROJECT_NAME} ${catkin_LIBRARIES})
