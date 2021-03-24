cmake_minimum_required(VERSION 3.0.2)
project(practica3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bica
  image_transport
  cv_bridge
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

find_package (OpenCV)

catkin_package(
  #CATKIN_DEPENDS bica roscpp std_msgs kobuki_msgs sensor_msgs image_transport cv_bridge
)

include_directories(
  include
  fsm
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/Turn.cpp
   src/${PROJECT_NAME}/Perception.cpp
   src/${PROJECT_NAME}/Forward.cpp
   src/${PROJECT_NAME}/Practica3Impl.cpp
   fsm/practica3.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(forward_node src/forward_node.cpp)
add_dependencies(forward_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(forward_node ${catkin_LIBRARIES})

add_executable(turn_node src/turn_node.cpp)
add_dependencies(turn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_node ${catkin_LIBRARIES})

add_executable(perception_node src/perception_node.cpp)
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_node ${catkin_LIBRARIES})

add_executable(practica3_node src/practica3_node.cpp)
add_dependencies(practica3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(practica3_node ${catkin_LIBRARIES})


#roslint_cpp(
#  src/turn_node.cpp
#  src/perception_node.cpp
#  src/forward_node.cpp
#  src/practica3_node.cpp
#  include/practica3/Practica3Impl.hpp src/practica3/Practica3Impl.cpp
#  include/practica3/Turn.hpp src/practica3/Turn.cpp
#  include/practica3/Perception.hpp src/practica3/Perception.cpp
#  include/practica3/Forward.hpp src/practica3/Forward.cpp
#)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_practica3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
