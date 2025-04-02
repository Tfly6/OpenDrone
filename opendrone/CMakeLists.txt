cmake_minimum_required(VERSION 3.0.2)
project(opendrone)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  controller_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opendrone
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(off_node src/off_node.cpp)
target_link_libraries(off_node ${catkin_LIBRARIES})

add_executable(off_node_rc src/off_node_rc.cpp)
target_link_libraries(off_node_rc ${catkin_LIBRARIES})

add_executable(rect_rc src/rect_rc.cpp)
target_link_libraries(rect_rc ${catkin_LIBRARIES})

add_executable(rect src/rect.cpp)
target_link_libraries(rect ${catkin_LIBRARIES})

add_executable(rect_other src/rect_other.cpp)
target_link_libraries(rect_other ${catkin_LIBRARIES})

add_executable(test1 src/test.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})


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


