cmake_minimum_required(VERSION 3.0.2)
project(opendrone)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
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
  ${EIGEN3_INCLUDE_DIRS}
)
# message(WARNING "INCLUDE_DIRS: ${catkin_LIBRARIES}")
add_executable(offb_node src/basic_example/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(off_node src/basic_example/off_node.cpp)
target_link_libraries(off_node ${catkin_LIBRARIES})

add_executable(off_node_rc src/basic_example/off_node_rc.cpp)
target_link_libraries(off_node_rc ${catkin_LIBRARIES})

add_executable(rect_rc src/basic_example/rect_rc.cpp)
target_link_libraries(rect_rc ${catkin_LIBRARIES})

add_executable(rect src/basic_example/rect.cpp)
target_link_libraries(rect ${catkin_LIBRARIES})

add_executable(rect_other src/basic_example/rect_other.cpp)
target_link_libraries(rect_other ${catkin_LIBRARIES})

add_executable(waypoint src/basic_example/waypoint.cpp)
target_link_libraries(waypoint ${catkin_LIBRARIES})

add_executable(trigger_land src/trigger_land.cpp)
target_link_libraries(trigger_land ${catkin_LIBRARIES})

# add_executable(test1 src/test.cpp)
# target_link_libraries(test1 ${catkin_LIBRARIES})


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
catkin_install_python(PROGRAMS
  scripts/camera_pose_publisher.py
  scripts/trajectory_msg_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


