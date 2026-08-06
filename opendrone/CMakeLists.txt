cmake_minimum_required(VERSION 3.0.2)
project(opendrone)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  mavros_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
)

add_message_files(
  FILES
  PlannerOutput.msg
  PlannerOutputPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES opendrone
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs trajectory_msgs message_runtime
#  DEPENDS system_lib
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime roscpp rospy sensor_msgs std_msgs std_srvs trajectory_msgs tf2_ros tf2_sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
# message(WARNING "INCLUDE_DIRS: ${catkin_LIBRARIES}")
add_executable(offb_node src/basic_example/offb_node.cpp)
add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(off_node src/basic_example/off_node.cpp)
add_dependencies(off_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(off_node ${catkin_LIBRARIES})

add_executable(off_node_rc src/basic_example/off_node_rc.cpp)
add_dependencies(off_node_rc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(off_node_rc ${catkin_LIBRARIES})

add_executable(rect_rc src/basic_example/rect_rc.cpp)
add_dependencies(rect_rc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rect_rc ${catkin_LIBRARIES})

add_executable(rect src/basic_example/rect.cpp)
add_dependencies(rect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rect ${catkin_LIBRARIES})

add_executable(rect_with_yaw src/basic_example/rect_with_yaw.cpp)
add_dependencies(rect_with_yaw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rect_with_yaw ${catkin_LIBRARIES})

add_executable(rect_other src/basic_example/rect_other.cpp)
add_dependencies(rect_other ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rect_other ${catkin_LIBRARIES})

add_executable(waypoint src/basic_example/waypoint.cpp)
add_dependencies(waypoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint ${catkin_LIBRARIES})

add_executable(base src/base.cpp)
add_dependencies(base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base ${catkin_LIBRARIES})

add_executable(trigger_land src/trigger_land.cpp)
add_dependencies(trigger_land ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trigger_land ${catkin_LIBRARIES})

add_executable(depth_camera_processor src/depth_camera_processor.cpp)
add_dependencies(depth_camera_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_camera_processor ${catkin_LIBRARIES})

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
  scripts/analytic_reference_publisher.py
  scripts/camera_pose_publisher.py
  scripts/planner_adapter.py
  scripts/pointcloud_to_world.py
  # scripts/trajectory_msg_converter.py
  # scripts/waypoint_msg_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
