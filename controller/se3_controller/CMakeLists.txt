cmake_minimum_required(VERSION 3.0.2)
project(se3_controller)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  math_utils
  # airsim_ros_pkgs
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/tune.cfg
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES se3_controller
#  CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs roscpp rospy std_msgs tf airsim_ros_pkgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(se3_controller_node
  src/se3_controller_node.cpp
  src/se3_ctrl.cpp
)

target_link_libraries(se3_controller_node
  ${catkin_LIBRARIES}
)

# add_executable(${PROJECT_NAME}_example_airsim_node 
#   src/se3_example_airsim.cpp
# )

# target_link_libraries(${PROJECT_NAME}_example_airsim_node
#   ${catkin_LIBRARIES}
# )

# add_executable(${PROJECT_NAME}_example_node 
#   src/se3_example.cpp
# )

# target_link_libraries(${PROJECT_NAME}_example_node
#   ${catkin_LIBRARIES}
# )