cmake_minimum_required(VERSION 2.8.12)
project(se3_lee)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
)

generate_dynamic_reconfigure_options(
  cfg/GeometricController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES se3_lee
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/se3_lee.cpp 
  src/nonlinear_attitude_control.cpp
  src/nonlinear_geometric_control.cpp
  src/jerk_tracking_control.cpp
)
add_dependencies(se3_lee ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(se3_lee_node
  src/se3_lee_node.cpp
)
add_dependencies(se3_lee_node se3_lee ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(se3_lee_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
# cs_install()
# cs_export()
#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#     # Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
#                                           test/test_example.cpp
#                                           test/se3_lee-test.cpp)

#     if(TARGET ${PROJECT_NAME}-test)
#         target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()

# endif()
