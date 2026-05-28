cmake_minimum_required(VERSION 3.0.2)
project(rpg_polynomial_trajectory)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpg_polynomial_trajectory
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



cs_add_library(${PROJECT_NAME} src/polynomial_trajectory.cpp
    src/polynomial_trajectories_common.cpp 
    src/minimum_snap_trajectories.cpp 
    src/constrained_polynomial_trajectories.cpp
    src/polynomial_trajectory_helper.cpp)

cs_add_executable(rpg_traj_node src/rpg_traj_node.cpp)
target_link_libraries(rpg_traj_node ${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export()