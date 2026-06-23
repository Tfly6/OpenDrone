cmake_minimum_required(VERSION 3.5)
project(super_planner_od)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

message("Using ROS1")
# add a marco for ros1
add_definitions(-DUSE_ROS1)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        rog_map
        tf
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(
        INCLUDE_DIRS
        LIBRARIES super_planner_od
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        #        super_planner
        -ldw
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super STATIC ${srcs})
target_include_directories(super PUBLIC
        include)
target_link_libraries(super PUBLIC
)


add_executable(fsm_node
        Apps/fsm_node_ros1.cpp
)
target_link_libraries(fsm_node
        super
        ${THIRD_PARTY}
)

#add_executable(read_replan_log
#        Apps/read_replan_log.cpp
#)
#target_link_libraries(read_replan_log
#        super
#        -lncurses
#        ${THIRD_PARTY}
#)
#
#
#add_executable(traj_opt_tuning
#        Apps/traj_opt_tuning.cpp
#)
#target_link_libraries(traj_opt_tuning
#        super
#        -lncurses
#        ${THIRD_PARTY}
#)
