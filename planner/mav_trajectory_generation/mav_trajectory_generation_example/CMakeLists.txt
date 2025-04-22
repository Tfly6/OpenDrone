cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation_example)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake/Modules)
find_package(Glog REQUIRED)
find_package(NLopt REQUIRED)
############
# BINARIES #
############
cs_add_executable(trajectory_generation_example
        src/example_planner.cc
        src/example_planner_node.cc
)
target_link_libraries(trajectory_generation_example ${GLOG_LIBRARIES} ${NLOPT_LIBRARIES})

cs_add_executable(trajectory_generation_6dof_example
        src/example_planner_6dof.cc
        src/example_planner_6dof_node.cc
)
target_link_libraries(trajectory_generation_6dof_example ${GLOG_LIBRARIES} ${NLOPT_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()
