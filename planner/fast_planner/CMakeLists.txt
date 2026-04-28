cmake_minimum_required(VERSION 2.8.3)
project(fast_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(NLOPT_INCLUDE_DIR "/usr/local/include")  
set(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  message_generation
  cv_bridge
  visualization_msgs
  message_filters
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_planner
 CATKIN_DEPENDS message_runtime visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}  
)
# --bspline
# add_library( bspline 
#     src/bspline/non_uniform_bspline.cpp 
#     )
# target_link_libraries( bspline
#     ${catkin_LIBRARIES} 
#     )  
# --bspline_opt
# add_library( bspline_opt 
#     src/bspline_opt/bspline_optimizer.cpp 
#     )
# target_link_libraries( bspline_opt
#     ${catkin_LIBRARIES} 
#     ${NLOPT_LIBRARY}   
#     )
# --path searching
# add_library( path_searching 
#     src/path_searching/kinodynamic_astar.cpp 
#     src/path_searching/astar.cpp
#     src/path_searching/topo_prm.cpp
#     )
# target_link_libraries( path_searching
#     ${catkin_LIBRARIES} 
#     )  
# # --plan_env
# add_library( plan_env 
#     src/plan_env/sdf_map.cpp 
#     src/plan_env/obj_predictor.cpp 
#     src/plan_env/edt_environment.cpp 
#     src/plan_env/raycast.cpp
#     )
# target_link_libraries( plan_env
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     )  

add_executable(obj_generator
    src/plan_env/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})

# --poly_traj
# add_library( poly_traj 
#     src/poly_traj/polynomial_traj.cpp 
#     )
# target_link_libraries( poly_traj
#     ${catkin_LIBRARIES} 
#     )
    
# add_library( traj_utils
#     src/traj_utils/planning_visualization.cpp 
#     )
# target_link_libraries( traj_utils
#     ${catkin_LIBRARIES} 
#     )  

set(SOURCES
  src/plan_manage/fast_planner_node.cpp 
  src/plan_manage/kino_replan_fsm.cpp
  src/plan_manage/topo_replan_fsm.cpp
  src/plan_manage/planner_manager.cpp
  src/plan_env/sdf_map.cpp 
  src/plan_env/obj_predictor.cpp 
  src/plan_env/edt_environment.cpp 
  src/plan_env/raycast.cpp
  src/bspline/non_uniform_bspline.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/path_searching/kinodynamic_astar.cpp 
  src/path_searching/astar.cpp
  src/path_searching/topo_prm.cpp
  src/poly_traj/polynomial_traj.cpp 
  src/traj_utils/planning_visualization.cpp  
)

add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NLOPT_LIBRARY}
)

add_executable(${PROJECT_NAME}_traj_server 
  src/plan_manage/traj_server.cpp
  src/bspline/non_uniform_bspline.cpp
)
target_link_libraries(${PROJECT_NAME}_traj_server ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


