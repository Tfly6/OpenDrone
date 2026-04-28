cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  # plan_env
  # path_searching
  # bspline_opt
  # traj_utils
  message_generation
  cv_bridge
  visualization_msgs
  message_filters
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs quadrotor_msgs message_runtime cv_bridge
 DEPENDS OpenCV
)

include_directories(
  include
  SYSTEM
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# add_library( bspline_opt 
#     src/bspline_opt/uniform_bspline.cpp 
#     src/bspline_opt/bspline_optimizer.cpp 
#     src/bspline_opt/gradient_descent_optimizer.cpp
#     )
# target_link_libraries( bspline_opt
#     ${catkin_LIBRARIES} 
#     ) 

# add_library( path_searching 
#     src/path_searching/dyn_a_star.cpp
#     )
# target_link_libraries( path_searching
#     ${catkin_LIBRARIES} 
#     )
    
# add_library( plan_env 
#     src/plan_env/grid_map.cpp 
#     src/plan_env/raycast.cpp
#     src/plan_env/obj_predictor.cpp 
#     )
# target_link_libraries( plan_env
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
#     )  

add_executable(obj_generator
    src/plan_env/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    )

# add_library( traj_utils
#     src/traj_utils/planning_visualization.cpp 
#     src/traj_utils/polynomial_traj.cpp 
#     )
# target_link_libraries( traj_utils
#     ${catkin_LIBRARIES} 
#     ) 

set(SOURCES
  src/plan_manage/ego_planner_node.cpp 
  src/plan_manage/ego_replan_fsm.cpp
  src/plan_manage/planner_manager.cpp
  src/bspline_opt/uniform_bspline.cpp 
  src/bspline_opt/bspline_optimizer.cpp 
  src/bspline_opt/gradient_descent_optimizer.cpp
  src/path_searching/dyn_a_star.cpp
  src/plan_env/grid_map.cpp 
  src/plan_env/raycast.cpp
  src/plan_env/obj_predictor.cpp 
  src/traj_utils/planning_visualization.cpp 
  src/traj_utils/polynomial_traj.cpp
)
add_executable(ego_planner_node ${SOURCES})
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/plan_manage/traj_server.cpp src/bspline_opt/uniform_bspline.cpp )
target_link_libraries(traj_server ${catkin_LIBRARIES})
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(bridge_node
  src/rosmsg_tcp_bridge/bridge_node.cpp 
  )
target_link_libraries(bridge_node 
  ${catkin_LIBRARIES}
  )

