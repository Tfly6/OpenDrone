cmake_minimum_required(VERSION 2.8.3)
project(ego_planner_v2)

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
  # traj_opt
  # traj_utils
  message_generation
  message_filters
  cv_bridge
  visualization_msgs
  sensor_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner_v2
 CATKIN_DEPENDS  message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  src
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# add_library( path_searching 
#     src/dyn_a_star.cpp
#     )
# target_link_libraries( path_searching
#     ${catkin_LIBRARIES} 
#     )  
# add_library( plan_env 
#     src/grid_map.cpp 
#     src/raycast.cpp
#     )

# target_link_libraries( plan_env
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ) 

# add_library( traj_opt 
#     src/poly_traj_optimizer.cpp 
#     )
# target_link_libraries( traj_opt
#     ${catkin_LIBRARIES} 
#     )  

# add_library( traj_utils
#     src/planning_visualization.cpp 
#     )
# target_link_libraries( traj_utils
#     ${catkin_LIBRARIES} 
#     ) 
set(SOURCES
  src/plan_manage/ego_planner_node.cpp
  src/plan_manage/ego_replan_fsm.cpp
  src/plan_manage/planner_manager.cpp
  src/path_searching/dyn_a_star.cpp
  src/plan_env/grid_map.cpp
  src/plan_env/raycast.cpp
  src/traj_opt/poly_traj_optimizer.cpp
  src/traj_utils/planning_visualization.cpp
)
add_executable(ego_planner_node_v2 ${SOURCES})
target_link_libraries(ego_planner_node_v2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
#add_dependencies(ego_planner_node_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server_v2 src/plan_manage/traj_server.cpp)
target_link_libraries(traj_server_v2 ${catkin_LIBRARIES})
# add_dependencies(traj_server_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS})

set(ENABLE_TCP false) # requires zmq, zmqpp 
add_executable(bridge_node_udp
  src/swarm_bridge/bridge_node_udp.cpp 
  )
target_link_libraries(bridge_node_udp 
  ${catkin_LIBRARIES}
  )

if(ENABLE_TCP)

  add_executable(bridge_node_tcp
    src/swarm_bridge/bridge_node_tcp.cpp 
    )
  target_link_libraries(bridge_node_tcp
    ${catkin_LIBRARIES}
    zmq
    zmqpp
    )

endif(ENABLE_TCP)

add_executable(traj2odom_node
  src/swarm_bridge/traj2odom_node.cpp 
  )
target_link_libraries(traj2odom_node 
  ${catkin_LIBRARIES}
  )

