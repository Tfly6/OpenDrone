cmake_minimum_required(VERSION 2.8.3)
project(airfar_reference_bridge)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  opendrone
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES airfar_path_scale_schedule airfar_planner_reference
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  opendrone
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_library(airfar_path_scale_schedule src/path_scale_schedule.cpp)
add_library(airfar_planner_reference src/planner_reference.cpp)
add_executable(airfar_local_planner src/localPlanner.cpp)
add_executable(airfar_path_follower src/pathFollower.cpp)
add_dependencies(airfar_path_scale_schedule ${catkin_EXPORTED_TARGETS})
add_dependencies(airfar_planner_reference ${catkin_EXPORTED_TARGETS})
add_dependencies(airfar_local_planner ${catkin_EXPORTED_TARGETS})
add_dependencies(airfar_path_follower ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(airfar_local_planner
  airfar_path_scale_schedule
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(airfar_path_follower
  airfar_planner_reference
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(airfar_planner_reference_test
    tests/planner_reference_test.cpp
  )
  if(TARGET airfar_planner_reference_test)
    target_link_libraries(airfar_planner_reference_test
      airfar_planner_reference
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(airfar_path_scale_schedule_test
    tests/path_scale_schedule_test.cpp
  )
  if(TARGET airfar_path_scale_schedule_test)
    target_link_libraries(airfar_path_scale_schedule_test
      airfar_path_scale_schedule
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS
  airfar_path_scale_schedule
  airfar_planner_reference
  airfar_local_planner
  airfar_path_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
