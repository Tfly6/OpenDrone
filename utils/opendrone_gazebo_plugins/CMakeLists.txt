cmake_minimum_required(VERSION 3.0.2)
project(opendrone_gazebo_plugins)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  ContactPulse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES opendrone_contact_plugin
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS GAZEBO
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(opendrone_contact_plugin
  src/contact_plugin.cpp
)
add_dependencies(opendrone_contact_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(opendrone_contact_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS opendrone_contact_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
