cmake_minimum_required(VERSION 2.8.3)
project(tf_republisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES tf_republisher
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf
 # DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

SET(exename "tf_to_pose")
add_executable(${exename} src/tf_to_pose.cpp)
target_link_libraries(${exename} ${catkin_LIBRARIES})
add_dependencies(${exename} ${catkin_EXPORTED_TARGETS})
