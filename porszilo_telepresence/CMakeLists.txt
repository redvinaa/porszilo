cmake_minimum_required(VERSION 2.8.3)
project(porszilo_telepresence)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
	cv_bridge
	std_srvs
	message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
	FILES
	CanvasSize.msg
)

add_service_files(
	FILES
	ClickedPoint.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}
  # LIBRARIES porszilo_telepresence
  CATKIN_DEPENDS
		geometry_msgs
		nav_msgs
		roscpp rospy
		sensor_msgs
		actionlib_msgs
		opencv2
		cv_bridge
		std_srvs
		message_runtime
  # DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
