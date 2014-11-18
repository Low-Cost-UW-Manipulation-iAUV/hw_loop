cmake_minimum_required(VERSION 2.8.3)
project(hw_loop)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	hardware_interface
	tf )
find_package(Boost REQUIRED COMPONENTS thread system)

## Declare a catkin package
catkin_package(
	 CATKIN_DEPENDS 
	 roscpp
	 controller_manager
	 hardware_interface
	 tf
	 INCLUDE_DIRS
	 )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR} )

add_executable(hw_loop src/hw_loop.cpp )
target_link_libraries(hw_loop ${catkin_LIBRARIES}  ${roscpp_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(hw_loop std_msgs_generate_messages_cpp)
