cmake_minimum_required(VERSION 2.8.3)
project(lfd_controller)
add_compile_options(-std=c++17)

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS 
	roscpp 
	tf 
	std_msgs
	message_generation
	testbed_msgs
	utilities
	roslib
	)

find_package (Eigen3 REQUIRED NO_MODULE)

add_message_files(
	DIRECTORY msg 
	FILES
	PerformanceMsg.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	)


catkin_package(
	INCLUDE_DIRS include/ # The package contains header files
	CATKIN_DEPENDS roscpp tf std_msgs
	message_runtime
	)

# Since we use eigen
include_directories(
	include/
	${catkin_INCLUDE_DIRS} # Found in "find_package"
	)


# Declare the executable to be build and the associated source
add_executable(lfd_controller 
	# src/gurobiRoutines.cpp
	exec/lfd_controller_node.cpp 
	src/lfd_controller.cpp
	src/lfd_controller_ros.cpp)

add_dependencies(lfd_controller 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(lfd_controller
	#	${GUROBI_LIBRARY}
	${catkin_LIBRARIES}
	utilities
	Eigen3::Eigen 
	)
