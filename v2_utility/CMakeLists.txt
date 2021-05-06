cmake_minimum_required(VERSION 3.0.2)
project(v2_utility)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
				roscpp
				rospy
				std_msgs
				sensor_msgs
				geometry_msgs
				)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

find_package(gazebo REQUIRED)

include_directories(
	include
	${GAZEBO_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/plugin_depth_sensor)
add_subdirectory(src/loc_sensors)