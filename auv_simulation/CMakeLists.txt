cmake_minimum_required(VERSION 2.8.3)
project(auv_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY config meshes launch src urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
