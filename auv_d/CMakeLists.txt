cmake_minimum_required(VERSION 2.8.3)
project(auv_d)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY config meshes launch src urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(auv_d_key_control src/auv_d_key_control.cpp)
target_link_libraries(auv_d_key_control ${catkin_LIBRARIES})
