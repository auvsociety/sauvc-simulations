add_executable(sauvc_tasks all_tasks.cpp ${UWV_DEPENDENCIES}) 
target_link_libraries(sauvc_tasks ${catkin_LIBRARIES})
