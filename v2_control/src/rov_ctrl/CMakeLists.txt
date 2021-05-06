add_executable(rov_ctrl rov_ctrl.cpp
						${UWV_DEPENDENCIES}
					 	)
target_link_libraries(rov_ctrl ${catkin_LIBRARIES})