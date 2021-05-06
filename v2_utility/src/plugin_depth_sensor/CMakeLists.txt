############# Plugin for simulating depth sensor #############
add_library(plugin_depth SHARED
    plugin_depth.cpp
)

target_link_libraries(plugin_depth
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

install(TARGETS plugin_depth
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)