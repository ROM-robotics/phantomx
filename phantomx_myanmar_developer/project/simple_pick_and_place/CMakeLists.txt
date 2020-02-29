



add_executable(myanmar_get_current_position myanmar_get_current_position.cpp)
target_link_libraries(myanmar_get_current_position ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myanmar_get_current_position DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(myanmar_simple_grasp_server myanmar_simple_grasp_server.cpp)
target_link_libraries(myanmar_simple_grasp_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myanmar_simple_grasp_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(myanmar_simple_grasp_client myanmar_simple_grasp_client.cpp)
target_link_libraries(myanmar_simple_grasp_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myanmar_simple_grasp_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(myanmar_pick_pose_publisher myanmar_pick_pose_publisher.cpp)
target_link_libraries(myanmar_pick_pose_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myanmar_pick_pose_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(add_collision_object add_collision_object.cpp)
target_link_libraries(add_collision_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_collision_object DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_tutorial move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

