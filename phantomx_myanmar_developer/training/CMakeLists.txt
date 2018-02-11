add_executable(move_group_inf_tuto src/move_group_inf_tuto.cpp)
target_link_libraries(move_group_inf_tuto ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_inf_tuto DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

