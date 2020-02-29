add_executable(move_group_inf_tuto src/move_group_inf_tuto.cpp)
target_link_libraries(move_group_inf_tuto ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_inf_tuto DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_test src/move_test.cpp)
target_link_libraries(move_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(joyandgo src/joyandgo.cpp)
target_link_libraries(joyandgo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

