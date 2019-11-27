cmake_minimum_required(VERSION 2.8.3)
project(phantomx_myanmar_custom_msg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   custom.srv  
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   Grasp.action
   PickAndPlace.action
   MoveToTarget.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs   geometry_msgs   std_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phantomx_myanmar_custom_msg
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

