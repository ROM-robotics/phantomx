#!/bin/sh
gnome-terminal -e 'roscore' --name="MASTER"

sleep 3
gnome-terminal --name="Driver" -e "roslaunch phantomx_reactor_arm_controller dynamixel_phantomx_reactor_arm_no_wrist.launch"

sleep 3
gnome-terminal --name="Rviz" -e "roslaunch phantomx_test_moveit_config demo_real.launch"

sleep 3
gnome-terminal --name="add_obj" -e "rosrun phantomx_myanmar_developer add_collision_object"

exit
# For debugging purpose use shell scripts, for run in one command use LAUNCH

