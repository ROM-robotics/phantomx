
#!/bin/sh
gnome-terminal -e 'roscore' --name="MASTER"

sleep 3
gnome-terminal -e "roslaunch phantomx_reactor_arm_moveit_config demo.launch" --name="DEMO"
sleep 10
gnome-terminal -e "rosrun phantomx_myanmar_developer move_group_interface_tutorial" --name="move_group_tuto"

exit

# For debugging purpose use shell scripts, for run in one command use LAUNCH