#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/2020-LattePandaROS/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5

roslaunch rs_launcher rs_latte_panda.launch &
roslaunch robot_urdf rviz_model.launch &
roslaunch localization localization.launch
