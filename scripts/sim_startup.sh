#! /bin/bash
source /opt/ros/melodic/setup.bash
source ~/robot-frc/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5

roslaunch robot_launch sim_startup.launch