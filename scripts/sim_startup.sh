#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/2020-LattePandaROS/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5

roslaunch robot_urdf rviz_model_sim.launch &
roslaunch localization localization_sim.launch &
roslaunch diff_drive demo.launch &
roslaunch sim_robot sim_robot.launch &

# # Launching the autonomous
# python /2020-LattePandaROS/src/autonomous/src/ros_node.py &

# # For viz
# rosrun rviz rviz &
# rqt