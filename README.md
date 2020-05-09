# robot-frc

The goal for this repository is to provide a working example and framework of an FRC robot using ROS to other teams, so that they too can explore the advantages of ROS. This repository is the code from the 2020 FRC season and is configured for the game with certain sensors, but it is **not** hard to implement your own sensors and game specific content. Even if testing on a non frc robot the proxy node can be replaced to interface with your hardware. If you're intrested in learning how to get this setup and running for yourself with a simulator and the physical robot or are looking for a more detailed documentation check out the ROS [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repo.

## Demo

This will be a video of a demo

## Getting Started
This was made to run on Ubuntu 16 with ROS Kinetic already installed. If you don't have this set up there are many ways to do this which are outlined within the [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repo.

If you want to quickly get set up with no installation required and try this out for yourself on the web by running a demo of an autonomous look at these quick start instructions for [ROS Development Studio](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/Quick-Start:--ROS-Development-Studio).

Before starting make sure to install this ROS package that the repo depends on.

`sudo apt-get install ros-kinetic-robot-pose-ekf`

In order to run a simple simulation of the robot code first clone this workspace into your home folder.

`git clone https://github.com/LeonidasVarveropoulos/robot-frc.git`


### Subrepositories

There are two subrepositories within this main one, `diff_drive` and `realsense-ros`. In order to use this you must first initialize them and then update. For more information on how to use [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). 

**NOTE: If you do not plan to use or physically don't have the realsense cameras do not initialize the sub-repo beacuase there are other things you need to install first. Instead just remove the directory**

#### diff_drive

First change directory to the sub-repo then init and update.

```
cd robot-frc/src/diff_drive
git submodule init
git submodule update
```

#### realsense-ros

First, **only if you have the physical cameras** install the nessesary dependancies by following the instructions on the realsense [repository](https://github.com/IntelRealSense/realsense-ros). Afterwards change directory to the sub-repo then init and update instead of cloning the repo into the directory as the normal instruction specify .

```
cd robot-frc/src/realsense-ros
git submodule init
git submodule update
```

### Building

Now you should see directories and files within those sub-repositories. Next build the catkin workspace in the repo's base directory and make sure that there were no errors.

```
cd
cd robot-frc
catkin_make
```

### Running

In order to run a simple example of an autonomous executing in simulation run the following commands in different terminals.

In the repo's base directory, launches the launch file to start all nodes in simualtion

`source ./scripts/demo.sh`

Launches a useful vizualization tool for viewing the robots odom and other topics.

`rviz`

Launches a useful tool for managing the ros system while running.

`rqt --perspective-file ./config/DefaultDash.perspective`

### Starting Autonomous

In order to start the ros state machine managing autonomous we need to publish two topics, `auto/state` and `auto/select`. We could do this through the command line with `rostopic pub` but it is much easier to use the tool rqt that you launched above. To load the pre-made dashboard go to the perspectives tab of the app and choose to load in a configuration file with the path of `robot-frc/rqt/Default.perspective`. 

Once loaded in you will see an option to publish to these two topics simply by checking the box with the topic name. To start the auton set `auto/state` to True and `auto/select` to 1. This will start the auton and you should be able to see the ros logging of what the state machine is doing on the left side of rqt. If you switch over to rviz you should see the robot following the paths given. 

To stop and reset the auton state machine set the `auto/state` to false. On a non simulated robot this would be changed by disabling the bot.

### More Info
For more information on how to configure this for your own physical robot and learn how to create your own autonomous, vist the repo's [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki).

# ROS API

## 1. diff_drive_go_to_goal
This is a modified documentation of the forked [repo](https://github.com/merose/diff_drive). We only use the `diff_drive_go_to_goal` node in this repo.

### Published Topics

`distance_to_goal` (std_msgs/Float32)
  - The distance to the current waypoint it is following
 
`cmd_vel` (geometry_msgs/Twist)
  - The needed velocity to reach your goal

`diff_drive/goal_achieved` (std_msgs/Bool)
  - Checks if the singular goal or waypoint within a path is met
  
`diff_drive/path_achieved` (std_msgs/Bool)
  - Checks if the entire path given is complete
  
`diff_drive/waypoints_achieved` (diff_drive/BoolArray)
  - An array of bools that represent all the completed waypoints in he given path

### Subscribed Topics

There are two different sets of subscribed topic that work one is for setting a singular simple goal with the default parameters through rviz and the other is setting a path of waypoints with their own set of paremeters for more configurability, this is usaully done in the autonomous.

`move_base_simple/goal` (geometry_msgs/PoseStamped)
  - Desired singular goal pose.
  
`odom` (nav_msgs/Odometry)
  - The current robot's pose

`diff_drive/goal_path` (diff_drive/GoalPath)
  - Desired path of waypoints with seperate parameters
  
### Parameters

`rate` (float, default: 10)

  - Rate at which to publish desired velocities (Hz).

**NOTE: The following params are the default params that will start at the beginning, but will be overwritten by the params specified in the** `diff_drive/goal_path` **msg**

`max_linear_speed` (float, default: 0.2)

  - The maximum linear speed toward the goal (meters/second).

`min_linear_speed` (float, default: 0.1)

  - The min linear speed toward the goal (meters/second).

`max_angular_speed` (float, default: 2.0)

  - The max angular speed toward the goal (radians/second).

`min_angular_speed` (float, default: 1.0)

  - The min angular speed toward the goal (radians/second).
  
`max_linear_acceleration` (float, default: 1E9)

  - The max acceleration toward the goal (meters/second).
  
`max_angular_acceleration` (float, default: 1E9)

  - The max angular acceleration toward the goal (radians/second).
  
**NOTE: The following inner/outer tolerances work so that the code recognizes that you are within the tolerance when the robot gets within the inner param and will only change to not being in the tolerance if it passes outside of the outer tolerance. This is made to avoid situations where the robot is on the edge of the tolerance and keeps switching between states never reaching the goal**
  
`linear_tolerance_outer` (float, default: 0.3)

  - The outer linear tolerance from the goal (meters)
  
`linear_tolerance_inner` (float, default: 0.1)

  - The inner linear tolerance from the goal (meters)
  
`angular_tolerance_outer` (float, default: 0.2)

  - The outer angular tolerance from the goal (radians)
  
`angular_tolerance_inner` (float, default: 0.1)

  - The inner angular tolerance from the goal (radians)

`forwardMovementOnly` (boolean, default: false)

  - If true, only forward movement is allowed to achieve the goal position.
If false, the robot will move backward to the goal if that is the most
direct path.

`ignore_angular_tolerance` (boolean, default: false)

  - If true, it will only check for the linear tolerances when deciding if it reached a waypoint or goal. If false, it will preform normally, checking if both linear and angular are within tolerance

`Kp` (float, default: 1.0)

  - Linear distance proportionality constant. Higher values make the robot accelerate more quickly toward the goal and decelerate less quickly.

`Ka` (float: default: 6.0)

  - Proportionality constant for angle to goal position. Higher values make the robot turn more quickly toward the goal.

`Kb` (float: default: -0.8)
  - Proportionality constant for angle to goal pose direction. Higher values make the robot turn more quickly toward the goal pose direction. This value should be negative.

