# ROS for FRC

The goal of this repository is to provide a working example and framework of an FRC robot using ROS to other teams, so that they too can explore the advantages of ROS. This repository hosts the ROS code from the 2020 FRC season robot created by [FRC Team 624](http://team624.org/) and is configured for the game Infinite Recharge with certain sensors. However, it is **not** difficult to implement your own sensors and game specific content into this framework. When testing on a non-FRC robot, the proxy node can be replaced to interface with your hardware. If you are interested in learning how to get ROS working for yourself with a simulator and the physical robot or looking for a more detailed documentation, check out the [ROS wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repository.

## Demo (Video)
//Whenever you clicked on the link, it opens it in the same tab. I looked into it and Markdown doesn't support opening in a new tab so I would suggest using HTML for the video or embedding it within the page.
This is a quick demo that briefly covers many of the features that our team utilized this season. [![Video](https://i9.ytimg.com/vi/V9NnU-9PFkE/mqdefault.jpg?time=1589047334568&sqp=CKDf2_UF&rs=AOn4CLDyWaLY9m2FdYP9mlALuzRIFnO0Kw)](https://www.youtube.com/watch?v=V9NnU-9PFkE)

## Getting Started
NOTE: This repository was made to run on Ubuntu 16 with ROS Kinetic already installed. If you don't have this set up, there are many ways to do this which are outlined within the [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repository. If you want to quickly get set up and run a demo without installing Ubuntu, follow the quick start instructions for [ROS Development Studio](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/Quick-Start:--ROS-Development-Studio).

Before starting, make sure to install the ROS package below. It is critical that this package is properly installed as the whole repository is dependent upon it.

`sudo apt-get install ros-kinetic-robot-pose-ekf`

In order to run a simple simulation of the robot code, clone the workspace below into your home folder.

`git clone https://github.com/LeonidasVarveropoulos/robot-frc.git`


### Subrepositories

There are two subrepositories within the main repository: `diff_drive` and `realsense-ros`. In order to use these two subrepositories, you must first initialize and update them. For more information on how to use submodules, visit the [wiki](https://git-scm.com/book/en/v2/Git-Tools-Submodules). 

//This statement is unclear. Which sub repos are dependent on me having a real camera? Which ones must I install regardless of whether or not I have a camera?
**NOTE: If you do not plan to use or physically don't have the realsense cameras, do not initialize the realsense-ros sub-repository because there are additional items you need to install first. To continue with the demo, just remove the directory from the project**

#### diff_drive

First, change the directory to the sub-repository. Then, initialize the submodule and update it.

```
cd robot-frc/src/diff_drive
git submodule init
git submodule update
```

#### realsense-ros

First, **only if you have the physical cameras**, install the necessary dependencies by following the instructions on the [realsense repository](https://github.com/IntelRealSense/realsense-ros). Afterwards, change the directory to the sub-repository. Then initialize and update the repository, instead of cloning it, into the directory as the normal instruction specify.

```
cd robot-frc/src/realsense-ros
git submodule init
git submodule update
```

### Building

Now you should see all of the directories and files within those sub-repositories. Next, build the Catkin workspace in the repository's base directory and make sure that there were no errors.

```
cd
cd robot-frc
catkin_make
```

### Running

In order to run a simple example of an autonomous executing in simulation, run the following commands in different terminals:

`source ./scripts/demo.sh`

Launches the launch file to start all of the nodes in simulation.

`rviz`

Launches a visualization tool for viewing the robots odometry and other information.

`rqt --perspective-file ./config/DefaultDash.perspective`

Launches a tool for managing the ROS system while the simulation is running.

### Starting Autonomous

In order to start the ROS state machine that manages autonomous, we need to publish two topics: `auto/state` and `auto/select`. While we could do this through the command line with `rostopic pub`, it is much easier to use the tool `rqt` that you launched above. To load the pre-made dashboard, go to the perspectives tab of the app and choose to load in a configuration file with the path of `robot-frc/rqt/Default.perspective`. 

Once loaded in, you will see an option to publish to these two topics. Simply check the box with the topic name to have it publish. To start the auton, set `auto/state` to True and `auto/select` to 1. This will start the auton and you should be able to see ROS logging what the state machine is doing on the left side of rqt. If you switch over to rviz you should see the robot following the paths given. 

To stop and reset the auton state machine, set the `auto/state` to false. On a non-simulated robot, this would be accomplished by disabling the robot.

### More Info
For more information on how to configure ROS for your own physical robot and learn how to create your own autonomous, visit the repository's [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki).

# ROS API

//Add a description about what the ROS API is and how it is being used.

//I changed some of the descriptions for grammar, double check the meaning hasn't changed
## 1. diff_drive_go_to_goal
This is a modified documentation of the forked [repository](https://github.com/merose/diff_drive). We only use the `diff_drive_go_to_goal` node in this repo.

### Published Topics

`distance_to_goal` (std_msgs/Float32)
  - The distance to the current waypoint
 
`cmd_vel` (geometry_msgs/Twist)
  - The needed velocity to reach the goal

`diff_drive/goal_achieved` (std_msgs/Bool)
  - Checks if the singular goal or waypoint within a path is met
  
`diff_drive/path_achieved` (std_msgs/Bool)
  - Checks if the given path is complete
  
`diff_drive/waypoints_achieved` (diff_drive/BoolArray)
  - An array of booleans that represent all the completed waypoints in the given path

### Subscribed Topics

There are two different sets of subscribed topic that are being used: one is for setting a singular simple goal with the default parameters through rviz and the other is for setting a path of waypoints with their own set of parameters. This allows for more configurability and is usually done in autonomous.

`move_base_simple/goal` (geometry_msgs/PoseStamped)
  - The desired singular goal pose.
  
`odom` (nav_msgs/Odometry)
  - The robot's current pose

`diff_drive/goal_path` (diff_drive/GoalPath)
  - The desired path of waypoints with separate parameters
  
### Parameters

`rate` (float, default: 10)

  - The rate at which to publish desired velocities (Hz).

**NOTE: The following parameters are the default parameters that will start at the beginning, but will be overwritten by the parameters specified in the** `diff_drive/goal_path` **msg**

`max_linear_speed` (float, default: 0.2)

  - The maximum linear speed toward the goal (meters/second).

`min_linear_speed` (float, default: 0.1)

  - The minimum linear speed toward the goal (meters/second).

`max_angular_speed` (float, default: 2.0)

  - The maximum angular speed toward the goal (radians/second).

`min_angular_speed` (float, default: 1.0)

  - The minimum angular speed toward the goal (radians/second).
  
`max_linear_acceleration` (float, default: 1E9)

  - The maximum acceleration toward the goal (meters/second).
  
`max_angular_acceleration` (float, default: 1E9)

  - The maximum angular acceleration toward the goal (radians/second).
  
**NOTE: The following inner/outer tolerances work so that the code recognizes that you are within the tolerance when the robot gets within the inner parameters and will only change to outside the tolerance if it passes outside of the outer tolerance. This is made to avoid situations where the robot is on the edge of the tolerance and keeps switching between states, never reaching the goal**
  
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

  - If true, it will only check for the linear tolerances when deciding if it reached a waypoint or goal. If false, it will perform normally, checking if both linear and angular are within tolerance

`Kp` (float, default: 1.0)

  - Linear distance proportionality constant. Larger values make the robot accelerate faster toward the goal and decelerate slower.

`Ka` (float: default: 6.0)

  - Proportionality constant for angle to goal position. Larger values make the robot turn faster toward the goal.

`Kb` (float: default: -0.8)
  - Proportionality constant for angle to goal pose direction. Larger values make the robot turn faster toward the goal pose direction. This value should be negative. 
  //Is higher value closer to 0 or higher in magnitude?

