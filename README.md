# FRC Robotics with ROS
The goal of this repository is to provide a working example and framework of an FRC robot using ROS to other teams, so that they too can explore the advantages of ROS. This repository hosts the ROS code from the 2020 FRC Season Robot created by [FRC Team 624](http://team624.org/) and is configured for the game Infinite Recharge with certain sensors. However, it is **not** difficult to implement your own sensors and game specific content into this framework. When testing on a non-FRC robot, the proxy node can be replaced to interface with your hardware. If you are interested in learning how to get ROS working for yourself with a simulator and the physical robot or looking for a more detailed documentation, check out the [ROS wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repository.

## Demo (Video)
This is a quick demo that briefly covers many of the features that our team utilized this season.

[![Video](https://user-images.githubusercontent.com/55664403/81489736-dfe25080-923e-11ea-8f55-c2d01ca1d112.jpg)](https://www.youtube.com/watch?v=V9NnU-9PFkE)

## Getting Started
NOTE: This repository was made to run on Ubuntu 16 with ROS Kinetic already installed. If you don't have this set up, there are many ways to do this which are outlined within the [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repository. If you want to quickly get set up and run a demo without installing Ubuntu, follow the quick start instructions for [ROS Development Studio](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/Quick-Start:--ROS-Development-Studio).

Before starting, make sure to install the ROS package below. It is critical that this package is properly installed as the repository is dependent upon it.

`sudo apt-get install ros-kinetic-robot-pose-ekf`

In order to run a simple simulation of the robot code, clone the workspace below into your home folder.

`git clone https://github.com/LeonidasVarveropoulos/robot-frc.git`


### Subrepositories

There are two subrepositories within the main repository: `diff_drive` and `realsense-ros`. In order to use these two subrepositories, you must first initialize and update them. For more information on how to use submodules, visit the [wiki](https://git-scm.com/book/en/v2/Git-Tools-Submodules). 

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

The parameters effecting many of the nodes existing within this repository are documented below.

## 1. [diff_drive_go_to_goal](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/ROS-API:-diff_drive_go_to_goal)
This is a ROS node that publishes cmds to the robot in order to follow the given path. This is a modified documentation of the forked [repository](https://github.com/merose/diff_drive). We only use the `diff_drive_go_to_goal` node in this repo.
  
## 2. [autonomous](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/ROS-API:-autonomous)
This is a ROS node that runs the python auton scripts created as a statemachine during autonomous.

## 3. [path_editor](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/ROS-API:-path_editor)
This is a ROS node that runs a web app with Flask for creating autonomous paths. All data is stored within a file called data.txt in the package.

## 4. [proxy](https://github.com/LeonidasVarveropoulos/robot-frc/wiki/ROS-API:-proxy)
This is a ROS node 
