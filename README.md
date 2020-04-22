# robot-frc

The goal for this repository is to provide a working example and framework of an frc robot using ROS to other teams, so that they too can explore the advantages of ROS. This repository is the code from the 2020 frc season and is configured for the game with certain sensors, but it **is simple** to implement your own sensors and game specific content. If you're intrested in quickly getting this setup and trying it out for yourself with a simulator on a virtual machine or are looking for a more detailed documentation check out the ROS [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) of this repo. For more information about how our team went around this task and how you could structure your robot code read this "paper".

## Used Packages

* `diff_drive` -- Provides the nodes and msgs nessesary for moving along a given path of waypoints.
* `autonomous` -- Provides a state machine for easily creating an autonomous.
* `localization` -- This provides the nodes needed for taking in sensor data and getting the robot's pose.
* `rs_launcher` -- Launches the realsense cameras in their own node.
* `cmd_vel_to_rpm` -- Converts the ROS cmd_vel msg into an rpm sent to the RoboRio.
* `proxy` -- Listens to all relavant topics and communicates with the RoboRio through ethernet/networktables.
* `robot_urdf` -- Sets up the nodes needed for creating the tf tree of the robot.
* `sim_robot` -- Creates a simulated model of the robot's features and sensors for testing without the physical robot.
* `turret_rotation` -- This is the PID for the turret's rotation.
* `vision_targeting` -- Does the vision processing for the 2020 game target.
* `vision_targeting_tuner` -- Provides a simple UI for changing the hsv values of the color filter.
* `path_editor` -- Provides a web app for changing and creating paths of waypoints for autonomous.

## Getting Started
This was made to run on Ubuntu 16 with ROS Kinetic already installed, for instructions on how to get this set up go to the repo's wiki.

In order to run a simple simulation of the robot code first clone this workspace into your home folder.

`git clone https://github.com/LeonidasVarveropoulos/robot-frc.git`

There are two subrepositories within this main one, `diff_drive` and `realsense-ros`. In order to use this you must first initialize them and then update. For more information on how to use [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). 

**NOTE: If you do not plan to use or physically don't have the realsense cameras do not initialize the sub-repo beacuase there are other things you need to install first.**

### diff_drive

First change directory to the sub-repo then init and update.

```
cd robot-frc/src/diff_drive
git submodule init
git submodule update
```

### realsense-ros

First, **only if you have the physical cameras** install the nessesary dependancies by following the instructions on the repo's [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki). Afterwards change directory to the sub-repo then init and update instaed of cloning the repo into the directory as the normal instruction specify .

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

In order to run a simple example of an autonomous executing in simulation run the following commands in **different terminals**.

In the repo's base directory, launches the launch file to start all nodes in simualtion

`sudo ./scripts/sim_startup.sh`

Launches a useful vizualization tool for viewing the robots odom and other topics.

`rviz`

Launches a useful tool for managing the ros system while running.

`rqt`

### Starting Autonomous

In order to start the ros state machine managing autonomous we need to publish two topics, `auto/state` and `auto/select`. We could do this through the command line with `rostopic pub` but it is much easier to use the tool rqt that you launched above. To load the pre-made dashboard go to the perspectives tab of the app and choose to load in a configuration file with the path of `robot-frc/rqt/Default.perspective`. 

Once loaded in you will see an option to publish to these two topics simply by checking the box with the topic name. To start the auton set `auto/state` to True and `auto/select` to 1. This will start the auton and you should be able to see the ros logging of what the state machine is doing on the left side of rqt. If you switch over to rviz you should see the robot following the paths given.

### More Info
For more information on how to configure this for you own physical robot vist the repo's [wiki](https://github.com/LeonidasVarveropoulos/robot-frc/wiki) and paper
