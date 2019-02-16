# Final_Build
This repo acts as a the base catkin directory for the sub's ROS packages.

The Sub Repo's are organized as such
![Repo_Graph](https://user-images.githubusercontent.com/2038191/28999287-e048f1c8-79f5-11e7-9a24-636e41ebb8e7.jpg)

Our project uses [ROS](http://wiki.ros.org/)

[CU Robosub Site](http://curobotics.org/robosub-2017)
## Requirements
### System
* Ubuntu 16.04
* ROS Kinetic - [Installation Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)
### Hardware
* You are going to need a sub
## Set up
To get the submodules to work run these commands:
* ```git submodule init```
* ```git submodule update```
* ```./scripts/init.sh``` (Under testing still - results may vary)
* ```catkin_make```
* ```source devel/setup.bash```
## Execution
* $ roslaunch launch/main.launch

--OR--

* You may launch any sub launch file or run any script on its own. For direction see [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
## Contents
* ### launch
  * main.launch [Launch File Info](http://wiki.ros.org/roslaunch)
   * All other launch files are linked to this one. This means one launch file as to be started to start the sub.
   * This file also starts a bag file to record each run. [ROS bag Info](http://wiki.ros.org/Bags)
  * dolphin.launch
   * Launch all launches in main.launch except Global Controller and Bag files
   * The purpose of this is for testing in small pools.
* ### scripts - Scripts to make common operations easier
  * git_add.sh - adds all changes made to the sub during a test run
  * git_pull.sh - pulls the lastest changes from all of the submodules
  * init.sh - Installs required packages
* ### src - This is where all the packages and submodules live
  * [Computer_Vision](https://github.com/CU-Robosub/Computer_Vision)
  * [Debugging-Console](https://github.com/CU-Robosub/Debugging-Console) (Under Construction)
  * [Global_Control](https://github.com/CU-Robosub/Global_Control)
  * [Local_Control](https://github.com/CU-Robosub/Local_Control)
  * [drivers](https://github.com/CU-Robosub/drivers)
  * [Sensor_Intergration](https://github.com/CU-Robosub/Sensor_Intergration)
