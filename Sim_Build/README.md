# Simulator Build
---
This is the catkin workspace for building the simulator for the Leviathan and Triton AUVs.  To initialize the workspace run:
```
wstool update
````
Additional things you will need to install for the catkin_make to succeed as well as all the other ros components execute:
```
apt-get install protobuf-c-compiler # UUV sim needs this to build

# Change the -kinetic- to melodic if you are on Ubuntu 18, everything still works
apt-get install ros-kinetic-robot-localization

````
Currently recomended to run the TRANSDEC launch for simulation.  Has 2018 obstacles
```
source devel/setup.bash
roslaunch launch/gazebo_TRANSDEC.launch
```
