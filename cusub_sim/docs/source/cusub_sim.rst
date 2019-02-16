*********
CUSub_Sim
*********

This section will go into the details of the gazebo based robosub simulator.

Building the sim
################

in your robosub workspace clone the cusub_sim repository into your package src/ folder
::

	git clone https://github.com/CU-Robosub/cusub_sim

Next you will have to use the git submodule to import the UUV simulator packages.
::

	git submodule init
	git submodule update

Once you have pulled both the cusub_sim repo as well as the UUV simulator submodule you will want ot build the system form the root of your workspace.
::

	catkin_make


Running the sim
###############

Once you are able to build the system without errors you can launch the basic sim.
::
	
	source ~/<your_robosub_ws>/devel/setup.bash
	roslaunch cusub_sim divewell_startgate_setpoint.launch

Keep in mind that this ONLY launches the simulator enviroment and objects. It does not launch the control packages or state estimation packages require to control the model. For more information check the cusub_common package.

if you have any issues or questions be sure to post within the slack or attend the next software meeting.


System Packages
###############

Will contain a list of system packages as well as links to package specific documentation. Including external site links, hand written package pages and synthesized pages based on code comments.



ROS Topic Interface
###################

Will contain a list of topics typically created by this meta package, their namespace conventinons as well as their basic uses.


Known Issues
############

Will contain information about known issues with this package and common methods for resolving them. 
