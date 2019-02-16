mkdir build
source /usr/share/gazebo-8/setup.sh
source /opt/ros/kinetic/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD
cd build
cmake ..
make
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD
