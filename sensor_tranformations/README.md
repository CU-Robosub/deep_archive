# sensor_tranformations
Contains the transform broadcasters for all the robosub sensors

**How to run ?**

Run the launch file **robosub_transform.launch** in the launch directory. It will launch 3 nodes.

**Files descriptions**

1.**scripts/temp_frame_changer.py** is a temporary file to change the frame ids to test with the existing rosbag logs.

2.**src/tf_broadcaster.cpp** contains the transforms of **base_dvl** and **base_imu** frames to **base_link**. Currently there is only a 
linear translation in the z axis, which **wouldn't create any difference once transformed**. Rotational and linear transformations
should be updated in this file.  

3.**scripts/transformed_sensors.py** publishes the dvl in the **base_link** frame under the topic **"/dvl_transformed"** and imu under 
the topic **"/imu_transformed"**
