#!/usr/bin/python2

import rospy
from std_msgs.msg import Float64

rospy,init_node('mrgostraight_node')

go_depth_pub = rospy.Publisher('/local_control/pid/depth/set', Float64, queue_size=10)
go_drive_pub = rospy.Publisher('/local_control/pid/drive/set', Float64, queue_size=10)
go_strafe_pub = rospy.Publisher('/local_control/pid/drive/set', Float64, queue_size=10)

rospy.sleep(15)

depth_data = Float64()
depth_data.data = 4

go_depth_pub.publish(depth_data)

drive_data = Float64()
drive_data.data = 10

go_drive_pub.publish(drive_data)

rospy.sleep(20)

strafe_data = Float64()
strafe_data.data = -1

go_strafe_pub.publish(strafe_data)

rospy.sleep(2)

drive_data.data = -10
go_drive_pub.publish(drive_data)
