#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_pololu_servo.srv import MotorRange
from ros_pololu_servo.msg import MotorCommand
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorStateList

class Motor_Controller():
    def __init__(self):
        self.command_sub = rospy.Subscriber('/DVL_depth', Float64MultiArray, self.command_callback, queue_size=1)
        self.pub_Pose = rospy.Publisher('/depth', PoseWithCovarianceStamped,queue_size=1)
        self.pub_Pose_data = PoseWithCovarianceStamped()
    def command_callback(self,msg):
        data = msg.data
        minD = data[0]
        for x in data:
            if(x < minD):
                minD = x
        self.pub_Pose_data.header.stamp = rospy.get_rostime()
        self.pub_Pose_data.header.frame_id = "base_link"
        self.pub_Pose_data.pose.pose.position.z = minD
        self.pub_Pose.publish(self.pub_Pose_data)


def main():
    rospy.init_node('Stab_Pololu')
    m = Motor_Controller()
    rospy.spin()

if __name__ == "__main__":
    main()
