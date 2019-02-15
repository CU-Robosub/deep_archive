#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import tf
import time
import numpy as np
from nav_msgs.msg import Odometry
import cv2
import redis
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

r = redis.Redis(
    host='localhost',
    port=6379)

class Redis_Converter():
    def __init__(self):
        self.pub = rospy.Publisher(rospy.get_param("~odometry"),Odometry,queue_size=1)
        self.image_pub = rospy.Publisher("image_simulator",Image,queue_size=1)
        self.bridge = CvBridge()
        for x in ['x_pos','y_pos','z_pos','x_vel','y_vel','z_vel','roll_pos','pitch_pos','yaw_pos','roll_vel','pitch_vel','yaw_vel','image_block']:
            r.set(x,0)
        for x in ['camera2_red','camera2_green','camera2_blue']:
            r.set(x,"1,1,")

    def update_ros(self):
        odom = Odometry()
        odom.pose.pose.position.x = float(r.get('x_pos'))
        odom.pose.pose.position.y = float(r.get('y_pos'))
        odom.pose.pose.position.z = float(r.get('z_pos'))

        quaternion = tf.transformations.quaternion_from_euler(float(r.get("roll_pos")), float(r.get("pitch_pos")), float(r.get("yaw_pos")))
        #type(pose) = geometry_msgs.msg.Pose
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = float(r.get("x_vel"))
        odom.twist.twist.linear.y = float(r.get("y_vel"))
        odom.twist.twist.linear.z = float(r.get("z_vel"))

        odom.twist.twist.angular.x = float(r.get("roll_vel"))
        odom.twist.twist.angular.y = float(r.get("pitch_vel"))
        odom.twist.twist.angular.z = float(r.get("yaw_vel"))
        odom.header.stamp = rospy.get_rostime()
        self.pub.publish(odom)

        red_temp = r.get('camera2_red').split(',')
        red_temp.pop()
        green_temp = r.get('camera2_green').split(',')
        green_temp.pop()
        blue_temp = r.get('camera2_blue').split(',')
        blue_temp.pop()
        if(float(r.get('image_block'))==0):
            # rospy.logerr("image_block set")
            return
        rospy.logerr("image_block not set")

        red = [int(s) for s in red_temp]
        green = [int(s) for s in green_temp]
        blue = [int(s) for s in blue_temp]
        # rospy.logerr("Red {0}, Green {1}, Blue {2}".format(len(red),len(green),len(blue)))
        n = int(np.sqrt(len(red)))

        image = np.zeros((n,n,3), dtype='uint8')

        for i in range(0,n):
            for j in range(0,n):
                k = i*n+j
                image[i,j,2] = red[k]
                image[i,j,1] = green[k]
                image[i,j,0] = blue[k]

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))



def main():
    rospy.init_node('Redis_Converter')
    m = Redis_Converter()
    # rospy.spin()
    r = rospy.Rate(40)
    while(not rospy.is_shutdown()):
        m.update_ros()
        r.sleep()

if __name__ == "__main__":
    main()
