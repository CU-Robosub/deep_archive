#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

class frame_changer:

    def __init__(self):
        self.imu_data = Imu()
        self.dvl_data = TwistWithCovarianceStamped()
        self.depth_data = PoseWithCovarianceStamped()
        self.imu_sub = rospy.Subscriber("/imu_raw", Imu, self.callback_imu)
        self.dvl_sub = rospy.Subscriber("/DVL_depth", Float64MultiArray, self.callback_depth)
        # self.dvl_sub = rospy.Subscriber("/depth", PoseWithCovarianceStamped, self.callback_depth)
        self.imu_pub = rospy.Publisher("/imu_new", Imu, queue_size=1)
        self.dvl_pub = rospy.Publisher("/depth", PoseWithCovarianceStamped, queue_size=1)
        # self.depth_pub = rospy.Publisher("/depth_new", PoseWithCovarianceStamped, queue_size=1)
        self.depth_avg = 0

    def callback_imu(self, imu):
        self.imu_data = imu
        self.imu_data.orientation_covariance = [1e9,0,0,
                                                0,1e9,0,
                                                0,0,1e9]
        self.imu_data.orientation_covariance = [1e-4,0,0,
                                                0,1e-4,0,
                                                0,0,1e-4]
        self.imu_data.orientation_covariance = [1e2,0,0,
                                                0,1e2,0,
                                                0,0,1e2]
        self.imu_pub.publish(self.imu_data)

    def callback_dvl(self, dvl):
        self.dvl_data = dvl
        self.dvl_data.header.frame_id = "/base_dvl"
        self.dvl_pub.publish(self.dvl_data)

    def callback_depth(self, depth):

        # depth is calculated as average of the 4 transducer values
        for i in range(0,3):
            self.depth_avg += depth.data[i]
        self.depth_avg = self.depth_avg/4
        # self.depth_data.pose.covariance = [-1]
        self.depth_data.pose.pose.position.z = self.depth_avg
        self.depth_data.header.stamp = rospy.get_rostime()
        self.depth_data.header.frame_id = "odom"
        self.dvl_pub.publish(self.depth_data)



def main():

    rospy.init_node('temp_frame_changer', anonymous=True)
    f1 = frame_changer()
    rate = rospy.Rate(100)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    main()
