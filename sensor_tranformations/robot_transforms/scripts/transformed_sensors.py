#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf import TransformListener
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped


class robosub_sensors:

    def __init__(self):

        # Used separate variables to make code more readable
        # Data subscribed and published
        self.imu_data = Imu()
        self.dvl_vel_data = TwistWithCovarianceStamped()
        self.dvl_depth_data = Float64MultiArray()
        self.imu_transformed_data = Imu()
        self.dvl_vel_transformed_data = TwistWithCovarianceStamped()
        self.dvl_depth_transformed_data = PoseWithCovarianceStamped()

        # vector3s structures for DVL velocity
        self.dvl_vel_v3s = Vector3Stamped()
        self.dvl_vel_transformed_v3s = Vector3Stamped()

        # vector3s structures for DVL depth
        #self.dvl_depth_v3s = Vector3Stamped()
        #self.dvl_depth_transformed_v3s = Vector3Stamped()

        # vector3s and quaternion structures for IMU
        self.imu_linear_v3s = Vector3Stamped()
        self.imu_angular_v3s = Vector3Stamped()
        self.imu_quaternion = QuaternionStamped()
        self.imu_linear_transformed_v3s = Vector3Stamped()
        self.imu_angular_transformed_v3s = Vector3Stamped()
        self.imu_quaternion_transformed = QuaternionStamped()

        # These are the transformed frames that will be published
        self.imu_transformed = Imu()
        self.dvl_vel_transformed = TwistWithCovarianceStamped()
        #self.dvl_depth_transformed = PoseWithCovarianceStamped()

        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu_raw', Imu, self.imu_callback)
        # self.dvl_vel_sub = rospy.Subscriber('/DVL_vel', TwistWithCovarianceStamped, self.dvl_vel_callback)
        #self.dvl_depth_sub = rospy.Subscriber('/depth_new', PoseWithCovarianceStamped, self.dvl_depth_callback)

        # Publishers
        self.imu_transformed_pub = rospy.Publisher('/imu_transformed', Imu, queue_size=1)
        self.dvl_vel_transformed_pub = rospy.Publisher('/dvl_vel_transformed', TwistWithCovarianceStamped, queue_size=1)
        #self.dvl_depth_transformed_pub = rospy.Publisher('/dvl_depth_transformed', PoseWithCovarianceStamped, queue_size=1)

        # Transform Listeners
        self.listener1 = TransformListener()
        self.listener2 = TransformListener()
        self.listener3 = TransformListener()

    def imu_callback(self, imu):
        self.imu_data = imu
        try:

            # If you don't want to cache the transform, one thing might be to use ros::Time(0)
            # instead of ros::Time::now(). Whatever now is, the listener will always have to
            # wait until the message from time=now arrives. With Time(0) it will just take the
            # last one (which what we want).
            now = rospy.Time(0)

            # If you have a static transform segment only waitForTransform will
            # return immediately for any time query so it will only cost you the
            # check that everything is connected which is relatively light weight.
            self.listener2.waitForTransform("/base_link", "/base_imu", now, rospy.Time(0.1))

            self.imu_linear_v3s.vector = self.imu_data.linear_acceleration
            self.imu_angular_v3s.vector = self.imu_data.angular_velocity
            self.imu_quaternion.quaternion = self.imu_data.orientation
            #self.imu_linear_v3s.header = self.imu_data.header
            #self.imu_angular_v3s.header = self.imu_data.header
            #self.imu_quaternion.header = self.imu_data.header
            self.imu_linear_v3s.header.frame_id = "base_imu"
            self.imu_angular_v3s.header.frame_id = "base_imu"
            self.imu_quaternion.header.frame_id = "base_imu"
            self.imu_linear_transformed_v3s = self.listener2.transformVector3("/base_link", self.imu_linear_v3s)
            self.imu_angular_transformed_v3s = self.listener2.transformVector3("/base_link", self.imu_angular_v3s)
            self.imu_quaternion_transformed = self.listener2.transformQuaternion("/base_link", self.imu_quaternion)
        except(tf.LookupException, tf.ConnectivityException):
            pass
        self.imu_transformed_data.orientation = self.imu_quaternion_transformed.quaternion
        self.imu_transformed_data.angular_velocity = self.imu_angular_transformed_v3s.vector
        self.imu_transformed_data.linear_acceleration = self.imu_linear_transformed_v3s.vector
        self.imu_transformed_data.header.frame_id = "base_link"
        self.imu_transformed_data.header.stamp = self.imu_data.header.stamp
        self.imu_transformed_pub.publish(self.imu_transformed_data)

        print self.imu_angular_transformed_v3s


    def dvl_vel_callback(self, dvl):
        self.dvl_vel_data = dvl
        try:
            now = rospy.Time(0)
            self.listener1.waitForTransform("/base_link", "/base_dvl", now, rospy.Time(0.001))
            self.dvl_vel_v3s.vector.x = self.dvl_vel_data.twist.twist.linear.x
            self.dvl_vel_v3s.vector.y = self.dvl_vel_data.twist.twist.linear.y
            self.dvl_vel_v3s.vector.z = self.dvl_vel_data.twist.twist.linear.z
            self.dvl_vel_v3s.header.frame_id = "base_dvl"
            self.dvl_vel_transformed_v3s = self.listener1.transformVector3("/base_link", self.dvl_vel_v3s)
        except(tf.LookupException, tf.ConnectivityException):
            pass

        self.dvl_vel_transformed_data.twist.twist.linear = self.dvl_vel_transformed_v3s.vector
        self.dvl_vel_transformed_data.header.frame_id = "base_link"
        self.dvl_vel_transformed_data.header.stamp = self.dvl_vel_data.header.stamp
        self.dvl_vel_transformed_pub.publish(self.dvl_vel_transformed_data)
        #print self.dvl_transformed_v3s

    # def dvl_depth_callback(self, dvl_depth):
    #     self.dvl_depth_data = dvl_depth
    #     try:
    #         now = rospy.Time(0)
    #         self.listener3.waitForTransform("/base_link", "/base_dvl", now, rospy.Time(0.001))
    #         self.dvl_depth_v3s.vector.x = 0
    #         self.dvl_depth_v3s.vector.y = 0
    #         self.dvl_depth_v3s.vector.z = self.pose.pose.position.z
    #         self.dvl_depth_v3s.header.frame_id = "base_dvl"
    #         self.dvl_depth_transformed_v3s = self.listener3.transformVector3("/base_link", self.dvl_vel_v3s)
    #     except(tf.LookupException, tf.ConnectivityException):
    #         pass
    #
    #     self.dvl_depth_transformed_data.pose.pose.position.x = 0
    #     self.dvl_depth_transformed_data.pose.pose.position.y = 0
    #     self.dvl_depth_transformed_data.pose.pose.position.z = self.dvl_depth_transformed_v3s.vector.z
    #     self.dvl_depth_transformed_data.header.frame_id = "base_link"
    #     self.dvl_vel_transformed_data.header.stamp = self.dvl_depth_data.header.stamp


def main():

    rospy.init_node('temp_frame_changer', anonymous=True)
    f1 = robosub_sensors()
    rate = rospy.Rate(100)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    main()
