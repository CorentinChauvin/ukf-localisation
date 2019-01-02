#!/usr/bin/env python
"""
    Removes offsets in the IMU output
    Assumes that the IMU is hold horizontally
"""

import roslib
import rospy
from sensor_msgs.msg import Imu


class CenterImuNode():
    def __init__(self):
        # ROS parameters
        wait_time = rospy.get_param('~wait_time', 0.0)             # time to wait [s] before the initialisation
        averaging_time = rospy.get_param('~averaging_time', 0.0)   # time of the initialisation

        # Initialise variables
        self.offset_a_x = 0.0
        self.offset_a_y = 0.0
        self.offset_a_z = 0.0
        self.offset_w_x = 0.0
        self.offset_w_y = 0.0
        self.offset_w_z = 0.0
        self.nbr_measurements = 0.0
        self.init_running = False
        self.init_done = False

        # ROS publishers
        self.imu_centered_publisher = rospy.Publisher('imu_centered', Imu, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('imu', Imu, self.imu_callback)

        # Offsets initialisation
        rospy.sleep(wait_time)

        rospy.loginfo("Centering IMU for {} seconds...".format(averaging_time))
        self.init_running = True
        rospy.sleep(averaging_time)

        rospy.loginfo("IMU centered")
        self.init_running = False
        self.offset_a_x /= self.nbr_measurements
        self.offset_a_y /= self.nbr_measurements
        self.offset_a_z /= self.nbr_measurements
        self.offset_a_z /= self.nbr_measurements
        self.offset_w_x /= self.nbr_measurements
        self.offset_w_y /= self.nbr_measurements
        self.offset_w_z /= self.nbr_measurements
        self.init_done = True


    def imu_callback(self, msg):
        """ Callback for the IMU message

            When init is running, sums the messages to average the offsets
            When init is done, removes the offsets and publish a new imu message
        """

        if self.init_running:
            self.offset_a_x += msg.linear_acceleration.x
            self.offset_a_y += msg.linear_acceleration.y
            self.offset_a_z += msg.linear_acceleration.z
            self.offset_w_x += msg.angular_velocity.x
            self.offset_w_y += msg.angular_velocity.y
            self.offset_w_z += msg.angular_velocity.z
            self.nbr_measurements += 1
        elif self.init_done:
            imu_centered = msg
            imu_centered.linear_acceleration.x -= self.offset_a_x
            imu_centered.linear_acceleration.y -= self.offset_a_y
            imu_centered.linear_acceleration.z -= self.offset_a_z
            imu_centered.angular_velocity.x -= self.offset_w_x
            imu_centered.angular_velocity.y -= self.offset_w_y
            imu_centered.angular_velocity.z -= self.offset_w_z

            self.imu_centered_publisher.publish(imu_centered)


if __name__ == '__main__':
    rospy.init_node('center_imu')
    CenterImuNode()
    rospy.spin()
