#!/usr/bin/env python
"""
    Publish the distance given by integrating an odometry message
"""

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class DistanceNode():
    def __init__(self):
        # ROS parameters
        publish_frequency = rospy.get_param('~publish_frequency', 10.0)

        # Initialise variables
        distance = 0.0
        self.v = 0.0
        dt = 1/publish_frequency

        # ROS publishers
        distance_publisher = rospy.Publisher('distance', Float64, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('odom', Odometry, self.odom_callback)


        # MAIN LOOP
        while not rospy.is_shutdown():
            distance += self.v * dt

            msg = Float64()
            msg.data = distance
            distance_publisher.publish(msg)

            rospy.sleep(dt)


    def odom_callback(self, msg):
        self.v = msg.twist.twist.linear.x


if __name__ == '__main__':
    rospy.init_node('distance_publisher')
    DistanceNode()
    rospy.spin()
