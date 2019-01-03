#!/usr/bin/env python
"""
    Republish the GPS position and velocity data with in suitable message formats
    for robot_localization
"""

import roslib
import rospy
import dynamic_reconfigure.server
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, PointStamped, Vector3Stamped
from eufs_gazebo.cfg import gpsPublisherConfig


class GpsPublisherNode():
    def __init__(self):
        # ROS parameters
        self.std_position = rospy.get_param('~std_position', 0.0)
        self.std_velocity = rospy.get_param('~std_velocity', 0.0)
        self.publish_frequency = rospy.get_param('~publish_frequency', 0.0)

        # Init variables
        self.last_position = None
        self.last_velocity = None

        # ROS publishers
        self.pose_publisher = rospy.Publisher('gps_pose', PoseWithCovarianceStamped, queue_size=1)
        self.twist_publisher = rospy.Publisher('gps_twist', TwistWithCovarianceStamped, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('gps_position', PointStamped, self.position_callback)
        rospy.Subscriber('gps_velocity', Vector3Stamped, self.velocity_callback)

        # Dynamic reconfigure
        self.configServer = dynamic_reconfigure.server.Server(gpsPublisherConfig, self.dynamicReconfigure_callback)


        # Main loop
        self.rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            if (self.last_position is not None) and (self.last_velocity is not None):
                # Pose message
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = self.last_position.header
                pose_msg.pose.pose.position = self.last_position.point
                pose_msg.pose.covariance[0] = self.std_position**2
                pose_msg.pose.covariance[7] = self.std_position**2
                self.pose_publisher.publish(pose_msg)

                # Twist message (the twist is not in the same frame in hector_gazebo plugin)(why??)
                twist_msg = TwistWithCovarianceStamped()
                twist_msg.header = self.last_velocity.header
                twist_msg.twist.twist.linear.x = -self.last_velocity.vector.y
                twist_msg.twist.twist.linear.y = self.last_velocity.vector.x
                twist_msg.twist.covariance[0] = self.std_velocity**2
                twist_msg.twist.covariance[7] = self.std_velocity**2
                self.twist_publisher.publish(twist_msg)

            self.rate.sleep()


    def position_callback(self, msg):
        """ Callback for the position
        """

        self.last_position = msg


    def velocity_callback(self, msg):
        """ Callback for the velocity
        """

        self.last_velocity = msg


    def dynamicReconfigure_callback(self, config, level):
        """ Dynamic reconfiguration of parameters
        """

        self.std_position = config['std_position']
        self.std_velocity = config['std_velocity']
        self.publish_frequency = config['publish_frequency']

        self.rate = rospy.Rate(self.publish_frequency)

        return config


if __name__ == '__main__':
    rospy.init_node('gps_publisher')
    GpsPublisherNode()
    rospy.spin()
