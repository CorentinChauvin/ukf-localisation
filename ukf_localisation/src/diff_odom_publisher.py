#!/usr/bin/env python
"""
    Publish an odometry message resulting from the difference between two odom
    messages
"""

from math import sqrt, sin, cos, atan2
import rospy
import tf
from nav_msgs.msg import Odometry


class DiffNode():
    def __init__(self):
        # ROS parameters
        self.publish_frequency = rospy.get_param('~publish_frequency', 0.0)
        frame_id = rospy.get_param('~frame_id', '')
        child_frame_id = rospy.get_param('~child_frame_id', '')
        verbose = rospy.get_param('~verbose', True)

        # Initialise variables
        self.odom1 = None
        self.odom2 = None

        # ROS publishers
        output_publisher = rospy.Publisher('diff_odom', Odometry, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('odom1', Odometry, self.odom1_callback)
        rospy.Subscriber('odom2', Odometry, self.odom2_callback)


        # MAIN LOOP
        rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            if (self.odom1 is not None) and (self.odom2 is not None):
                diff = Odometry()
                diff.header.stamp = rospy.get_rostime()
                diff.header.frame_id = frame_id
                diff.child_frame_id = child_frame_id

                diff.pose.pose.position.x = (self.odom1.pose.pose.position.x
                                            - self.odom2.pose.pose.position.x
                                            )
                diff.pose.pose.position.y = (self.odom1.pose.pose.position.y
                                            - self.odom2.pose.pose.position.y
                                            )
                diff_theta = self.compute_angle_difference(
                                    self.odom1.pose.pose.orientation,
                                    self.odom2.pose.pose.orientation
                                    )
                quaternion = tf.transformations.quaternion_from_euler(0, 0, diff_theta)
                diff.pose.pose.orientation.x = quaternion[0]
                diff.pose.pose.orientation.y = quaternion[1]
                diff.pose.pose.orientation.z = quaternion[2]
                diff.pose.pose.orientation.w = quaternion[3]

                diff.twist.twist.linear.x = (self.odom1.twist.twist.linear.x
                                            - self.odom2.twist.twist.linear.x
                                            )
                diff.twist.twist.linear.y = (self.odom1.twist.twist.linear.y
                                            - self.odom2.twist.twist.linear.y
                                            )
                diff.twist.twist.angular.z = (self.odom1.twist.twist.angular.z
                                            - self.odom2.twist.twist.angular.z
                                            )

                output_publisher.publish(diff)

                if verbose:
                    rospy.loginfo('Diff odom:')
                    rospy.loginfo("d_x={0:.3f} ; d_y={0:.3f} ; d_theta={0:.3f}".format(
                        diff.pose.pose.position.x,
                        diff.pose.pose.position.y,
                        diff_theta
                    ))
                    rospy.loginfo("d_v_x={0:.3f} ; d_v_y={0:.3f} ; d_w_z={0:.3f}".format(
                        diff.twist.twist.linear.x,
                        diff.twist.twist.linear.y,
                        diff.twist.twist.angular.z,
                    ))

            rate.sleep()


    def odom1_callback(self, msg):
        self.odom1 = msg


    def odom2_callback(self, msg):
        self.odom2 = msg


    def compute_angle_from_quaternion(self, orientation):
        """ Compute yaw from a quaternion (geometry_msgs/Quaternion)
        """

        return tf.transformations.euler_from_quaternion([
                                                        orientation.x,
                                                        orientation.y,
                                                        orientation.z,
                                                        orientation.w
                                                        ])[2]


    def compute_angle_difference(self, last_orientation, new_orientation):
        """ Compute the angle difference (rad) between two orientations (geometry_msgs/Quaternion)
        """

        last_angle = self.compute_angle_from_quaternion(last_orientation)
        new_angle = self.compute_angle_from_quaternion(new_orientation)

        return new_angle - last_angle


if __name__ == '__main__':
    rospy.init_node('diff_odom_publisher')
    DiffNode()
    rospy.spin()
