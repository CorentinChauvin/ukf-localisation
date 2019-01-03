#!/usr/bin/env python
"""
    Publish a noised and valid (linear speed in the direction of the motion)
    odometry of the car

    The odometry noise model is mainly based on "Probabilistic Robotics" (5.4
    Odometry Motion Model)
"""

from math import sqrt, atan2, cos, sin, pi, copysign
from random import gauss
import roslib
import rospy
import tf
import tf2_ros as tf2
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
import dynamic_reconfigure.server
from eufs_gazebo.cfg import odometryPublisherConfig


class OdometryPublisherNode():
    def __init__(self):
        # ROS parameters
        self.frame_id = rospy.get_param('~frame_id', '')
        self.child_frame_id = rospy.get_param('~child_frame_id', '')
        self.publish_frequency = rospy.get_param('~publish_frequency', 0.0)
        self.publish_tf_gt = rospy.get_param('~publish_tf_gt', False)
        self.publish_tf_noised = rospy.get_param('~publish_tf_noised', False)

        # Parameters for the noise model:
        # alpha_1: influence of the rotation on the rotation
        # alpha_2: influence of the translation on the rotation
        # alpha_3: influence of the translation on the translation
        # alpha_4: influence of the rotation on the translation
        self.alpha_1 = rospy.get_param('~alpha_1', 0.0)
        self.alpha_2 = rospy.get_param('~alpha_2', 0.0)
        self.alpha_3 = rospy.get_param('~alpha_3', 0.0)
        self.alpha_4 = rospy.get_param('~alpha_4', 0.0)

        # TF initialisation
        self.tf_broadcaster = tf2.TransformBroadcaster()

        # Init variables
        self.last_input_odom = None
        self.last_noised_odom = None

        # ROS publishers
        self.noised_publisher = rospy.Publisher('noised', Odometry, queue_size=1)
        self.ground_truth_publisher = rospy.Publisher('ground_truth', Odometry, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('input', Odometry, self.input_callback)

        # Dynamic reconfigure
        self.configServer = dynamic_reconfigure.server.Server(odometryPublisherConfig, self.dynamicReconfigure_callback)

        # ROS services
        rospy.Service('reset', Empty, self.reset_callback)


    def reset_callback(self, req):
        """ Service callback for reseting the noised odometry
        """

        self.last_input_odom = None
        return EmptyResponse()


    def input_callback(self, msg):
        """ Callback for the input odometry message
        """

        # Initialise the odometries if needed
        if self.last_input_odom is None:
            header = Header()
            header.stamp = rospy.get_rostime()
            header.frame_id = self.frame_id

            self.last_noised_odom = msg
            self.last_input_odom = msg

            return

        # Publish a new ground truth odometry (frame changed)
        odom = msg
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        v_x = odom.twist.twist.linear.x
        v_y = odom.twist.twist.linear.y
        v = sqrt(v_x**2 + v_y**2)

        theta = self.compute_angle_from_quaternion(odom.pose.pose.orientation)
        u_x = v*cos(theta)  # orientation vector
        u_y = v*sin(theta)
        dot_product = v_x*u_x + v_y*u_y
        v = copysign(v, dot_product)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0  # assuming no lateral motion

        self.ground_truth_publisher.publish(odom)

        # Compute deltas between last and current odoms
        dt = 1.0 / self.publish_frequency   # FIXME: the stamps published by Gazebo are not to be trusted
        last_theta = self.compute_angle_from_quaternion(self.last_input_odom.pose.pose.orientation)
        d_theta = theta - last_theta  # Care: it should be in [-pi, pi]

        d_d = sqrt(v_x**2 + v_y**2) * dt

        # Compute odometry model deltas
        d_trans = abs(d_d)
        sign_v = copysign(1, v)
        d_rot1 = theta + 0.5*(1-sign_v)*pi - last_theta  # don't use atan2(d_y, d_x) which is very noisy
        d_rot2 = d_theta - d_rot1

        # Noise these deltas
        if abs(d_trans) > 5*10**-3 or abs(d_theta) > 5*10**-3:
            d_rot1_mod = self.modulo_pi(d_rot1)  # prevent problems when going backwards
            d_rot2_mod = self.modulo_pi(d_rot2)

            sigma_d_trans = self.alpha_3*d_trans + self.alpha_4*(abs(d_rot1_mod)+abs(d_rot2_mod))
            sigma_d_rot1 = self.alpha_1*abs(d_rot1_mod) + self.alpha_2*d_trans
            sigma_d_rot2 = self.alpha_1*abs(d_rot2_mod) + self.alpha_2*d_trans

            d_rot1 += gauss(sigma_d_rot1, 0.5*sigma_d_rot1)
            d_rot2 += gauss(sigma_d_rot2, 0.5*sigma_d_rot2)
            d_trans += gauss(sigma_d_trans, 0.5*sigma_d_trans)

            rospy.logdebug("e_x={:.3} ; e_y={:.3} ; e_theta={:.3}".format(
                self.last_input_odom.pose.pose.position.x - self.last_noised_odom.pose.pose.position.x,
                self.last_input_odom.pose.pose.position.y - self.last_noised_odom.pose.pose.position.y,
                self.compute_angle_difference(
                    self.last_input_odom.pose.pose.orientation,
                    self.last_noised_odom.pose.pose.orientation
                    )
                ))
        else:
            noise_d_trans = 0.0
            sigma_d_rot1 = 0.0
            sigma_d_rot2 = 0.0
            sigma_d_trans = 0.0

        # Apply it to the pose and compute its covariance
        new_noised_odom = Odometry()
        last_noised_theta = self.compute_angle_from_quaternion(self.last_noised_odom.pose.pose.orientation)
        d_x = d_trans * cos(last_noised_theta + d_rot1)
        d_y = d_trans * sin(last_noised_theta + d_rot1)
        d_theta = d_rot1 + d_rot2
        new_noised_odom.pose.pose.position.x = self.last_noised_odom.pose.pose.position.x + d_x
        new_noised_odom.pose.pose.position.y = self.last_noised_odom.pose.pose.position.y + d_y
        new_noised_odom.pose.pose.orientation = self.compute_orientation_from_diff(
            self.last_noised_odom.pose.pose.orientation,
            d_theta
        )

        new_noised_odom.header.stamp = odom.header.stamp
        new_noised_odom.header.frame_id = self.frame_id
        new_noised_odom.child_frame_id = self.child_frame_id

        sigma_last_theta = sqrt(self.last_noised_odom.pose.covariance[35])
        sigma_d_x = abs(sigma_d_trans*cos(theta+d_rot1)) + abs(d_trans*sigma_last_theta*sin(theta+d_rot1))
        sigma_d_y = abs(sigma_d_trans*sin(theta+d_rot1)) + abs(d_trans*sigma_last_theta*cos(theta+d_rot1))
        sigma_d_theta = sigma_d_rot1 + sigma_d_rot2
        new_noised_odom.pose.covariance[0] = (sqrt(self.last_noised_odom.pose.covariance[0]) + sigma_d_x)**2
        new_noised_odom.pose.covariance[7] = (sqrt(self.last_noised_odom.pose.covariance[7]) + sigma_d_y)**2
        new_noised_odom.pose.covariance[35] = (sqrt(self.last_noised_odom.pose.covariance[35]) + sigma_d_theta)**2

        # Set linear speed in the right direction
        v_x = d_x / dt
        v_y = d_y / dt
        v = sqrt(v_x**2 + v_y**2)

        u_x = v*cos(last_noised_theta)  # orientation vector
        u_y = v*sin(last_noised_theta)
        dot_product = v_x*u_x + v_y*u_y
        v = copysign(v, dot_product)

        new_noised_odom.twist.twist.linear.x = v   # we are assuming no lateral motion
        new_noised_odom.twist.twist.linear.y = 0.0
        new_noised_odom.twist.twist.angular.z = d_theta / dt

        new_noised_odom.twist.covariance[0] = (sigma_d_x / dt)**2
        new_noised_odom.twist.covariance[7] = (sigma_d_y / dt)**2
        new_noised_odom.twist.covariance[35] = (sigma_d_theta / dt)**2

        # Publish noised odometry and transformations
        self.noised_publisher.publish(new_noised_odom)

        if self.publish_tf_gt:
            # Publish ground truth tf transform frame_id->child_frame_id
            pose = odom.pose.pose
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()   # FIXME: standard function to convert pose to transform?
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            self.tf_broadcaster.sendTransform(t)

        elif self.publish_tf_noised:
            # Publish ground truth tf transform frame_id->child_frame_id
            pose = new_noised_odom.pose.pose
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()   # FIXME: standard function to convert pose to transform?
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            self.tf_broadcaster.sendTransform(t)

        # Save the current odometries
        self.last_noised_odom = new_noised_odom
        self.last_input_odom = odom


    def compute_orientation_from_diff(self, last_orientation, d_theta):
        """ Add a given angle to an orientation

            Parameters:
                - last_orientation: last orientation (geometry_msgs/Quaternion)
                - d_theta: the difference of orientation between the last and the new odom
            Return:
                - new_orientation: the new orientation (geometry_msgs/Quaternion)
        """
        last_theta = tf.transformations.euler_from_quaternion([
                                                            last_orientation.x,
                                                            last_orientation.y,
                                                            last_orientation.z,
                                                            last_orientation.w
                                                            ])[2]
        new_theta = last_theta + d_theta
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, new_theta)
        new_orientation = Quaternion()
        new_orientation.x = quaternion[0]
        new_orientation.y = quaternion[1]
        new_orientation.z = quaternion[2]
        new_orientation.w = quaternion[3]

        return new_orientation


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


    def modulo_pi(self, angle):
        """ Bring an angle in [-pi/2, pi/2]
        """

        while (angle > pi/2) or (angle < -pi/2):
            if angle > pi/2:
                angle -= pi
            else:
                angle += pi

        return angle


    def dynamicReconfigure_callback(self, config, level):
        """ Dynamic reconfiguration of parameters
        """

        self.alpha_1 = config['alpha_1']
        self.alpha_2 = config['alpha_2']
        self.alpha_3 = config['alpha_3']
        self.alpha_4 = config['alpha_4']

        return config


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    OdometryPublisherNode()
    rospy.spin()
