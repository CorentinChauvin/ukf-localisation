#!/usr/bin/env python
"""
    ddf
"""

from math import sqrt, sin, cos
import numpy as np
from copy import deepcopy
import scipy.linalg import sqrtm
import roslib
import rospy
import tf
import tf2_ros as tf2
import tf2_geometry_msgs
import dynamic_reconfigure.server
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from fs_msgs.msg import Cones, Cone
# TODO: import cfg file


class UKFNode():
    def __init__(self):
        # ROS parameters
        self.update_frequency = rospy.get_param('~update_frequency', 0.0)
        init_state = rospy.get_param('~init_state', [0.0]*6)
        init_covariance = rospy.get_param('~init_covariance', [0.0]*36)
        self.alpha = rospy.get_param('~alpha', 0.0)
        self.gamma = rospy.get_param('~gamma', 0.0)
        self.omega_decay = rospy.get_param('~omega_decay', 0.0)  # decay parameter for the imu angular speed update

        # Init variables
        self.odometry_available = False
        self.gnss_pose_available = False
        self.gnss_twist_available = False
        self.imu_available = False
        self.cones_available = False

        self.last_odom_msg = None
        self.last_imu_msg = None
        self.last_gnss_pose_msg = None
        self.last_gnss_twist_msg = None
        self.last_cones_msg = None

        self.dt = 1/float(self.update_frequency)

        # Initialise tf2
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # ROS publishers
        self.cones_detected_publisher = rospy.Publisher('cones_detected', Cones, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('odom', Odometry, self.odometry_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('gnss_pose', PoseWithCovarianceStamped, self.gnss_pose_callback)
        rospy.Subscriber('gnss_twist', TwistWithCovarianceStamped, self.gnss_twist_callback)
        rospy.Subscriber('cones', Cones, self.cones_callback)

        # Dynamic reconfigure
        # self.configServer = dynamic_reconfigure.server.Server(conesPerceptionConfig, self.dynamicReconfigure_callback)  # TODO

        # UKF initialisation
        self.mu = np.array(init_state)
        self.n = len(self.mu)
        self.covariance = np.array(init_covariance).reshape(self.n, self.n)
        self.lambda = self.alpha**2 * (self.n + self.gamma)
        self.eta = sqrt(self.n + self.lambda)
        self.Q = 0 # TODO
        # self.map = # TODO for data association

        # UKF weights
        self.w_m = [0.0]*(2*self.n+1)
        self.w_c = [0.0]*(2*self.n+1)
        self.w_m[0] = self.lambda/(self.n+self.lambda)
        self.w_c[0] = self.w_m[0] + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2*self.n+1):
            self.w_m[i] = 1 / (2*(self.n+self.lambda))
            self.w_c[i] = self.w_m[i]


        # Main loop
        self.rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            # Time update
            sigma_points = [self.mu]
            for i in range(n):
                matrix_squareroot = self.eta * sqrtm(self.covariance)
                sigma_points.append(self.mu + matrix_squareroot)
                sigma_points.append(self.mu - matrix_squareroot)

            sigma_points = predict(sigma_points)

            self.w_mu = np.array([0.0]*self.n)
            for i in range(2*self.n+1):
                neself.w_mu += self.w_m[i] * mu[i]
            self.mu = deepcopy(neself.w_mu)  # FIXME: be careful with the copy

            self.covariance = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)
            for i in range(2*n+1):
                self.covariance += self.w_c[i]*(sigma_points[i]-self.mu[i])*(sigma_points[i]-self.mu[i]).transpose() + self.Q

            # Measurement update
            if self.odometry_available:
                [Z, z, R] = self.odometry_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.odometry_available = False

            if self.gnss_pose_available:
                [Z, z, R] = self.gnss_pose_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.gnss_pose_available = False

            if self.gnss_twist_available:
                [Z, z, R] = self.gnss_twist_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.gnss_twist_available = False

            if self.cones_available:
                [Z, z, R] = self.cones_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.cones_available = False


    def predict(self, sigma_points):
        """ Propagate the evolution model to the sigma points

            Argument:
                - sigma_points: list of 2n+1 n-dimensional numpy vectors
            Returns:
                - sigma_points: the updated sigma_points
        """

        if self.last_imu_msg is None:
            return

        omega_imu = self.last_imu_msg.angular_velocity.z
        a_x = self.last_imu_msg.linear_acceleration.x
        a_y = self.last_imu_msg.linear_acceleration.y

        for i in range(self.n):
            x = sigma_points[i][0]
            y = sigma_points[i][1]
            theta = sigma_points[i][2]
            v_x = sigma_points[i][3]
            v_y = sigma_points[i][4]
            omega = sigma_points[i][5]

            sigma_points[i][0] += v_x * cos(theta) * self.dt
            sigma_points[i][1] += v_x * sin(theta) * self.dt
            sigma_points[i][2] += omega * self.dt
            sigma_points[i][3] += a_x * self.dt
            sigma_points[i][4] += a_y * self.dt
            sigma_points[i][5] += self.omega_decay*omega + (1-self.omega_decay)*omega_imu

        return sigma_points


    def measurement_update(self, Z, z, R, sigma_points):
        """ Update the estimate according to the given measurements

            Arguments:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
                - sigma_points: the sigma points
        """

        d = len(Z[0])  # size of a measurement

        z_hat = np.array([0.0]*d)
        for i in range(2*self.n+1):
            z_hat += self.w_m[i] * Z[i]

        S = R
        for i in range(2*self.n+1):
            S += w_c[i] * (Z[i]-z_hat) * ((Z[i]-z_hat).transpose())

        self.covariance = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)
        for i in range(2*self.n+1):
            self.covariance += self.w_i[i] * (sigma_points[i]-self.mu) * ((Z[i]-z_hat).transpose())

        K = self.covariance * np.linealg.inv(S)

        # Updating the estimate
        self.mu += K*(z-z_hat)
        self.covariance -= K*S*(K.transpose())


    def odometry_model(self, sigma_points):
        """ Provide the odometry measurement model for all the sigma points

            Argument:
                - sigma_points: the sigma points
            Returns:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
        """

        # Project measurement for all the sigma points
        Z = [0.0]*(2*self.n+1)
        for i in range(2*self.n+1):
            Z[i] = np.array([0.0, 0.0, 0.0,
                             sigma_points[i][4],
                             0.0,
                             sigma_points[i][6]])

        # Provide real measurement
        z = np.array([0.0, 0.0, 0.0,
                      self.last_odom_msg.twist.twist.linear.x,
                      0.0,
                      self.last_odom_msg.twist.twist.angular.z])

        # Build the measurement noise matrix
        R = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)
        R[3][3] = self.last_odom_msg.twist.covariance[0]
        R[5][5] = self.last_odom_msg.twist.covariance[35]

        return [Z, z, R]


    def gnss_pose_model(self, sigma_points):
        """ Provide the gnss pose measurement model for all the sigma points

            Argument:
                - sigma_points: the sigma points
            Returns:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
        """

        # Project measurement for all the sigma points
        Z = [0.0]*(2*self.n+1)
        for i in range(2*self.n+1):
            Z[i] = np.array([sigma_points[i][0], sigma_points[i][1], 0.0,
                             0.0, 0.0, 0.0])

        # Provide real measurement
        z = np.array([self.last_gnss_pose_msg.pose.pose.position.x,
                      self.last_gnss_pose_msg.pose.pose.position.y,
                      0.0, 0.0, 0.0, 0.0])

        # Build the measurement noise matrix
        R = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)
        R[0][0] = self.last_gnss_pose_msg.pose.covariance[0]
        R[1][1] = self.last_gnss_pose_msg.pose.covariance[7]

        return [Z, z, R]


    def gnss_twist_model(self, sigma_points):
        """ Provide the gnss twist measurement model for all the sigma points

            Argument:
                - sigma_points: the sigma points
            Returns:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
        """

        # Project measurement for all the sigma points
        Z = [0.0]*(2*self.n+1)
        for i in range(2*self.n+1):
            Z[i] = np.array([sigma_points[i][0], sigma_points[i][1], 0.0,
                             0.0, 0.0, 0.0])

        # Provide real measurement
        v_x = self.last_gnss_twist_msg.twist.twist.linear.x
        v_y = self.last_gnss_twist_msg.twist.twist.linear.y
        z = np.array([0.0, 0.0, atan2(v_y, v_x), v_x, v_y, 0.0])

        # Build the measurement noise matrix
        sigma_v_x = sqrt(self.last_gnss_twist_msg.twist.covariance[0])
        sigma_v_y = sqrt(self.last_gnss_twist_msg.twist.covariance[7])
        R = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)
        R[2][2] = (v_x*sigma_v_y)**2 + (v_y*sigma_v_x)**2)) / (v_x**2 + v_y**2)
        R[3][3] = sigma_v_x**2
        R[4][4] = sigma_v_y**2

        return [Z, z, R]


    def odometry_callback(self, msg):
        self.last_odom_msg = msg
        self.odometry_available = True


    def imu_callback(self, msg):
        self.last_imu_msg = msg
        self.imu_available = True


    def gnss_pose_callback(self, msg):
        self.last_gnss_pose_msg = msg
        self.gnss_pose_available = msg


    def gnss_twist_callback(self, msg):
        self.last_gnss_twist_msg = msg
        self.gnss_twist_available = msg


    def cones_callback(self, msg):
        self.last_cones_msg = msg
        self.cones_available = True









    # def dynamicReconfigure_callback(self, config, level):
    #     """ Dynamic reconfiguration of parameters
    #     """
    #
    #     self.std_position = config['std_position']
    #
    #     self.rate = rospy.Rate(self.publish_frequency)
    #
    #     return config


if __name__ == '__main__':
    rospy.init_node('ukf_localisation')
    UKFNode()
    rospy.spin()
