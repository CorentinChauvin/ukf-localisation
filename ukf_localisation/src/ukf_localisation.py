#!/usr/bin/env python
"""
    UKF algorithm for localisation of the car

    Corentin Chauvin-Hameau - 2018-2019
"""

from math import sqrt, sin, cos, atan2, pi
import numpy as np
from copy import deepcopy
from scipy.linalg import sqrtm
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
        self.frame_id = rospy.get_param('~frame_id', '')
        self.child_frame_id = rospy.get_param('~child_frame_id', '')
        self.cones_sensor_frame = rospy.get_param('~cones_sensor_frame', 0.0)

        self.alpha = rospy.get_param('~alpha', 0.0)
        self.beta = rospy.get_param('~beta', 0.0)
        self.gamma = rospy.get_param('~gamma', 0.0)
        self.omega_decay = rospy.get_param('~omega_decay', 0.0)  # decay parameter for the imu angular speed update

        init_with_topic = rospy.get_param('~init_with_topic', False)
        init_state = rospy.get_param('~init_state', [0.0]*6)
        init_covariance = rospy.get_param('~init_covariance', [0.0]*36)
        process_noise_covariance = rospy.get_param('~process_noise_covariance', [0.0]*36)
        gnss_init_time = rospy.get_param('~gnss_init_time', 0.0)
        self.std_position_cones = rospy.get_param('~std_position_cones', 0.0)
        self.std_position_gnss = rospy.get_param('~std_position_gnss', 0.0)
        min_gnss_speed = rospy.get_param('~min_gnss_speed', 0.0)

        fuse_odometry = rospy.get_param('~fuse_odometry', True)
        fuse_gnss_position = rospy.get_param('~fuse_gnss_position', True)
        fuse_gnss_speed = rospy.get_param('~fuse_gnss_speed', True)
        fuse_cones = rospy.get_param('~fuse_cones', True)
        self.use_lateral_speed = rospy.get_param('~use_lateral_speed', True)


        # Init variables
        self.odometry_available = False
        self.gnss_pose_available = False
        self.gnss_twist_available = False
        self.imu_available = False
        self.cones_available = False
        self.cones_absolute_available = False

        self.last_odom_msg = None
        self.last_imu_msg = None
        self.last_gnss_pose_msg = None
        self.last_gnss_twist_msg = None
        self.last_cones_msg = None
        self.last_cones_absolute_msg = None

        self.is_gnss_initialised = None

        self.dt = 1/float(self.update_frequency)
        self.min_cov_value = 10.0**-15  # minimal diagonal covariance value
        self.distance_sensor = None     # distance between base_footprint and the cones sensor

        # Initialise tf2
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # ROS publishers
        self.output_publisher = rospy.Publisher('output', Odometry, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('odom', Odometry, self.odometry_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('gnss_pose', PoseWithCovarianceStamped, self.gnss_pose_callback)
        rospy.Subscriber('gnss_twist', TwistWithCovarianceStamped, self.gnss_twist_callback)
        rospy.Subscriber('cones', Cones, self.cones_callback)
        rospy.Subscriber('cones_absolute', Cones, self.cones_absolute_callback)
        if init_with_topic:
            self.mu = None
            self.sub_init_topic = rospy.Subscriber('init_topic', Odometry, self.init_topic_callback)

        # Dynamic reconfigure
        # self.configServer = dynamic_reconfigure.server.Server(conesPerceptionConfig, self.dynamicReconfigure_callback)  # TODO

        # UKF state initialisation
        if init_with_topic:
            rospy.logdebug("Waiting for initialisation...")
            while self.mu is None:
                rospy.Rate(5).sleep()
            rospy.logdebug("mu initialised: {}".format(self.mu))
        else:
            self.mu = np.array(init_state)
        self.n = len(self.mu)

        # GNSS pose offsets estimation
        self.gnss_offset_x = 0.0
        self.gnss_offset_y = 0.0
        self.gnss_init_counter = 0
        self.is_gnss_initialised = False
        rospy.loginfo("Initialise GNSS offsets...")
        rospy.sleep(gnss_init_time)
        self.gnss_offset_x = self.gnss_offset_x/self.gnss_init_counter - self.mu[0]
        self.gnss_offset_y = self.gnss_offset_y/self.gnss_init_counter - self.mu[1]
        self.is_gnss_initialised = True
        rospy.loginfo("GNSS initialised")

        # Get the distance between base_footprint and the cones sensor
        # (assumes that the sensor is mounted in front of the car aligned with the x axis)
        while self.distance_sensor is None:
            try:
                transform = self.tf_buffer.lookup_transform(self.child_frame_id, self.cones_sensor_frame, rospy.Time(0))
                self.distance_sensor = transform.transform.translation.x
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                rospy.sleep(0.2)

        # Initialise miscellaneous UKF parameters
        self.covariance = np.array(init_covariance).reshape(self.n, self.n)
        self.ko = self.alpha**2 * (self.n + self.gamma) - self.n
        self.eta = sqrt(self.n + self.ko)
        self.Q = np.array(process_noise_covariance).reshape(self.n, self.n)
        # self.map = # TODO for data association

        # UKF weights
        self.w_m = [0.0]*(2*self.n+1)
        self.w_c = [0.0]*(2*self.n+1)
        self.w_m[0] = self.ko/(self.n+self.ko)
        self.w_c[0] = self.w_m[0] + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2*self.n+1):
            self.w_m[i] = 1 / (2*(self.n+self.ko))
            self.w_c[i] = self.w_m[i]

        np.set_printoptions(precision=4, suppress=True)


        # MAIN LOOP
        rate = rospy.Rate(self.update_frequency)
        while not rospy.is_shutdown():
            rospy.logdebug("-----*--*-*-*")
            rospy.logdebug("mu: {}".format(self.mu))
            rospy.logdebug("covariance:\n {}".format(self.covariance))

            # Time update
            sigma_points = self.sample_sigma_points()
            sigma_points = deepcopy(self.predict(sigma_points))

            mu = np.array([0.0]*self.n)
            for i in range(2*self.n+1):
                mu += self.w_m[i] * sigma_points[i]
            self.mu = deepcopy(mu)

            self.covariance = deepcopy(self.Q)
            for i in range(2*self.n+1):
                diff = np.matrix(sigma_points[i]-self.mu).transpose()  # column vector
                self.covariance += self.w_c[i] * np.matmul(diff, diff.transpose())

            # Measurement update
            if self.odometry_available and fuse_odometry:
                sigma_points = self.sample_sigma_points()
                [Z, z, R] = self.odometry_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.odometry_available = False

            if self.gnss_pose_available and fuse_gnss_position:
                sigma_points = self.sample_sigma_points()
                [Z, z, R] = self.gnss_pose_model(sigma_points)
                self.measurement_update(Z, z, R, sigma_points)
                self.gnss_pose_available = False

            if self.gnss_twist_available and fuse_gnss_speed:
                sigma_points = self.sample_sigma_points()
                v_x = self.last_gnss_twist_msg.twist.twist.linear.x
                v_y = self.last_gnss_twist_msg.twist.twist.linear.y
                v = sqrt(v_x**2 + v_y**2)

                if v >= min_gnss_speed:
                    [Z, z, R] = self.gnss_twist_model(sigma_points)
                    self.measurement_update(Z, z, R, sigma_points)
                    self.gnss_twist_available = False

            if self.cones_available and self.cones_absolute_available and fuse_cones:
                cones = self.last_cones_msg.cones
                cones_absolute = self.last_cones_absolute_msg.cones

                if self.last_cones_msg.header.stamp == self.last_cones_absolute_msg.header.stamp:
                    # One update for each cone measurement
                    for k in range(len(cones)):
                        sigma_points = self.sample_sigma_points()
                        cone = deepcopy(cones[k])
                        cone.x += self.distance_sensor
                        [Z, z, R] = self.cones_model(sigma_points, cone, cones_absolute[k])
                        self.measurement_update(Z, z, R, sigma_points,
                                mask_correction=[True, True, False, False, False, False])

                    self.cones_available = False
                    self.cones_absolute_available = False

            # Publish odometry output
            self.publish_output()

            rate.sleep()


    def sample_sigma_points(self):
        """ Compute new sigma points

            Returns:
                - sigma_points: the new sigma points
        """

        sigma_points = [self.mu]
        matrix_squareroot = self.eta * sqrtm(self.covariance)
        for i in range(self.n):
            sigma_points.append(self.mu + matrix_squareroot[i, :])
            sigma_points.append(self.mu - matrix_squareroot[i, :])

        return sigma_points


    def predict(self, sigma_points):
        """ Propagate the evolution model to the sigma points

            Argument:
                - sigma_points: list of 2n+1 n-dimensional numpy vectors
            Returns:
                - sigma_points: the updated sigma_points
        """

        if self.last_imu_msg is None:
            return sigma_points

        omega_imu = self.last_imu_msg.angular_velocity.z
        a_x = self.last_imu_msg.linear_acceleration.x
        a_y = self.last_imu_msg.linear_acceleration.y

        for i in range(2*self.n+1):
            x = sigma_points[i][0]
            y = sigma_points[i][1]
            theta = sigma_points[i][2]
            v_x = sigma_points[i][3]
            if self.use_lateral_speed:
                v_y = sigma_points[i][4]
            else:
                v_y = 0.0
            omega = sigma_points[i][5]

            sigma_points[i][0] += (v_x*cos(theta) - v_y*sin(theta)) * self.dt
            sigma_points[i][1] += (v_x*sin(theta) + v_y*cos(theta)) * self.dt
            sigma_points[i][2] += omega * self.dt
            sigma_points[i][3] += a_x * self.dt
            if self.use_lateral_speed:
                sigma_points[i][4] += a_y * self.dt
            else:
                sigma_points[i][4] = 0.0
            sigma_points[i][5] = self.omega_decay*omega + (1-self.omega_decay)*omega_imu

        return sigma_points


    def measurement_update(self, Z, z, R, sigma_points, mask_correction=None):
        """ Update the estimate according to the given measurements

            Arguments:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
                - sigma_points: the sigma points
                - mask_correction: dimensions to fuse in the corrections (list of booleans)
        """

        if mask_correction is None:
            mask_correction = [True]*self.n
        mask_correction = np.array(mask_correction)

        d = len(Z[0])  # size of a measurement

        z_hat = np.array([0.0]*d)
        for i in range(2*self.n+1):
            z_hat += self.w_m[i] * Z[i]

        S = R
        for i in range(2*self.n+1):
            diff = np.matrix(Z[i]-z_hat).transpose()  # column vector
            diff[2, 0] = self.modulo_2pi(diff[2, 0])
            S += self.w_c[i] * np.matmul(diff, diff.transpose())

        covariance_z = np.array([0.0]*(self.n**2)).reshape(self.n, self.n)  # not a proper symetrical covariance matrix
        for i in range(2*self.n+1):
            diff = np.matrix(Z[i]-z_hat)
            diff[0, 2] = self.modulo_2pi(diff[0, 2])
            covariance_z += self.w_c[i] * np.matmul(
                                                    np.matrix(sigma_points[i]-self.mu).transpose(),
                                                    diff
                                                    )

        K = np.matmul(covariance_z, np.linalg.inv(S))

        # Updating the estimate
        diff = z-z_hat
        diff[2] = self.modulo_2pi(diff[2])
        self.mu += np.multiply(np.dot(K, diff), mask_correction)
        self.covariance -= K.dot(S).dot(K.transpose())


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
                             sigma_points[i][3],
                             0.0,
                             sigma_points[i][5]])

        # Provide real measurement
        z = np.array([0.0, 0.0, 0.0,
                      self.last_odom_msg.twist.twist.linear.x,
                      0.0,
                      self.last_odom_msg.twist.twist.angular.z])

        # Build the measurement noise matrix
        R = self.initialise_cov_matrix(self.n, 1/self.min_cov_value)
        R[3][3] = max(self.last_odom_msg.twist.covariance[0], self.min_cov_value)
        R[5][5] = max(self.last_odom_msg.twist.covariance[35], self.min_cov_value)

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
        R = self.initialise_cov_matrix(self.n, 1/self.min_cov_value)
        if self.std_position_gnss >= self.min_cov_value:
            R[0][0] = self.std_position_gnss**2
            R[1][1] = self.std_position_gnss**2
        else:
            R[0][0] = max(self.last_gnss_pose_msg.pose.covariance[0], self.min_cov_value)
            R[1][1] = max(self.last_gnss_pose_msg.pose.covariance[7], self.min_cov_value)

        return [Z, z, R]


    def gnss_twist_model(self, sigma_points):
        """ Provide the gnss twist measurement model for all the sigma points
            Fuse the GNSS speed only above a minimum speed

            Argument:
                - sigma_points: the sigma points
            Returns:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
        """

        v_x = self.last_gnss_twist_msg.twist.twist.linear.x
        v_y = self.last_gnss_twist_msg.twist.twist.linear.y
        v = sqrt(v_x**2 + v_y**2)
        track = atan2(v_y, v_x)
        (v_x, v_y) = (cos(track)*v_x + sin(track)*v_y,
                      -sin(track)*v_x + cos(track)*v_y)

        # Project measurement for all the sigma points
        Z = [0.0]*(2*self.n+1)
        for i in range(2*self.n+1):
            Z[i] = np.array([0.0, 0.0, sigma_points[i][2], sigma_points[i][3],
                             sigma_points[i][4], 0.0])

        # Provide real measurement
        z = np.array([0.0, 0.0, track, v_x, v_y, 0.0])

        # Build the measurement noise matrix
        sigma_v_x = sqrt(self.last_gnss_twist_msg.twist.covariance[0])
        sigma_v_y = sqrt(self.last_gnss_twist_msg.twist.covariance[7])
        R = self.initialise_cov_matrix(self.n, 1/self.min_cov_value)
        R[2][2] = max(
            ((v_x*sigma_v_y)**2 + (v_y*sigma_v_x)**2) / (v_x**2 + v_y**2),
            self.min_cov_value)
        # TODO: project along the track (useless if sigma_v_x=sigma_v_y)
        R[3][3] = max(sigma_v_x**2, self.min_cov_value)
        R[4][4] = max(sigma_v_y**2, self.min_cov_value)

        return [Z, z, R]


    def cones_model(self, sigma_points, cone, cone_absolute):
        """ Provide the cone measurement model for all the sigma points

            An update is done for each cone sequentially, so only one cone is
            considered here.

            Argument:
                - sigma_points: the sigma points
                - cone: the cone measurement (relative to the car)
                - cone_absolute: the corresponding absolute cone position
            Returns:
                - Z: projected measurement for all the sigma points
                - z: real measurement
                - R: measurement noise matrix
        """

        # Project measurement for all the sigma points
        Z = [0.0]*(2*self.n+1)
        for i in range(2*self.n+1):
            Z[i] = np.array([sigma_points[i][0], sigma_points[i][1],
                             0.0, 0.0, 0.0, 0.0])

        # Provide real measurement
        x_m = cone.x
        y_m = cone.y
        theta = self.mu[2]
        z = np.array([cone_absolute.x - x_m*cos(theta) + y_m*sin(theta),
                      cone_absolute.y - x_m*sin(theta) - y_m*cos(theta),
                      0.0, 0.0, 0.0, 0.0])

        # Build the measurement noise matrix
        R = self.initialise_cov_matrix(self.n, 1/self.min_cov_value)
        if self.std_position_cones >= self.min_cov_value:
            R[0][0] = self.std_position_cones**2
            R[1][1] = self.std_position_cones**2
        else:
            R[0][0] = max(cone.covariance[0], self.min_cov_value)
            R[1][1] = max(cone.covariance[2], self.min_cov_value)

        return [Z, z, R]


    def odometry_callback(self, msg):
        self.last_odom_msg = msg
        self.odometry_available = True


    def imu_callback(self, msg):
        self.last_imu_msg = msg
        self.imu_available = True


    def gnss_pose_callback(self, msg):
        if self.is_gnss_initialised is None:
            return  # to early to start the initialisation
        elif not self.is_gnss_initialised:
            # Average the initial position
            self.gnss_offset_x += msg.pose.pose.position.x
            self.gnss_offset_y += msg.pose.pose.position.y
            self.gnss_init_counter += 1
        else:
            # Remove the offset and release the message
            self.last_gnss_pose_msg = msg
            self.last_gnss_pose_msg.pose.pose.position.x -= self.gnss_offset_x
            self.last_gnss_pose_msg.pose.pose.position.y -= self.gnss_offset_y
            self.gnss_pose_available = True


    def gnss_twist_callback(self, msg):
        self.last_gnss_twist_msg = msg
        self.gnss_twist_available = True


    def cones_callback(self, msg):
        self.last_cones_msg = msg
        self.cones_available = True


    def cones_absolute_callback(self, msg):
        self.last_cones_absolute_msg = msg
        self.cones_absolute_available = True


    def init_topic_callback(self, msg):
        """ Callback for the initialisation topic
            Initialise the state with an odometry topic
        """

        if self.mu is None:
            self.mu = np.array([0.0]*6)

            self.mu[0] = msg.pose.pose.position.x
            self.mu[1] = msg.pose.pose.position.y

            orientation = msg.pose.pose.orientation
            self.mu[2] = tf.transformations.euler_from_quaternion([
                                                            orientation.x,
                                                            orientation.y,
                                                            orientation.z,
                                                            orientation.w
                                                            ])[2]

            # Unsubscribe to the topic
            self.sub_init_topic.unregister()


    def initialise_cov_matrix(self, n, diag_value):
        """ Create a n by n covariance matrix

            Arguments:
                - n: dimension of the covariance matrix
                - diag_value: diagonal terms
            Returns:
                - a n by n numpy array
        """

        M = np.array([0.0]*(n**2)).reshape(n, n)
        for k in range(n):
            M[k][k] = diag_value

        return M


    def publish_output(self):
        """ Build odometry output and publish it
        """

        msg = Odometry()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.mu[0]
        msg.pose.pose.position.y = self.mu[1]

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.mu[2])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        msg.pose.covariance[0] = self.covariance[0][0]
        msg.pose.covariance[7] = self.covariance[1][1]
        msg.pose.covariance[35] = self.covariance[2][2]

        msg.twist.twist.linear.x = self.mu[3]
        msg.twist.twist.linear.y = self.mu[4]
        msg.twist.twist.angular.z = self.mu[5]

        msg.twist.covariance[0] = self.covariance[3][3]
        msg.twist.covariance[7] = self.covariance[4][4]
        msg.twist.covariance[35] = self.covariance[5][5]

        self.output_publisher.publish(msg)


    def print_sigma_points(self, sigma_points):
        """ Print the provided sigma points
        """

        n = len(sigma_points)
        for i in range(n):
            print("sigma_points[{}]: {}".format(i, sigma_points[i]))


    def modulo_2pi(self, angle):
        """ Bring an angle in [-pi, pi]
        """

        while (angle > pi) or (angle < -pi):
            if angle > pi:
                angle -= 2*pi
            else:
                angle += 2*pi

        return angle

    # TODO
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
