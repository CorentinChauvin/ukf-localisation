#!/usr/bin/env python
"""
    Fake the perception system and give the position and color of the cones
    which are in a given radius around the car (field of view: 180 degrees)

    Note: when getting the pose of the objects from Gazebo, it doesn't correspond
    to tf frames
"""

from math import sqrt
import numpy
import roslib
import rospy
import tf
import tf2_ros as tf2
import tf2_geometry_msgs
import dynamic_reconfigure.server
from nav_msgs.msg import Odometry
from fs_msgs.msg import Cones, Cone
from gazebo_msgs.msg import LinkStates
from eufs_gazebo.cfg import conesPerceptionConfig


class ConesPerceptionNode():
    def __init__(self):
        # ROS parameters
        self.std_position = rospy.get_param('~std_position', 0.0)                       # Constant factor in the standard deviation
        self.std_position_linear = rospy.get_param('~std_position_linear', 0.0)         # Linear factor in the standard deviation
        self.std_position_quadratic = rospy.get_param('~std_position_quadratic', 0.0)   # Quadratic factor in the standard deviation
        self.range = rospy.get_param('~range', 0.0)  # Range of the sensor
        self.sensor_frame = rospy.get_param('~sensor_frame', '')
        self.absolute_frame = rospy.get_param('~absolute_frame', '')
        self.publish_frequency = rospy.get_param('~publish_frequency', 0.0)
        self.display_undetected_cones = rospy.get_param('~display_undetected_cones', False)

        # Init variables
        self.cones_list = []    # List of all the cones on the map
        self.range_squared = self.range**2
        self.distance_sensor = None
        self.last_sensor_transform = None  # Transform world->sensor (matrix)
        self.last_gazebo_message = None

        # Initialise tf2
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # ROS publishers
        self.cones_detected_publisher = rospy.Publisher('cones_detected', Cones, queue_size=1)
        self.cones_detected_absolute_publisher = rospy.Publisher('cones_detected_absolute', Cones, queue_size=1)
        self.cones_undetected_publisher = rospy.Publisher('cones_undetected', Cones, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('gazebo_links', LinkStates, self.gazebo_callback)

        # Dynamic reconfigure
        self.configServer = dynamic_reconfigure.server.Server(conesPerceptionConfig, self.dynamicReconfigure_callback)


        # Main loop
        self.rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            if self.last_gazebo_message is not None:
                self.process_gazebo_message(self.last_gazebo_message)

            if self.cones_list != [] and self.last_sensor_transform is not None:
                detected_cones = Cones()
                detected_cones.header.stamp = rospy.get_rostime()
                detected_cones.header.frame_id = self.sensor_frame
                detected_cones_absolute = Cones()  # Real position of the detected cones in the absolute frame for data association
                detected_cones.header.stamp = rospy.get_rostime()
                detected_cones.header.frame_id = self.absolute_frame
                undetected_cones = Cones()
                undetected_cones.header.stamp = rospy.get_rostime()
                undetected_cones.header.frame_id = self.sensor_frame

                for cone in self.cones_list:
                    # Transform the cone in the sensor frame
                    cone_transformed = Cone()
                    pose_cone_transformed = numpy.dot(
                        tf.transformations.inverse_matrix(self.last_sensor_transform),
                        [cone.x, cone.y, 0.0, 1.0]
                    )
                    cone_transformed.x = pose_cone_transformed[0]
                    cone_transformed.y = pose_cone_transformed[1]

                    # Select the cones that are in the field of view and in range
                    distance_squared = cone_transformed.x**2 + cone_transformed.y**2
                    if (cone_transformed.x >= 0) and (distance_squared <= self.range_squared):
                        cone_transformed.color = cone.color
                        cone_transformed.probability = 1.0  # TODO

                        # Compute standard deviation
                        std_deviation = (
                            self.std_position
                            + self.std_position_linear*sqrt(distance_squared)
                            + self.std_position_quadratic*(distance_squared**2)
                            )
                        cone_transformed.covariance[0] = std_deviation**2
                        cone_transformed.covariance[2] = std_deviation**2

                        # Noise the position
                        cone_transformed.x += numpy.random.normal(0.0, std_deviation)
                        cone_transformed.y += numpy.random.normal(0.0, std_deviation)

                        detected_cones.cones.append(cone_transformed)

                        # Add the initial cone in the absolute frame
                        detected_cones_absolute.cones.append(cone)

                    elif self.display_undetected_cones:
                        cone_transformed.color = Cone.UNDEFINED
                        undetected_cones.cones.append(cone_transformed)

                # Display the position of the car as a cone (FIXME: to remove?)
                if self.display_undetected_cones:
                    car_pos = Cone()
                    car_pos.x = 0.0
                    car_pos.y = 0.0
                    car_pos.color = Cone.ORANGE
                    undetected_cones.cones.append(car_pos)

                # Publish the cones
                self.cones_detected_publisher.publish(detected_cones)
                self.cones_detected_absolute_publisher.publish(detected_cones_absolute)
                self.cones_undetected_publisher.publish(undetected_cones)

            self.rate.sleep()


    def gazebo_callback(self, msg):
        """ Callback for Gazebo links

            Since the frequency is very high, we only store the message. The
            processing is done at a lower frequency.
        """

        self.last_gazebo_message = msg


    def process_gazebo_message(self, msg):
        """ Process the Gazebo links message (the cones and the car in particular)

            The list of cones is only built once.
            We don't want to call it at each Gazebo callback (1000Hz).
        """

        # Get the distance between base_footprint and the sensor
        if self.distance_sensor is None:
            try:
                transform = self.tf_buffer.lookup_transform('base_footprint', self.sensor_frame, rospy.Time(0))
                self.distance_sensor = transform.transform.translation.x
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                return

        # Compute the transform world->sensor
        n = len(msg.name)
        for k in range(n):
            if 'base_footprint' in msg.name[k]:
                # Get the world->base_footprint transform
                base_pose = msg.pose[k]
                trans_mat = tf.transformations.translation_matrix([
                                    base_pose.position.x,
                                    base_pose.position.y,
                                    0.0
                                ])
                rot_mat = tf.transformations.quaternion_matrix([
                                    base_pose.orientation.x,
                                    base_pose.orientation.y,
                                    base_pose.orientation.z,
                                    base_pose.orientation.w,
                                ])
                # transform1 = numpy.dot(trans_mat, tf.transformations.inverse_matrix(rot_mat))
                transform1 = numpy.dot(trans_mat, rot_mat)

                # Compute the world->sensor transform
                transform2 = tf.transformations.translation_matrix([
                                    self.distance_sensor,
                                    0.0,
                                    0.0
                                ])
                self.last_sensor_transform = numpy.dot(transform1, transform2)

        # Get the cones (in the world frame)
        if self.cones_list == []:
            for k in range(n):
                name = msg.name[k]
                if 'cone' in name:
                    new_cone = Cone()
                    new_cone.x = msg.pose[k].position.x
                    new_cone.y = msg.pose[k].position.y

                    if 'blue' in name:
                        new_cone.color = Cone.BLUE
                    elif 'yellow' in name:
                        new_cone.color = Cone.YELLOW
                    elif 'orange' in name:
                        new_cone.color = Cone.ORANGE
                    else:
                        new_cone.color = Cone.UNDEFINED

                    self.cones_list.append(new_cone)


    def dynamicReconfigure_callback(self, config, level):
        """ Dynamic reconfiguration of parameters
        """

        self.std_position = config['std_position']
        self.std_position_linear = config['std_position_linear']
        self.std_position_quadratic = config['std_position_quadratic']
        self.range = config['range']
        self.range_squared = self.range**2
        self.publish_frequency = config['publish_frequency']
        self.display_undetected_cones = config['display_undetected_cones']

        self.rate = rospy.Rate(self.publish_frequency)

        return config


if __name__ == '__main__':
    rospy.init_node('cones_perception')
    ConesPerceptionNode()
    rospy.spin()
