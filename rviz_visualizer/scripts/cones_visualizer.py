#!/usr/bin/env python
"""
    Visualisation of a list of cones in Rviz
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from fs_msgs.msg import Cones, Cone


class ConesVisualizerNode():
    def __init__(self):
        # ROS parameters
        publish_frequency = rospy.get_param('~publish_frequency', 0.0)
        marker_scale = rospy.get_param('~marker_scale', 0.0)
        marker_height = rospy.get_param('~marker_height', 0.0)
        namespace = rospy.get_param('~namespace', '')

        # Init variables
        self.last_cones = None

        # ROS publishers
        markers_publisher = rospy.Publisher('cone_markers', MarkerArray, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('cones', Cones, self.cones_callback)

        # Main loop
        self.rate = rospy.Rate(publish_frequency)
        while not rospy.is_shutdown():
            if self.last_cones is not None:
                markers_msg = MarkerArray()

                n = len(self.last_cones.cones)
                for k in range(n):
                    cone = self.last_cones.cones[k]  # FIXME: sometimes, index out of range (?!)
                    marker = Marker()

                    marker.ns = namespace
                    marker.id = k
                    marker.header.stamp = self.last_cones.header.stamp
                    marker.header.frame_id = self.last_cones.header.frame_id
                    marker.type = Marker.CYLINDER
                    marker.action = Marker.ADD

                    marker.scale.x = marker_scale
                    marker.scale.y = marker_scale
                    marker.scale.z = marker_height

                    marker.pose.position.x = cone.x
                    marker.pose.position.y = cone.y

                    marker.color.a = 1.0
                    if cone.color == Cone.BLUE:
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                    elif cone.color == Cone.YELLOW:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    elif cone.color == Cone.ORANGE:
                        marker.color.r = 1.0
                        marker.color.g = 0.5
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 0.5
                        marker.color.g = 0.5
                        marker.color.b = 0.5

                    marker.lifetime = rospy.Duration(1/publish_frequency)
                    marker.frame_locked = False    # FIXME: maybe not a good idea (?)

                    markers_msg.markers.append(marker)

                # Publish the marker array
                markers_publisher.publish(markers_msg)


            self.rate.sleep()


    def cones_callback(self, msg):
        """ Callback for the cones
        """

        self.last_cones = msg


if __name__ == '__main__':
    rospy.init_node('cones_visualizer')
    ConesVisualizerNode()
    rospy.spin()
