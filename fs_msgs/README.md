# interface_messages
Package defining all messages being used as input or output

# Naming convention
Each message being added to this package starts with its dedicated subgroup and following by the package from which it is being published. So the active package defines its second naming characteristic and the final name explains its functionality.

E.g.:
nav_controller_pwm.msg : This message is being used by the Navigation subgroup in the controller package.

# Perception - Camera based

The raw input of the camera can be retrieved using *Image* messages from the *sensor_msgs* standard messages provided by ROS. The camera based perception module outputs a *BoundingBoxes* message, that consists of multiple *BoundingBox* messages. Each *BoundingBox* message indicates the position and class of a detected object, and also includes the confidence about the estimate.
