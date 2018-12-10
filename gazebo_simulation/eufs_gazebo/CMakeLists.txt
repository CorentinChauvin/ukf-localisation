cmake_minimum_required(VERSION 2.8.3)
project(eufs_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  eufs_description
  robot_control
  roscpp
  std_msgs
  std_srvs
  tf
  dynamic_reconfigure
  fs_msgs
)

generate_dynamic_reconfigure_options(
  cfg/gpsPublisher.cfg
  cfg/odometryPublisher.cfg
  cfg/conesPerception.cfg
)

catkin_package()

install(PROGRAMS
  nodes/ground_truth_republisher.py
  nodes/gps_publisher.py
  nodes/odometry_publisher.py
  nodes/cones_perception.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY cfg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.cfg"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
