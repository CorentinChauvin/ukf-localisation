# Applied Estimation project

Description

## Install

Required packages:
* ros-kinetic-ackermann-msgs
* ros-kinetic-twist-mux
* ros-kinetic-joy
* ros-kinetic-controller-manager
* ros-kinetic-robotnik-msgs
* ros-kinetic-velodyne-simulator
* ros-kinetic-effort-controllers
* ros-kinetic-velocity-controllers
* ros-kinetic-joint-state-controller
* ros-kinetic-gazebo-ros-control
* ros-kinetic-teleop-twist-keyboard
* ros-kinetic-hector-gazebo-plugins
* python-catkin-tools
* python-numpy
* python-scipy


```
sudo apt-get install ros-kinetic-ackermann-msgs ros-kinetic-twist-mux ros-kinetic-joy ros-kinetic-controller-manager ros-kinetic-robotnik-msgs ros-kinetic-velodyne-simulator ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-joint-state-controller ros-kinetic-gazebo-ros-control ros-kinetic-robotnik-msgs ros-kinetic-teleop-twist-keyboard ros-kinetic-hector-gazebo-plugins python-catkin-tools python-numpy python-scipy
```

```
cd
mkdir -p project_ws/src
cd project_ws/src
git clone https://github.com/CorentinChauvin/ukf-localisation.git
catkin build
```

## Run the project
- To launch the simulation: `roslaunch eufs_gazebo small_track.launch`
- To control the car: `roslaunch robot_control keyboard_robot_control.launch`

## TODO

## Changelog
- 2018.12.10:
  - Add working Gazebo simulation
