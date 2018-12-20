# EUFS Autonomous Simulation

ROS/Gazebo simulation packages for KTH Formula Student driverless.

It is highly based on the EUFS code: https://github.com/eufsa/eufs_sim

![simulation](http://eufs.co/wp-content/uploads/2018/05/eufs-sim.jpg)

### Contents
1. [Install Prerequisites](#requirements)
2. [Compiling and running](#compiling)
3. [Sensors](#sensors)

## Setup Instructions
### 1. Install Prerequisites <a name="requirements"></a>
##### - Install Ubuntu 16.04 LTS
##### - Install [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation)
##### - Install ROS packages:
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

Or if you are lazy like me, here's a one-liner
```
sudo apt-get install ros-kinetic-ackermann-msgs ros-kinetic-twist-mux ros-kinetic-joy ros-kinetic-controller-manager ros-kinetic-robotnik-msgs ros-kinetic-velodyne-simulator ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-joint-state-controller ros-kinetic-gazebo-ros-control ros-kinetic-robotnik-msgs ros-kinetic-teleop-twist-keyboard ros-kinetic-hector-gazebo-plugins
```


### 2. Compiling and running <a name="compiling"></a>

Create a workspace for the simulation if you don't have one:
```mkdir -p ~/ros/eufs_ws/src```
Copy the contents of this repository to the `src` folder you just created.

Navigate to your workspace and build the simulation:
```
cd ~/ros/eufs_ws
catkin_make
```
_Note:_ You can use `catkin build` instead of `catkin_make` if you know what you are doing.

To enable ROS to find the EUFS packages you also need to run
```source /devel/setup.bash```
_Note:_ source needs to be run on each new terminal you open. You can also include it in your `.bashrc` file.

Now you can finally run our kickass simulation!!
```roslaunch eufs_gazebo small_track.launch```

Easy ways to control the car is via:
  - ```roslaunch robot_control rqt_robot_control.launch``` (GUI)
  - ```roslaunch robot_control keyboard_robot_control.launch``` (keyboard)

### 3. Additional sensors <a name="sensors"></a>
Additional sensors for testing are avilable via the `ros-kinetic-robotnik-sensor` package. Some of them are already defined in `eufs_description/robots/eufs.urdf.xarco`. You can simply commment them in and attach them appropriately to the car.


**Sensor suit of the car by default:**

* VLP16 lidar
* ZED Stereo camera
* IMU
* GPS
* odometry

---

## TODO
- Move the sensors in the Gazebo description model
- Generate outliers for the perception (color and position)
- Investigate delay of perception in Rviz
- Remove display of car as a cone (in undetected cones) (?)
- Move the *geodetic_utils* call in the SLAM package

## Changelog
- 2018.12.14 (Corentin Chauvin-Hameau, corch@kth.se)
  - Fix odometries (based on Gazebo instead of on the controllers)
- 2018.12.08 (Corentin Chauvin-Hameau, corch@kth.se)
  - Noise odometry
- 2018.12.07 (Corentin Chauvin-Hameau, corch@kth.se)
  - Handle odometry
  - Publish detected cones
- 2018.12.06 (Corentin Chauvin-Hameau, corch@kth.se)
  - Keyboard teleop
  - Prepare GPS output for filtering
