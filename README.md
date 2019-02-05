crazyflie_ros
=============

ROS stack for Bitcraze Crazyflie (http://www.bitcraze.se/), with the following features:

* Support for Crazyflie 1.0 and Crazyflie 2.0 (using stock firmware)
* Publishes on-board sensors in ROS standard message formats
* Supports ROS parameters to reconfigure crazyflie parameters
* Support for using multiple Crazyflies with a single Crazyradio
* Includes external controller for waypoint navigation (if motion capture system is available)
* No dependency to the Bitcraze SDK (Driver and Controller written in C++)

A tutorial (for a slightly older version) is available in W. Hönig and N. Ayanian. "Flying Multiple UAVs Using ROS", Chapter in Robot Operating System (ROS): The Complete Reference (Volume 2), Springer, 2017. (see http://act.usc.edu/publications.html for a free pre-print).

If you want to control many Crazyflies or look for a good controller for a single Crazyflie, take a look at http://crazyswarm.readthedocs.io/en/latest/. We are currently in the process to unify the Crazyswarm and crazyflie_ros as well as contributing the Crazyswarm firmware changes back to the official firmware.

## Citing This Work

This project is published under the very permissive MIT License. However,
if you use the package we appreciate if you credit this project accordingly.

For academic publications, you can cite the following book chapter:
```
@Inbook{crazyflieROS,
  author={Wolfgang H{\"o}nig
          and Nora Ayanian},
  editor={Anis Koubaa},
  title={Flying Multiple UAVs Using ROS},
  bookTitle={Robot Operating System (ROS): The Complete Reference  (Volume 2)},
  year={2017},
  publisher={Springer International Publishing},
  pages={83--118},
  isbn={978-3-319-54927-9},
  doi={10.1007/978-3-319-54927-9_3},
  url={https://doi.org/10.1007/978-3-319-54927-9_3}
}

```

If your work is related to Mixed Reality, you might cite the paper which introduced the package instead, using the following bibtex entry:
```
@conference{HoenigMixedReality2015,
  author = {Wolfgang H{\"o}nig and Christina Milanes and Lisa Scaria and Thai Phan and Mark Bolas and Nora Ayanian},
  booktitle = {IEEE/RSJ Intl Conf. Intelligent Robots and Systems},
  pages = {5382 - 5387},
  title = {Mixed Reality for Robotics},
  year = {2015}}
```

For any other mentioning please include my affiliation (ACTLab at University of Southern California or USC in short; The link to our webpage is http://act.usc.edu) as this work was partially done as part of my research at USC.

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
```

Use `catkin_make` on your workspace to compile.

## Usage

There are six packages included: crazyflie_cpp, crazyflie_driver, crazyflie_tools, crazyflie_description, crazyflie_controller, and crazyflie_demo.
Note that the below description might be slightly out-of-date, as we continue merging the Crazyswarm and crazyflie_ros.

### Crazyflie_Cpp

This package contains a cpp library for the Crazyradio and Crazyflie. It can be used independently of ROS.

### Crazyflie_driver

This package contains the driver. In order to support multiple Crazyflies with a single Crazyradio, there is crazyflie_server (communicating with all Crazyflies) and crazyflie_add to dynamically add Crazyflies.
The server does not communicate to any Crazyflie initially, hence crazyflie_add needs to be used.

### Crazyflie_tools

This package contains tools which are helpful, but not required for normal operation. So far, it just support one tool for scanning for a Crazyflie.

You can find connected Crazyflies using:
```
rosrun crazyflie_tools scan
```

### Crazyflie_description

This package contains a 3D model of the Crazyflie (1.0). This is for visualization purposes in rviz.

### Crazyflie_controller

This package contains a simple PID controller for hovering or waypoint navigation.
It can be used with external motion capture systems, such as VICON.

### Crazyflie_demo

This package contains a rich set of examples to get quickly started with the Crazyflie.

For teleoperation using a joystick, use:
```
roslaunch crazyflie_demo teleop_xbox360.launch uri:=radio://0/100/2M
```
where the uri specifies the uri of your Crazyflie. You can find valid uris using the scan command in the crazyflie_tools package.

For hovering at (0,0,1) using VICON, use:
```
roslaunch crazyflie_demo hover_vicon.launch uri:=radio://0/100/2M frame:=/vicon/crazyflie/crazyflie x:=0 y:=0 z:=1
```
where the uri specifies the uri of your Crazyflie and frame the tf-frame. The launch file runs vicon_bridge automatically.

For multiple Crazyflies make sure that all Crazyflies have a different address.
Crazyflies which share a dongle should use the same channel and datarate for best performance.
The performance degrades with the number of Crazyflies per dongle due to bandwidth limitations, however it was tested successfully to use 3 CFs per Crazyradio.
```
roslaunch crazyflie_demo multi_teleop_xbox360.launch uri1:=radio://0/100/2M/E7E7E7E7E7 uri2:=radio://0/100/2M/E7E7E7E705
```

Please check the launch files in the crazyflie_demo package for other examples, including simple waypoint navigation.

## ROS Features

### Parameters

The launch file supports the following arguments:
* uri: Specifier for the crazyflie, e.g. radio://0/80/2M
* tf_prefix: tf prefix for the crazyflie frame(s)
* roll_trim: Trim in degrees, e.g. negative if flie drifts to the left
* pitch_trim: Trim in degrees, e.g. negative if flie drifts forward

See http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks for details on how to obtain good trim values.

### Subscribers

#### cmd_vel

Similar to the hector_quadrotor, package the fields are used as following:
* linear.y: roll [e.g. -30 to 30 degrees]
* linear.x: pitch [e.g. -30 to 30 degrees]
* angular.z: yawrate [e.g. -200 to 200 degrees/second]
* linear.z: thrust [10000 to 60000 (mapped to PWM output)]

### Publishers

#### imu
* sensor_msgs/IMU
* contains the sensor readings of gyroscope and accelerometer
* The covariance matrices are set to unknown
* orientation is not set (this could be done by the magnetometer readings in the future.)
* update: 10ms (time between crazyflie and ROS not synchronized!)
* can be viewed in rviz

#### temperature
* sensor_msgs/Temperature
* From Barometer (10DOF version only) in degree Celcius (Sensor readings might be higher than expected because the PCB warms up; see http://www.bitcraze.se/2014/02/logging-and-parameter-frameworks-turtorial/)
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### magnetic_field
* sensor_msgs/MagneticField
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### pressure
* Float32
* hPa (or mbar)
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### battery
* Float32
* Volts
* update: 100ms (time between crazyflie and ROS not synchronized!)

## Similar Projects

* https://github.com/gtagency/crazyflie-ros
  * no documentation
  * no teleop
* https://github.com/omwdunkley/crazyflieROS
  * coupled with GUI
  * based on custom firmware
* https://github.com/mbeards/crazyflie-ros
  * incomplete
* https://github.com/utexas-air-fri/crazyflie_fly
  * not based on official SDK
  * no support for logging
* https://github.com/mchenryc/crazyflie
  * no documentation

## Notes

* The dynamic_reconfigure package (http://wiki.ros.org/dynamic_reconfigure/) seems like a good fit to map the parameters, however it has severe limitations:
  * Changed-Callback does not include which parameter(s) were changed. There is only a notion of a level which is a simple bitmask. This would cause that on any parameter change we would need to update all parameters on the Crazyflie.
  * Parameters are statically generated. There are hacks to add parameters at runtime, however those might not work with future versions of dynamic_reconfigure.
  * Groups not fully supported (https://github.com/ros-visualization/rqt_common_plugins/issues/162; This seems to be closed now, however the Indigo binary packages did not pick up the fixes yet).
