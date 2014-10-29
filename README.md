crazyflie_ros
=============

ROS Driver for Bitcraze Crazyflie (http://www.bitcraze.se/), with the following features:

* Uses the official python SDK (which needs to be installed/copied separately)
* Publishes on-board sensors in ROS standard message formats
* Supports ROS parameters to reconfigure crazyflie parameters

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/whoenig/crazyflie_ros.git
```

Additionally, you should have the bitcraze SDK on your file system.
See https://github.com/bitcraze/crazyflie-clients-python for details.

If you want to use joystick teleoperation, you should setup the hector_quadrotor package (http://wiki.ros.org/hector_quadrotor).

## Usage

There are two packages included: crazyflie and crazyflie_demo.
Crazyflie contains the driver and a launch file which can be used in your own projects.

Crazyflie_demo contains a small demo for teleoperating the crazyflie with a joystick and visualizing the sensor data in rviz.

You can use
```
roslaunch crazyflie_demo teleop_xbox360.launch
```
to teleoperate the crazyflie.
Please note that there are (optional) arguments to change the device uri and path to the sdk.
See `crazyflie_demo/launch/teleop_xbox360.launch` for details.

## ROS Features

### Subscribers

#### cmd_vel

Similar to the hector_quadrotor, package the fields are used as following:
* linear.y: roll [e.g. -30 to 30]
* linear.x: pitch [e.g. -30 to 30]
* angular.z: yawrate [e.g. -200 to 200]
* linear.z: thrust [10000 to 60000]

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
