crazyflie_ros
=============

ROS Driver for Bitcraze Crazyflie (http://www.bitcraze.se/).
Unlike other solutions, it is based on the official python SDK (which needs to be installed/copied separately).

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/whoenig/crazyflie_ros.git
```

Additionally, you should have the bitcraze SDK on your file system.
See https://github.com/bitcraze/crazyflie-clients-python for details.

If you want to use joystick teleoperation, you should setup the hector_quadrotor package (http://wiki.ros.org/hector_quadrotor).

## Usage

You can use
```
roslaunch crazyflie_ros crazyflie.launch
```
to teleoperate the crazyflie.
Please note that there are (optional) arguments to change the device uri and path to the sdk.

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
This node contains the sensor readings of gyroscope and accelerometer.
The covariance matrices are set to unknown and the orientation is not set. (This could be done by the magnetometer readings in the future.)
The data is updated every 10ms, however the different time sources (ROS vs. crazyflie) are not synchronized, hence the time_stamp is the time when the packet was received.
The IMU data can be viewed in rviz.

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
