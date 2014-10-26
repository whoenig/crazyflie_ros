#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Float32

import time, sys
import math
from threading import Thread

class CrazyflieROS:
    Disconnected = 0
    Connecting = 1
    Connected = 2

    """Wrapper between ROS and Crazyflie SDK"""
    def __init__(self, link_uri, tf_prefix):
        self.link_uri = link_uri
        self.tf_prefix = tf_prefix
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cmdVel = Twist()
        rospy.Subscriber("cmd_vel", Twist, self._cmdVelChanged)

        self._pubImu = rospy.Publisher('imu', Imu, queue_size=10)
        self._pubTemp = rospy.Publisher('temperature', Temperature, queue_size=10)
        self._pubMag = rospy.Publisher('magnetic_field', MagneticField, queue_size=10)
        self._pubPressure = rospy.Publisher('pressure', Float32, queue_size=10)

        self._state = CrazyflieROS.Disconnected
        Thread(target=self._update).start()

    def _try_to_connect(self):
        rospy.loginfo("Connecting to %s" % self.link_uri)
        self._state = CrazyflieROS.Connecting
        self._cf.open_link(self.link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        rospy.loginfo("Connected to %s" % link_uri)
        self._state = CrazyflieROS.Connected

        self._lg_imu = LogConfig(name="IMU", period_in_ms=10)
        self._lg_imu.add_variable("acc.x", "float")
        self._lg_imu.add_variable("acc.y", "float")
        self._lg_imu.add_variable("acc.z", "float")
        self._lg_imu.add_variable("gyro.x", "float")
        self._lg_imu.add_variable("gyro.y", "float")
        self._lg_imu.add_variable("gyro.z", "float")

        self._cf.log.add_config(self._lg_imu)
        if self._lg_imu.valid:
            # This callback will receive the data
            self._lg_imu.data_received_cb.add_callback(self._log_data_imu)
            # This callback will be called on errors
            self._lg_imu.error_cb.add_callback(self._log_error)
            # Start the logging
            self._lg_imu.start()
        else:
            rospy.logfatal("Could not add logconfig since some variables are not in TOC")

        self._lg_log2 = LogConfig(name="LOG2", period_in_ms=100)
        self._lg_log2.add_variable("mag.x", "float")
        self._lg_log2.add_variable("mag.y", "float")
        self._lg_log2.add_variable("mag.z", "float")
        self._lg_log2.add_variable("baro.temp", "float")
        self._lg_log2.add_variable("baro.pressure", "float")

        self._cf.log.add_config(self._lg_log2)
        if self._lg_log2.valid:
            # This callback will receive the data
            self._lg_log2.data_received_cb.add_callback(self._log_data_log2)
            # This callback will be called on errors
            self._lg_log2.error_cb.add_callback(self._log_error)
            # Start the logging
            self._lg_log2.start()
        else:
            rospy.logfatal("Could not add logconfig since some variables are not in TOC")

        p_toc = self._cf.param.toc.toc
        for group in p_toc.keys():
            self._cf.param.add_update_callback(group=group, name=None, cb=self._param_callback)
            for name in p_toc[group].keys():
                ros_param = "~{}/{}".format(group, name)
                cf_param = "{}.{}".format(group, name)
                if rospy.has_param(ros_param):
                    self._cf.param.set_value(cfparam, rospy.get_param(ros_param))
                else:
                    self._cf.param.request_param_update(cf_param)


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        rospy.logfatal("Connection to %s failed: %s" % (link_uri, msg))
        self._state = CrazyflieROS.Disconnected

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.logfatal("Connection to %s lost: %s" % (link_uri, msg))
        self._state = CrazyflieROS.Disconnected

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        rospy.logfatal("Disconnected from %s" % link_uri)
        self._state = CrazyflieROS.Disconnected

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        rospy.logfatal("Error when logging %s: %s" % (logconf.name, msg))


    def _log_data_imu(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        msg = Imu()
        # ToDo: it would be better to convert from timestamp to rospy time
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.tf_prefix + "/base_link"
        msg.orientation_covariance[0] = -1 # orientation not supported

        # measured in deg/s; need to convert to rad/s
        msg.angular_velocity.x = math.radians(data["gyro.x"])
        msg.angular_velocity.y = math.radians(data["gyro.y"])
        msg.angular_velocity.z = math.radians(data["gyro.z"])

        # measured in mG; need to convert to m/s^2
        msg.linear_acceleration.x = data["acc.x"] * 9.81
        msg.linear_acceleration.y = data["acc.y"] * 9.81
        msg.linear_acceleration.z = data["acc.z"] * 9.81

        self._pubImu.publish(msg)

        #print "[%d][%s]: %s" % (timestamp, logconf.name, data)

    def _log_data_log2(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        msg = Temperature()
        # ToDo: it would be better to convert from timestamp to rospy time
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.tf_prefix + "/base_link"
        # measured in degC (but value seems to be wrong/too high?)
        msg.temperature = data["baro.temp"]
        self._pubTemp.publish(msg)

        # ToDo: it would be better to convert from timestamp to rospy time
        msg = MagneticField()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.tf_prefix + "/base_link"

        # measured in Tesla
        msg.magnetic_field.x = data["mag.x"]
        msg.magnetic_field.y = data["mag.y"]
        msg.magnetic_field.z = data["mag.z"]

        self._pubMag.publish(msg)

        msg = Float32()
        # hPa (=mbar)
        msg.data = data["baro.pressure"]
        self._pubPressure.publish(msg)

    def _param_callback(self, name, value):
        ros_param = "~{}".format(name.replace(".", "/"))
        rospy.set_param(ros_param, value)

    def _send_setpoint(self):
        roll = self._cmdVel.linear.y
        pitch = self._cmdVel.linear.x
        yawrate = self._cmdVel.angular.z
        thrust = max(10000, int(self._cmdVel.linear.z))
        #print(roll, pitch, yawrate, thrust)
        self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _cmdVelChanged(self, data):
        self._cmdVel = data
        self._send_setpoint()

    def _update(self):
        while not rospy.is_shutdown():
            if self._state == CrazyflieROS.Disconnected:
                self._try_to_connect()
            elif self._state == CrazyflieROS.Connected:
                # Crazyflie will shut down if we don't send any command for 500ms
                # Hence, make sure that we don't wait too long
                self._send_setpoint()
                rospy.sleep(0.2)
            else:
                rospy.sleep(0.5)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        rospy.sleep(0.1)
        self._cf.close_link()

if __name__ == '__main__':
    rospy.init_node('crazyflie', anonymous=True)
    crazyflieSDK = rospy.get_param("~crazyflieSDK")
    uri = rospy.get_param("~uri")
    tf_prefix = rospy.get_param("~tf_prefix")

    sys.path.append(crazyflieSDK)
    import cflib
    from cflib.crazyflie import Crazyflie
    from cfclient.utils.logconfigreader import LogConfig

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    CrazyflieROS(uri, tf_prefix)
    rospy.spin()
