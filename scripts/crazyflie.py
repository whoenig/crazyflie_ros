#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import time, sys
from threading import Thread

class CrazyflieROS:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        rospy.loginfo("Connectving to %s" % link_uri)

        self._cmdVel = Twist()
        rospy.Subscriber("cmd_vel", Twist, self._cmdVelChanged)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        #Thread(target=self._update).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        rospy.logfatal("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.logfatal("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        rospy.logfatal("Disconnected from %s" % link_uri)

    def _cmdVelChanged(self, data):
        self._cmdVel = data
        roll = self._cmdVel.linear.y
        pitch = self._cmdVel.linear.x
        yawrate = self._cmdVel.angular.z
        thrust = max(10000, int(self._cmdVel.linear.z))
        print(roll, pitch, yawrate, thrust)
        self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _update(self):
        while not rospy.is_shutdown():
            roll = self._cmdVel.linear.y
            pitch = self._cmdVel.linear.x
            yawrate = self._cmdVel.angular.z
            thrust = max(10000, int(self._cmdVel.linear.z))
            print(roll, pitch, yawrate, thrust)
            #self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            rospy.sleep(0.2)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        rospy.sleep(0.1)
        self._cf.close_link()

if __name__ == '__main__':
    rospy.init_node('crazyflie', anonymous=True)
    crazyflieSDK = rospy.get_param("~crazyflieSDK")
    uri = rospy.get_param("~uri")

    sys.path.append(crazyflieSDK)
    import cflib
    from cflib.crazyflie import Crazyflie

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    CrazyflieROS(uri)
    rospy.spin()
