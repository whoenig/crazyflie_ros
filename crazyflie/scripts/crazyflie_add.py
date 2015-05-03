#!/usr/bin/env python

import rospy
from crazyflie.srv import *

if __name__ == "__main__":
    rospy.init_node('crazyflie_add', anonymous=True)
    uri = rospy.get_param("~uri")
    tf_prefix = rospy.get_param("~tf_prefix")
    roll_trim = float(rospy.get_param("~roll_trim", "0"))
    pitch_trim = float(rospy.get_param("~pitch_trim", "0"))
    enable_logging = bool(rospy.get_param("~enable_logging", "True"))
    rospy.loginfo("wait_for_service /add_crazyflie")
    rospy.wait_for_service('/add_crazyflie')
    rospy.loginfo("found /add_crazyflie")
    add_crazyflie = rospy.ServiceProxy('/add_crazyflie', AddCrazyflie)
    add_crazyflie(uri, tf_prefix, roll_trim, pitch_trim, enable_logging)
