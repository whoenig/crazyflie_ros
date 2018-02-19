#!/usr/bin/env python

import crazyflie
import time

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("crazyflie", "/vicon/crazyflie/crazyflie")

    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(3.0)
    cf.land(targetHeight = 0.0, duration = 2.0)
