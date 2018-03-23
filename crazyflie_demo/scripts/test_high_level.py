#!/usr/bin/env python

import rospy
import crazyflie
import time

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("crazyflie", "/vicon/crazyflie/crazyflie")

    cf.setParam("commander/enHighLevel", 1)

    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(3.0)

    cf.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    time.sleep(3.0)

    cf.land(targetHeight = 0.0, duration = 2.0)
