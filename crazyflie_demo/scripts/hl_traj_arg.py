#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

from __future__ import absolute_import, division, unicode_literals, print_function

import rospy
import crazyflie
import uav_trajectory
import time
# import tf


# from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData

# from threading import Thread
# import tty, termios

import sys


def launcher(msg):
    rospy.loginfo("******************* launching  --- ".format(prefix))

    # takeoff
    cf.startTrajectory(0, timescale=1.0)
    time.sleep(traj1.duration * 1.6)

    # figure8
    cf.startTrajectory(1, timescale=1.3)
    time.sleep(traj2.duration * 1.7)

    # land
    cf.startTrajectory(0, timescale=0.7, reverse=True)
    time.sleep(1.1)

    cf.stop()


if __name__ == '__main__':
    rospy.init_node('test_high_level')

    prefix = sys.argv[1]
    prefix = "/" + prefix
    cf = crazyflie.Crazyflie(prefix, "world")
    rospy.loginfo("********* wait for update params -- {0} ".format(prefix))
    rospy.wait_for_service(prefix + '/update_params')
    rospy.loginfo("found update_params service for {0} ".format(prefix))

    cf.setParam("commander/enHighLevel", 1)  # enable use of high level commands
    cf.setParam("stabilizer/estimator", 2)  # 2=Use EKF

    cf.setParam("ctrlMel/kp_z", 0.7)  # reduce z wobble - default 1.25
    # cf.setParam("ctrlMel/ki_z", 0.06)  # reduce z wobble - default 0.05
    # cf.setParam("ctrlMel/kd_z", 0.2)  # reduce z wobble - default 0.4
    time.sleep(0.1)
    ## reset kalman
    cf.setParam("kalman/initialX", 0)
    cf.setParam("kalman/initialY", 0)
    cf.setParam("kalman/initialZ", 0)
    cf.setParam("kalman/resetEstimation", 1)
    ########

    cf.setParam("stabilizer/controller", 2)  # 2=Use mellinger controller

    time.sleep(0.1)

    rospy.loginfo("******************* uploading to {0}".format(prefix))

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("/home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/scripts/takeoff.csv")

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("/home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv")

    cf.uploadTrajectory(0, 0, traj1)
    cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
    rospy.Subscriber("swarm_launch", Empty, launcher)

    rospy.loginfo("******************* ready to fly  --- {0} ".format(prefix))

    rospy.spin()  # do nothing - until swarm_launch message
