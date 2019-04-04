#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

from __future__ import absolute_import, division, unicode_literals, print_function

import rospy
import crazyflie
import uav_trajectory
import time
import tf
# from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData

from threading import Thread

import tty, termios
import sys

if __name__ == '__main__':
    rospy.init_node('test_high_level')
    # rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)
    prefix = '/cf1'
    cf = crazyflie.Crazyflie("/cf1", "world")
    rospy.wait_for_service(prefix + '/update_params')
    rospy.loginfo("found update_params service")

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2)  # Use EKF

    cf.setParam("ctrlMel/kp_z", 1.0)  # reduce z wobble - default 1.25
    # cf.setParam("ctrlMel/ki_z", 0.06)  # reduce z wobble - default 0.05
    # cf.setParam("ctrlMel/kd_z", 0.2)  # reduce z wobble - default 0.4

    ## reset kalman
    cf.setParam("kalman/initialX", 0)
    cf.setParam("kalman/initialY", 0)
    cf.setParam("kalman/initialZ", 0)
    cf.setParam("kalman/resetEstimation", 1)
    ########

    cf.setParam("stabilizer/controller", 2)  # 2=Use mellinger controller
    time.sleep(1.0)

    rospy.loginfo("launching")

    # cf.takeoff(targetHeight = 0.4, duration = 3.0)
    # time.sleep(5.0)
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("/home/galbra/catkin_ws/src/crazyflie_ros/crazyflie_demo/scripts/takeoff.csv")

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("/home/galbra/catkin_ws/src/crazyflie_ros/crazyflie_demo/scripts/sine.csv")

    print('traj2 duration :', traj2.duration)

    cf.uploadTrajectory(0, 0, traj1)
    cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    cf.startTrajectory(0, timescale=1.0)
    time.sleep(traj1.duration * 2.0)

    cf.startTrajectory(1, timescale=1.5)
    time.sleep(traj2.duration * 1.5)
    time.sleep(1)  # additional delay at end

    cf.startTrajectory(0, timescale=1.0, reverse=True)
    time.sleep(1.2)

    cf.stop()
