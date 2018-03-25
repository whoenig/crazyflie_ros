#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("crazyflie", "/vicon/crazyflie/crazyflie")

    cf.setParam("commander/enHighLevel", 1)

    # cf.takeoff(targetHeight = 0.5, duration = 2.0)
    # time.sleep(3.0)

    # cf.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    # time.sleep(3.0)

    # cf.land(targetHeight = 0.0, duration = 2.0)

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("takeoff.csv")
    idx1 = 0

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("figure8.csv")
    idx2 = len(traj1.polynomials)

    print(traj1.duration)

    cf.uploadTrajectoryPieces(idx1, traj1)
    cf.uploadTrajectoryPieces(idx2, traj2)

    cf.startTrajectory(idx1, len(traj1.polynomials))
    time.sleep(traj1.duration)

    cf.startTrajectory(idx2, len(traj2.polynomials))
    time.sleep(traj2.duration)

    cf.startTrajectory(idx1, len(traj1.polynomials), reverse=True)
    time.sleep(traj1.duration)

    cf.stop()
