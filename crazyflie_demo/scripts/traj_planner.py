#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

from __future__ import absolute_import, division, unicode_literals, print_function

import time

import crazyflie
import rospy
import uav_trajectory
from std_msgs.msg import Empty


# Todo change to trajectory planner only without creating a CF object!!!!

# Globals
traj_container = []


def launcher(msg):
    rospy.loginfo("******************* launching  --- ".format(prefix))

    # Loop on all trajectory's in traj_container:
    for i, traj in enumerate(traj_container):
        cf.startTrajectory(i, timescale=1.0)
        time.sleep(traj.duration * 1.6)

    # land (assuming first trajectory in container is "TakeOFF")
    cf.startTrajectory(0, timescale=0.7, reverse=True)
    time.sleep(1.1)

    cf.stop()


if __name__ == '__main__':
    # Create a ros node
    rospy.init_node('highlevel_trajectory_manager', anonymous=True)

    # prefix = sys.argv[1]
    # prefix = "/" + prefix
    prefix = "/" + rospy.get_param("~tf_prefix")
    cf = crazyflie.Crazyflie(prefix, "world")

    rospy.loginfo("********* wait for update params -- {} ".format(prefix))
    rospy.wait_for_service(prefix + '/update_params')

    rospy.loginfo("found update_params service for {} ".format(prefix))

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

    trajectory_list = rospy.get_param("~traj_list")

    # A simple loop for uploading all trajectory's to CF.
    pieceOffset = 0
    pieceOffset_of_previeus_traj = 0
    for i, traj in enumerate(trajectory_list):
        rospy.loginfo("\n\n******************* uploading: {}".format(traj))

        temp_traj = uav_trajectory.Trajectory()
        temp_traj.loadcsv(traj)
        traj_container.append(temp_traj)
        if i is 0:
            # First traj, upload with 0 piecOffset, than update pieceoffset.
            cf.uploadTrajectory(i, 0, temp_traj)
            pieceOffset = pieceOffset + len(temp_traj.polynomials)

            rospy.wait_for_service(prefix + "/upload_trajectory")
            uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
            uploadTrajectoryService(i, 0, temp_traj)

        else:
            cf.uploadTrajectory(i, pieceOffset, temp_traj)
            pieceOffset = pieceOffset + len(temp_traj.polynomials)

        rospy.loginfo("\n\n******************* traj {} uploaded".format(i))

    rospy.Subscriber("swarm_launch", Empty, launcher)

    rospy.loginfo("******************* ready to fly  --- {0} ".format(prefix))

    rospy.spin()  # do nothing - until swarm_launch message
