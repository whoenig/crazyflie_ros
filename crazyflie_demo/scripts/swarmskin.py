#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

import swarmlib
import message_filters
from geometry_msgs.msg import PoseStamped, TransformStamped
import os
from multiprocessing import Process
import random



rospy.init_node('test_high_level')


cf1_name = 'cf1'
lp2_name = 'lp2' # lp1

drone1 = swarmlib.Drone(cf1_name)
lp2    = swarmlib.Mocap_object(lp2_name)

def start_recording():
    os.system("rosbag record -o Desktop/SwarmSkin/ /vicon/crazyflie100/crazyflie100 /vicon/landing_pad/landing_pad")

pose_recording = Process(target=start_recording)



def land_detector(cf):

    while not rospy.is_shutdown():

        drone1.position()
        lp2.position()

        print "abs(drone1.pose[2] - lp2.pose[2])", abs(drone1.pose[2] - lp2.pose[2])

        if abs(drone1.pose[2] - lp2.pose[2])<0.05:
            print "Stop motors"
            cf.stop()
            # cf.setParam("ring/effect", 14)
            time.sleep(0.1)

            # print 'kill the recorder'
            # node_list = os.popen("rosnode list").read()
            # os.system('rosnode kill '+node_list[72:99])

            print "Shutdown"
            rospy.signal_shutdown("landed")









cf = crazyflie.Crazyflie("cf1", "/vicon/cf1/cf1")



# landing_velocity = random.choice([13,22]) #13,22
# landing_velocity = 13
landing_velocity = 22
print "landing_velocity", landing_velocity



cf.setParam("commander/enHighLevel", 1)
cf.setParam("stabilizer/estimator", 2) # Use EKF
cf.setParam("stabilizer/controller", 2) # Use mellinger controller
cf.takeoff(targetHeight = 2.0, duration = 7.0)
time.sleep(7.0)

# pose_recording.start()



cf.land(targetHeight = 0.0, duration = landing_velocity)

land_detector(cf)







    




























    # cf.goTo(goal = [0.0, -0.4, 0.0], yaw=0.0, duration = 2.0, relative = True)
    # time.sleep(2.0)

    # cf.goTo(goal = [0.0, 0.8, 0.0], yaw=0.0, duration = 4.0, relative = True)
    # time.sleep(4.0)

    # cf.goTo(goal = [0.0, -0.4, 0.0], yaw=0.0, duration = 2.0, relative = True)
    # time.sleep(2.0)






    # time_to_sleep = 1.5
    # cf.goTo(goal = [.5, .5, 0.0], yaw=-.75*3.14, duration = 2.0, relative = True)
    # time.sleep(3.0)
    # for i in range(2):
    #     cf.goTo(goal = [0.0, -1.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [-1.0, 0.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [0.0, 1.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [1.0, 0.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    # cf.goTo(goal = [0, 0.0, 0.0], yaw=0, duration = 2.0, relative = False)
    # time.sleep(3.0)



    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("takeoff.csv")

    # traj2 = uav_trajectory.Trajectory()
    # traj2.loadcsv("figure8.csv")

    # print(traj1.duration)

    # cf.uploadTrajectory(0, 0, traj1)
    # cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    # cf.startTrajectory(0, timescale=1.0)
    # time.sleep(traj1.duration * 2.0)

    # cf.startTrajectory(1, timescale=2.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(0, timescale=1.0, reverse=True)
    # time.sleep(traj1.duration * 1.0)

    # cf.stop()