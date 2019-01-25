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

def start_recording():
    os.system("rosbag record -o Desktop/SwarmSkin/ /vicon/crazyflie100/crazyflie100 /vicon/landing_pad/landing_pad")
def land_detector():
    while not rospy.is_shutdown():
        drone1.position()
        lp1.position()
        # lp2.position()
        if abs(drone1.pose[2] - lp1.pose[2])<0.05:
            print "Stop motors"
            cf1.stop()
            # cf1.setParam("ring/effect", 14)
            time.sleep(0.1)
            if data_recording:
                print 'kill the recorder'
                node_list = os.popen("rosnode list").read()
                os.system('rosnode kill '+node_list[72:99])
            print "Shutdown"
            rospy.signal_shutdown("landed")


rospy.init_node('test_high_level')

# Names and variables
TAKEOFFHEIGHT = 1.8
data_recording = False
cf1_name = 'cf1'
lp1_name = 'lp1'
lp2_name = 'lp2' # lp1

drone1 = swarmlib.Drone(cf1_name)
drone_list = [drone1]
lp1    = swarmlib.Mocap_object(lp1_name)
lp2    = swarmlib.Mocap_object(lp2_name)
lp_list = [lp1, lp2]

cf1 = crazyflie.Crazyflie(cf1_name, '/vicon/'+cf1_name+'/'+cf1_name)
cf_list = [cf1]

# landing_velocity = random.choice([13,22]) #13,22
# landing_velocity = 13
# landing_velocity = 22
landing_velocity = 30
print "landing_velocity", landing_velocity





print "Takeoff"
for cf in cf_list:
    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 7.0)
time.sleep(7.0)



if data_recording:
    print "Data recording started"
    pose_recording = Process(target=start_recording)
    pose_recording.start()

print 'Landing...'
for cf in cf_list:
    cf.land(targetHeight = 0.0, duration = landing_velocity)


land_detector()







    




























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