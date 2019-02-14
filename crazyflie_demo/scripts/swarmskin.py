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
import numpy as np
from math import *

def start_recording():
    os.system("rosbag record -o Desktop/SwarmSkin/ /vicon/crazyflie100/crazyflie100 /vicon/landing_pad/landing_pad")
# def land_detector():
#     while not rospy.is_shutdown():
#         drone1.position()
#         lp1.position()
#         # lp2.position()
#         if abs(drone1.pose[2] - lp1.pose[2])<0.05:
#             print "Stop motors"
#             cf1.stop()
#             # cf1.setParam("ring/effect", 14)
#             time.sleep(0.1)
#             if data_recording:
#                 print 'kill the recorder'
#                 node_list = os.popen("rosnode list").read()
#                 os.system('rosnode kill '+node_list[72:99])
#             print "Shutdown"
#             rospy.signal_shutdown("landed")


def land_detector():
    land_time = - np.ones( min(len(drone_list), len(lp_list)) )
    switched_off = np.zeros(len(drone_list))
    while not rospy.is_shutdown():
        for drone in drone_list: drone.position()
        for lp in lp_list: lp.position()
        landed_drones_number = 0
        for i in range(min(len(drone_list), len(lp_list))):
            # print(drone_list[i], lp_list[i])
            print(abs(drone_list[i].pose[0] - lp_list[i].pose[0]), abs(drone_list[i].pose[1] - lp_list[i].pose[1]), abs(drone_list[i].pose[2] - lp_list[i].pose[2]))
            if abs(drone_list[i].pose[0] - lp_list[i].pose[0])<0.15 and abs(drone_list[i].pose[1] - lp_list[i].pose[1])<0.15 and abs(drone_list[i].pose[2] - lp_list[i].pose[2])<0.07:
                landed_drones_number += 1
                # if land_time[i]==-1:
                #     land_time[i] = time.time()-start_time
                #     print("Drone %d is landed after %s seconds" % (i+1, land_time[i]))
                print "Switch off the motors, %d drone" %i
                if switched_off[i]==0:
                    for t in range(3): cf_list[i].stop()                  # stop motors
                switched_off[i] = 1
                # cf_list[i].setParam("tf/state", 0) # switch off LEDs
                if landed_drones_number==len(drone_list): rospy.signal_shutdown("landed")

def land_detector1():
    switched_off = np.zeros(len(drone_list))
    while not rospy.is_shutdown():
        if switched_off[0] and switched_off[1]: break
        for drone in drone_list: drone.position()
        for lp in lp_list: lp.position()
        landed_drones_number = 0
        for i in range(min(len(drone_list), len(lp_list))):
            # print(drone_list[i], lp_list[i])
            print(abs(drone_list[i].pose[0] - lp_list[i].pose[0]), abs(drone_list[i].pose[1] - lp_list[i].pose[1]), abs(drone_list[i].pose[2] - lp_list[i].pose[2]))
            if abs(drone_list[i].pose[0] - lp_list[i].pose[0])<0.15 and abs(drone_list[i].pose[1] - lp_list[i].pose[1])<0.15 and abs(drone_list[i].pose[2] - lp_list[i].pose[2])<0.07:
                landed_drones_number += 1
                # if land_time[i]==-1:
                #     land_time[i] = time.time()-start_time
                #     print("Drone %d is landed after %s seconds" % (i+1, land_time[i]))
                print "Switch off the motors, %d drone" %i
                if switched_off[i]==0:
                    for t in range(3): cf_list[i].stop()                  # stop motors
                switched_off[i] = 1
                # cf_list[i].setParam("tf/state", 0) # switch off LEDs
                if landed_drones_number==len(drone_list): rospy.signal_shutdown("landed")



rospy.init_node('test_high_level')

# Names and variables
TAKEOFFHEIGHT = 0.9
data_recording = False

# lp1_name = 'lp1'
# lp2_name = 'lp2' # lp1

# drone1 = swarmlib.Drone(cf1_name)
# drone_list = [drone1]
# lp1    = swarmlib.Mocap_object(lp1_name)
# lp2    = swarmlib.Mocap_object(lp2_name)
# lp_list = [lp1, lp2]


# cf_names = ['cf1', 'cf2', 'cf3', 'cf4']
# cf_names = ['cf1', 'cf2', 'cf3']
cf_names = ['cf1', 'cf2']
# cf_names = ['cf1']

lp_names = []
# lp_names = ['lp2', 'lp1', 'lp3', 'lp4']
# lp_names = ['lp2', 'lp1']
# lp_names = ['lp2']
drone_list = []
for name in cf_names:
    drone = swarmlib.Drone(name)
    drone_list.append(drone)
lp_list = []
for lp_name in lp_names:
    lp_list.append( swarmlib.Mocap_object(lp_name) )




# cf1_name = 'cf1'
# cf2_name = 'cf2'
# cf3_name = 'cf3'
# cf4_name = 'cf4'
# cf1 = crazyflie.Crazyflie(cf1_name, '/vicon/'+cf1_name+'/'+cf1_name)
# cf2 = crazyflie.Crazyflie(cf2_name, '/vicon/'+cf2_name+'/'+cf2_name)
# cf3 = crazyflie.Crazyflie(cf3_name, '/vicon/'+cf3_name+'/'+cf3_name)
# cf4 = crazyflie.Crazyflie(cf4_name, '/vicon/'+cf4_name+'/'+cf4_name)
# cf_list = [cf1, cf2, cf3, cf4]
# cf_list = [cf1, cf2, cf3]
# cf_list = [cf1, cf2]
# cf_list = [cf1]

# landing_velocity = random.choice([13,22]) #13,22
# landing_velocity = 13
# landing_velocity = 22
# landing_velocity = 30
# print "landing_velocity", landing_velocity




cf_list = []
for cf_name in cf_names:
    print "adding.. ", cf_name
    cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    cf_list.append(cf)
for t in range(3):
    for cf in cf_list:
        print "takeoff.. ", cf.prefix
        cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 5.0)
time.sleep(200.0)

# for cf in cf_list:
#     print "goto.. ", cf.prefix
#     cf.goTo(goal = [-0.5, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
# time.sleep(2.0)
# for cf in cf_list:
#     print "goto.. ", cf.prefix
#     cf.goTo(goal = [0.5, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
# time.sleep(2.0)

# l = 0.25
# width = 0.03
# goto_arr = [ [(-0.4+l),l+0.03+width],  
#              [-0.4,l+width],
#              [(-0.4+l),-l-0.03-width],
#              [-0.4,-l-width] ]
# r = 0.3; theta1 = pi/4; theta2 = pi/2
# l = 0.25 # distance between drones (arm length)
# width = 0.5
# human_pose = np.array([0.5,0.5]); hx = human_pose[0]; hy = human_pose[1]
# goto_arr = [ [hx+ (r+l)*cos(theta1), hy+ (r+l)*sin(theta1) +width/2],
#              [hx+ r*cos(theta1),     hy+ r*sin(theta1) +width/2],
#              [hx+ (r+l)*cos(theta2), hy- (r+l)*sin(theta2) -width/2],
#              [hx+ r*cos(theta2),     hy- r*sin(theta2) -width/2] ]

# for t in range(3):
#     cf_list[0].goTo(goal=[0,0.8, TAKEOFFHEIGHT], yaw=0.0, duration=3.0, relative=False)
#     cf_list[1].goTo(goal=[0,0.4, TAKEOFFHEIGHT], yaw=0.0, duration=3.0, relative=False)
#     cf_list[3].goTo(goal=[0,0.0, TAKEOFFHEIGHT], yaw=0.0, duration=3.0, relative=False)
#     cf_list[2].goTo(goal=[0,-0.4, TAKEOFFHEIGHT], yaw=0.0, duration=3.0, relative=False)
# time.sleep(4.0)



# # for cf in cf_list[:2]:
# #     print "goto.. ", cf.prefix
# #     drone_id = int(cf.prefix[2])
# #     cf.goTo(goal = goto_arr[drone_id-1]+[TAKEOFFHEIGHT], yaw=0.0, duration = 3.0, relative = False)
# # time.sleep(3.0)

# # print 'Landing...1'
# # for cf in cf_list[:2]:
# #     cf.land(targetHeight = 0.0, duration = 15)
# # time.sleep(5)
# # # land_detector1()

# for cf in cf_list[2:]:
#     print "goto.. ", cf.prefix
#     drone_id = int(cf.prefix[2])
#     cf.goTo(goal = goto_arr[drone_id-1]+[TAKEOFFHEIGHT], yaw=0.0, duration = 4.0, relative = False)
# time.sleep(4.0)

# print 'Landing...2'
# for cf in cf_list[2:]:
#     cf.land(targetHeight = 0.0, duration = 15)




# # for cf in cf_list:
# #     # import tf
# #     cf.setParam("tf/state", 4)

# # if data_recording:
# #     print "Data recording started"
# #     pose_recording = Process(target=start_recording)
# #     pose_recording.start()


# land_detector()







    




























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