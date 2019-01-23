#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from math import *
import math
import time
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from time import sleep
import message_filters
import matplotlib.pyplot as plt
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from crazyflie_driver.msg import FullState
import geometry_msgs
import tf_conversions
import crazyflie
import swarmlib
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


# PARAMETERs #############
toFly            = 1
pos_ctrl         = 1
pos_coef         = 3.0 #2
vel_ctrl         = 0
vel_koef         = 1.0
impedance_on     = 0
TAKEOFFHEIGHT    = 1.0 # meters 1.45
TakeoffTime      = 4     # seconds
l                = 0.4     # distance between drones, meters
R_obstacles      = 0.27
tacile_glove_on  = 0
uper_limits      = np.array([ 1.7, 1.70, 2.5 ]) # np.array([ 2.0, 2.0, 2.5 ])
lower_limits     = np.array([ -1.7, -1.50, -2.5 ]) # np.array([ 2.0, 2.0, 2.5 ])
cf1_name         = 'cf1'
cf2_name         = 'cf2'
cf3_name         = 'cf3'
human_name       = 'karton_big' #'glove' 'karton_big'
obstacle_name_list = []
# obstacle_name_list = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8', 'obstacle9', 'obstacle10', 'obstacle11', 'obstacle12', 'obstacle13']
# 'obstacle13', 'obstacle14', 'obstacle15', 'obstacle16', 'obstacle17', 'obstacle18', 'obstacle19', 'obstacle20', 'obstacle21', 'obstacle22', 'obstacle23', 'obstacle24', 'obstacle25'
data_recording = False
killed_recorder = False
subject_name = "Evgeny"
# Variables #########################
initialized = False
imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()
# SetUp
if tacile_glove_on:
	swarmlib.startXbee()


# START ##########################################################################################################
rospy.init_node('follow_multiple', anonymous=True)

# Objects init  TODO: swarmlib.swarm_manager(drone_name_list)
human = swarmlib.Mocap_object(human_name)
drone1 = swarmlib.Drone(cf1_name, leader = True)
drone2 = swarmlib.Drone(cf2_name)
drone3 = swarmlib.Drone(cf3_name)
swarm = [drone1, drone2, drone3]
for drone in swarm:
	print "drone_name: ", drone.name
obstacle_objects_list = []
for obstacle in obstacle_name_list:
	obstacle_objects_list.append(swarmlib.Mocap_object(obstacle))


if toFly:
	print "takeoff"
	cf1 = crazyflie.Crazyflie(cf1_name, '/vicon/'+cf1_name+'/'+cf1_name)
	# cf2 = crazyflie.Crazyflie(cf2_name, '/vicon/'+cf2_name+'/'+cf2_name)
	# cf3 = crazyflie.Crazyflie(cf3_name, '/vicon/'+cf3_name+'/'+cf3_name)
	# cf_list = [cf1, cf2, cf3]
	cf_list = [cf1]
	for cf in cf_list:
		cf.setParam("commander/enHighLevel", 1)
		cf.setParam("stabilizer/estimator", 2) # Use EKF
		cf.setParam("stabilizer/controller", 2) # Use mellinger controller
		cf.setParam("ring/effect", 2)
		cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 3.0)
	time.sleep(TakeoffTime) # time to takeoff and select position for human
if data_recording:
	print "start data recording"
	swarmlib.start_recording(cf1_name, cf2_name, cf3_name, human_name, obstacle_name_list, subject_name, tacile_glove_on)



rate = rospy.Rate(100)
while not rospy.is_shutdown():

	swarmlib.publish_vel(human.vel, 'human_vel')
	swarmlib.publish_vel(drone1.vel, 'drone1_vel')
	swarmlib.publish_vel(drone1.vel_sp, 'drone1_vel_sp')


	for drone in swarm: drone.position()
	for drone in swarm: drone.velocity()
	for drone in swarm: drone.velocity_sp()
	human.position()
	human.velocity()

	# Human following strategies for the leader drone
	if pos_ctrl:
		if not initialized:
			human_pose_init = human.pose # np.array([human_pose[0],human_pose[1],human_pose[2]])
			if toFly:
				drone1_pose_init = drone1.pose# + np.array([1,0,0])
			else:
				drone1_pose_init = np.array([1.4, 0, 0])
			initialized = True
		dx, dy = (human.pose - human_pose_init)[:2]
		drone1.sp = np.array([  drone1_pose_init[0] + pos_coef*dx,
								drone1_pose_init[1] + pos_coef*dy    ,
								human.pose[2]        ]) #TAKEOFFHEIGHT
	elif vel_ctrl:
		if not initialized:
			point_to_follow_pose_prev = np.array([drone1.pose[0],drone1.pose[1],TAKEOFFHEIGHT])
			human_pose_init = human.pose

			human_pose_init[2]=human_pose_init[2]-0.3 ###################################
			
			time_prev = time.time()
			initialized = True
		cmd_vel = -vel_koef*(human_pose_init-human.pose)
		np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.035), 0)
		time_now = time.time()
		point_to_follow_pose = point_to_follow_pose_prev + cmd_vel*(time.time()-time_prev)
		
		if point_to_follow_pose[2]>1.45:#############################
			point_to_follow_pose[2]=1.45######################################################3

		time_prev = time_now
		point_to_follow_pose_prev = point_to_follow_pose
		drone1.sp = point_to_follow_pose
	else:
		drone1.sp = np.array([  human.pose[0] -3.5*l , #   5
								human.pose[1]        ,
								TAKEOFFHEIGHT              ])
	# drone1.apply_limits(uper_limits, lower_limits)

	if not impedance_on: # simple triangle
		drone2.sp = drone1.sp + np.array([-l*0.86 , l/2, 0]) # triangle with equal sides
		drone3.sp = drone1.sp + np.array([-l*0.86 ,-l/2, 0]) # triangle with equal sides
		# drone3.sp = drone2.sp + np.array([0 ,-l, 0])
	if impedance_on and not vel_ctrl: # Drone-to-drone IMPEDANCE from human
		hum_vel = swarmlib.hum_vel(human.pose)
		imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
		imp_pose_prev = imp_pose; imp_vel_prev = imp_vel

		# drone1.sp = drone1.sp + imp_pose # human-to swarm impedance
		# within swarm impedance:
		drone2.sp = drone1.sp + np.array([-l*0.86 , l/2, 0]) + imp_pose # triangle with equal sides
		drone3.sp = drone1.sp + np.array([-l*0.86 ,-l/2, 0]) + imp_pose # triangle with equal sides

		drone2.sp[1] = drone2.sp[1] - imp_pose[0]*0.5
		drone3.sp[1] = drone3.sp[1] + imp_pose[0]*0.5


	# # ROTATION due to hand position
	# centroid = swarmlib.centroid_calc(drone1, drone2, drone3)
	# for drone in swarm:
	# 	drone.sp = swarmlib.rotate(centroid, drone, human)


	# # OBSTACLEs
	# centroid_before_obstacles = swarmlib.centroid_calc(drone1, drone2, drone3)

	# Update obstacles status
	for drone in swarm:
		for obstacle in obstacle_objects_list:
			drone = swarmlib.update_obstacle(drone, obstacle, R_obstacles)
			if drone.near_obstacle: # each drone could be only near one obstacle (just an assumption),
				break               # then if one obstacle is near -> break the loop over obstacles

	print '\ndrone1.near_obstacle', drone1.near_obstacle
	if drone1.near_obstacle:
		print 'drone1.nearest_obstacle', drone1.nearest_obstacle.name
	print "drone1.rad_imp.inside", drone1.rad_imp.inside
	print "drone1.rad_imp.penetration", drone1.rad_imp.penetration
	print 'drone1.rad_imp.imp_pose', drone1.rad_imp.imp_pose
	print 'np.linalg.norm(drone1.rad_imp.penetration)', np.linalg.norm(drone1.rad_imp.penetration)

	# Move over the circle avoiding obstacle
	for drone in swarm:
		if drone.near_obstacle:
			swarmlib.pose_update_obstacle_circle(drone, R_obstacles)

	if drone1.near_obstacle:
		path_limit = 100
		drone1.publish_path_(limit=path_limit) #set -1 for unlimited path


	# Radius impedance
	for drone in swarm:
		if drone.near_obstacle:
			drone = swarmlib.radius_obstacle_impedance(drone)

	# print 'drone1.rad_imp.imp_pose', drone1.rad_imp.imp_pose

	# for obstacle in obstacle_objects_list:
	# 	drone1 = swarmlib.pose_update_obstacle(drone1, obstacle, R_obstacles)
	# 	drone2 = swarmlib.pose_update_obstacle(drone2, obstacle, R_obstacles)
	# 	drone3 = swarmlib.pose_update_obstacle(drone3, obstacle, R_obstacles)

	# # tl.waitForTransform("/world", '/vicon/karton_big/karton_big', rospy.Time(0), rospy.Duration(1))
	# twist = tl.lookupTwist('/world', '/vicon/karton_big/karton_big', rospy.Time(0), rospy.Duration(0.5)) #Twist
	# swarmlib.publish_vel(twist, 'human_vel')
	# print twist[0]




	# TO VISUALIZE
	human.publish_position()
	path_limit = 100
	for drone in swarm:
		drone.publish_sp()
		drone.publish_path(limit=path_limit) #set -1 for unlimited path
	for obstacle in obstacle_objects_list:
		obstacle.publish_position()

	# # TO FLY
	if toFly:
		for drone in swarm:
			drone.fly()

	# tactile glove
	if tacile_glove_on:
		centroid_after_obstacles = swarmlib.centroid_calc(drone1, drone2, drone3)
		move_right = False
		move_left = False
		if centroid_after_obstacles[1]-centroid_before_obstacles[1]>0.01:
			move_right = True
		elif centroid_after_obstacles[1]-centroid_before_obstacles[1]<-0.01:
			move_left = True
		swarmlib.publish_pose(centroid_after_obstacles, np.array([0,0,0]), "centroid_after_obstacles")
		swarmlib.publish_pose(centroid_before_obstacles, np.array([0,0,0]), "centroid_before_obstacles")

		swarmlib.tactile_patterns(drone1, drone2, drone3, human, l, move_right, move_left)

	# stop recorder
	if drone1.sp[0]==-1.7:
		# Stop recorder
		if not killed_recorder and data_recording:
			swarmlib.killer_of_recorder()
			killed_recorder = True

	# Landing
	if toFly and human.pose[2]<0.7:
		
		# Stop recorder
		if not killed_recorder and data_recording:
			swarmlib.killer_of_recorder()
			killed_recorder = True

		print 'Landing!!!'
		drone1_landing_pose = drone1.pose
		drone2_landing_pose = drone2.pose
		drone3_landing_pose = drone3.pose
		while not rospy.is_shutdown():
			drone1.sp = drone1_landing_pose
			drone2.sp = drone2_landing_pose
			drone3.sp = drone3_landing_pose
			drone1_landing_pose[2] = drone1_landing_pose[2]-0.007
			drone2_landing_pose[2] = drone2_landing_pose[2]-0.007
			drone3_landing_pose[2] = drone3_landing_pose[2]-0.007
			for drone in swarm: drone.fly()
			if drone1.sp[2]<-1.0 and drone2.sp[2]<-1.0 and drone2.sp[2]<-1.0:
				sleep(1)
				for cf in cf_list:
					cf.stop()
				print 'reached the floor, shutdown'
				rospy.signal_shutdown('landed')
			rate.sleep()
	rate.sleep()

	# END #######################################################################



















































		# drone1.sp = drone1.sp + imp_pose
		# drone2.sp = drone2.sp + imp_pose
		# drone3.sp = drone3.sp + imp_pose
		# drone2.sp[1] = drone2.sp[1] - imp_pose[0]*0.15
		# drone3.sp[1] = drone3.sp[1] + imp_pose[0]*0.15

		# # all drones follow a human
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] + l, drone1.sp[2] ])
		# drone3.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] - l, drone1.sp[2] ])

		# # first is followed by the second
		# # Second is followed by the third
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1_pose[0] - l, drone1_pose[1] + l, drone1_pose[2] ])
		# drone3.sp = np.array([ drone2_pose[0]    , drone2_pose[1] -2*l, drone2_pose[2] ])

		# OBSTACLE DELTA IMPEDANCE
		# global imp_delta_pose1; global imp_delta_vel1; global imp_delta_time1
		# global imp_delta_pose2; global imp_delta_vel2; global imp_delta_time2
		# global imp_delta_pose3; global imp_delta_vel3; global imp_delta_time3
		# # drone1.rad_imp.pose, drone1.rad_imp.vel, drone1.rad_imp.time = swarmlib.impedance_obstacle_delta(drone1, delta1)
		# drone1.update_radius_imp(delta1)
		# # imp_delta_pose1, imp_delta_vel1, imp_delta_time1 = swarmlib.impedance_obstacle_delta(delta1, imp_delta_pose1, imp_delta_vel1, imp_delta_time1)
		# # print 'imp_delta_pose1', imp_delta_pose1
		# # imp_delta_pose2, imp_delta_vel2, imp_delta_time2 = swarmlib.impedance_obstacle_delta(delta2, imp_delta_pose2, imp_delta_vel2, imp_delta_time2)
		# # imp_delta_pose3, imp_delta_vel3, imp_delta_time3 = swarmlib.impedance_obstacle_delta(delta3, imp_delta_pose3, imp_delta_vel3, imp_delta_time3)
		# drone1.sp += drone1.rad_imp.pose
		# # drone1.sp += imp_delta_pose1
		# # drone2_pose_goal += imp_delta_pose2
		# # drone3_pose_goal += imp_delta_pose3


