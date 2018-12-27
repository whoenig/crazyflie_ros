#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path

from math import *
import math
import time
from time import sleep
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import message_filters
import sys
import numpy as np
import serial
from scipy.integrate import odeint
from tf import TransformListener
from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position
from multiprocessing import Process
import os
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


# Main classes ####################################################################
class Swarm_manager():
	def __init__(self,drone_name_list):
		self.drone_name_list = drone_name_list

		drone_object_list = []
		for drone_name in self.drone_name_list:
			drone_object = drone(drone_name)
			drone_object_list.append(drone_object)
		return drone_object_list
	def update_position_for_all(self, drone_object_list):
		for drone_object in drone_object_list:
			drone_object.position()

class Mocap_object: # superclass
	def __init__(self, name):
		self.name = name
		self.tf = '/vicon/'+name+'/'+name
		self.tl = TransformListener()
		self.pose = np.array([0,0,0])
		self.orient = np.array([0,0,0])
		# for velocity:
		sub = message_filters.Subscriber(self.tf, TransformStamped)
		self.cache = message_filters.Cache(sub, 100)
		self.vel = np.array([0,0,0])
	def position(self):
		self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
		position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
		self.pose = np.array(position)
		return np.array(position)
	def orientation(self):
		self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
		position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
		self.orient = get_angles(np.array(quaternion))
		return get_angles(np.array(quaternion))
	def publish_position(self):
		publish_pose(self.pose, self.orient, self.name+"_pose")
	def velocity(self):
		aver_interval = 0.1 # sec
		msg_past = self.cache.getElemAfterTime(self.cache.getLatestTime() - rospy.rostime.Duration(aver_interval))
		msg_now = self.cache.getElemAfterTime(self.cache.getLatestTime())
		if (msg_past is not None) and  (msg_now is not None) and (msg_now.header.stamp != msg_past.header.stamp):
			vel = vel_estimation_TransformStamped(msg_past, msg_now)
			self.vel = vel

class Drone(Mocap_object): # TODO: use superclass mocap_object
	def __init__(self, name, leader = False):
		Mocap_object.__init__(self, name)
		self.leader = leader
		self.sp = np.array([0,0,0])
		self.path = Path()
		self.path_ = Path()
		self.near_obstacle = False
		self.nearest_obstacle = None
		self.rad_imp = radius_impedance_model()      # Obstacle avoidance

		sub_sp = message_filters.Subscriber(self.name+"_sp", PoseStamped)
		self.cache_sp = message_filters.Cache(sub_sp, 100)
		self.vel_sp = np.array([0,0,0])

	def publish_sp(self):
		publish_pose(self.sp, np.array([0,0,0]), self.name+"_sp")
	def publish_path(self, limit=1000):
		publish_path(self.path, self.sp, self.orient, self.name+"_path", limit)
	def publish_path_(self, limit=1000):
		publish_path(self.path_, self.sp, self.orient, self.name+"_path_", limit)
	def fly(self):
		publish_goal_pos(self.sp, 0, "/"+self.name)
	def apply_limits(self, uper_limits, lower_limits):
		np.putmask(self.sp, self.sp >= uper_limits, uper_limits)
		np.putmask(self.sp, self.sp <= lower_limits, lower_limits)
	def update_radius_imp(self, delta):
		if self.rad_imp.inside:
			radius_obstacle_impedance(self)
			self.sp += self.rad_imp.pose
	def velocity_sp(self):
		aver_interval = 0.2 # sec
		if self.cache_sp.getLatestTime() is not None:
			msg_past = self.cache_sp.getElemAfterTime(self.cache_sp.getLatestTime() - rospy.rostime.Duration(aver_interval))
			msg_now = self.cache_sp.getElemAfterTime(self.cache_sp.getLatestTime())
			if (msg_past is not None) and (msg_now is not None) and (msg_now.header.stamp != msg_past.header.stamp):
				vel_sp = vel_estimation_PoseStamped(msg_past, msg_now)
				self.vel_sp = vel_sp


class radius_impedance_model:
	def __init__(self):
		self.inside = False
		self.penetration = None
		self.imp_pose = 0
		self.imp_vel = 0
		self.time_prev = time.time()


# Service functions ###############################################################
def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
	name = cf_name + "/cmd_position"
	msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
	pub = rospy.Publisher(name, Position, queue_size=1)
	pub.publish(msg)
def publish_pose(pose, orient, topic_name):
	msg = msg_def_PoseStamped(pose, orient)
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pub.publish(msg)
def publish_path(path, pose, orient, topic_name, limit=1000):
	msg = msg_def_PoseStamped(pose, orient)
	path.header = msg.header
	path.poses.append(msg)
	if limit>0:
		path.poses = path.poses[-limit:]
	pub = rospy.Publisher(topic_name, Path, queue_size=1)
	pub.publish(path)
def publish_vel(vel, topic_name):
	msg = Twist()
	msg.linear.x = vel[0]
	msg.linear.y = vel[1]
	msg.linear.z = vel[2]
	pub = rospy.Publisher(topic_name, Twist, queue_size=1)
	pub.publish(msg)
def get_angles(message):
	quat = ( message[0], message[1], message[2], message[3] )
	euler = tf.transformations.euler_from_quaternion(quat)
	return euler
def msg_def_crazyflie(pose, yaw):
	worldFrame = rospy.get_param("~worldFrame", "/world")
	msg = Position()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.x = pose[0]
	msg.y = pose[1]
	msg.z = pose[2]
	msg.yaw = yaw
	now = rospy.get_time()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	return msg
def msg_def_PoseStamped(pose, orient):
	worldFrame = "world"
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.header.seq += 1
	return msg
def rotate(origin, drone, human): # rotate drone around point
	"""
	Rotate a point counterclockwise by a given angle around a given origin.
	The angle should be given in radians.
	"""
	ox, oy = origin[0], origin[1]
	px, py = drone.sp[0], drone.sp[1]
	qx = ox + math.cos(human.orientation()[2]) * (px - ox) - math.sin(human.orientation()[2]) * (py - oy)
	qy = oy + math.sin(human.orientation()[2]) * (px - ox) + math.cos(human.orientation()[2]) * (py - oy)
	return np.array([qx, qy, drone.sp[2]])
def centroid_calc(drone1, drone2, drone3): # centroid of triangle
	x_aver = np.array([drone1.sp[0], drone2.sp[0], drone3.sp[0]])
	y_aver = np.array([drone1.sp[1], drone2.sp[1], drone3.sp[1]])
	z_aver = np.array([drone1.sp[2], drone2.sp[2], drone3.sp[2]])
	centroid = np.array([ np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
	return centroid
def vel_estimation_TransformStamped(msg_past, msg_now): # from two TransformStamped messages
	x_now = msg_now.transform.translation.x
	x_past = msg_past.transform.translation.x
	y_now = msg_now.transform.translation.y
	y_past = msg_past.transform.translation.y
	z_now = msg_now.transform.translation.z
	z_past = msg_past.transform.translation.z
	time_now = msg_now.header.stamp.to_sec()
	time_past = msg_past.header.stamp.to_sec()
	vel_x = (x_now-x_past)/(time_now-time_past)
	vel_y = (y_now-y_past)/(time_now-time_past)
	vel_z = (z_now-z_past)/(time_now-time_past)
	vel = np.array([vel_x, vel_y, vel_z])
	return vel
def vel_estimation_PoseStamped(msg_past, msg_now): # from two TransformStamped messages
	x_now = msg_now.pose.position.x
	x_past = msg_past.pose.position.x
	y_now = msg_now.pose.position.y
	y_past = msg_past.pose.position.y
	z_now = msg_now.pose.position.z
	z_past = msg_past.pose.position.z
	time_now = msg_now.header.stamp.to_sec()
	time_past = msg_past.header.stamp.to_sec()
	vel_x = (x_now-x_past)/(time_now-time_past)
	vel_y = (y_now-y_past)/(time_now-time_past)
	vel_z = (z_now-z_past)/(time_now-time_past)
	vel = np.array([vel_x, vel_y, vel_z])
	return vel



# Obstacle avoidance functions #######################################################
def update_obstacle(drone, obstacle, R):
	# obstacle_pose = obstacle.position()[:2]
	# drone_pose = drone.sp[:2]
	dist = np.linalg.norm(obstacle.position()[:2]-drone.sp[:2]) # in 2D
	if dist<R:
		updated_pose = quad_prog_circle(drone.sp, obstacle.position(), R)
		drone.near_obstacle = True
		drone.nearest_obstacle = obstacle
		drone.rad_imp.inside = True
		drone.rad_imp.penetration = updated_pose - drone.sp[:2]
	else:
		# updated_pose = drone_pose
		drone.near_obstacle = False
		drone.nearest_obstacle = None
		drone.rad_imp.inside = False
		drone.rad_imp.penetration = None

		drone.rad_imp.imp_pose = 0
		drone.rad_imp.imp_vel = np.linalg.norm(drone.vel_sp[:2]) # 0
		drone.rad_imp.time_prev = time.time()

	return drone



# Obstacle avoidance functions #######################################################
def pose_update_obstacle_circle(drone, R):
	updated_pose = quad_prog_circle(drone.sp, drone.nearest_obstacle.position(), R)
	drone.sp = np.append(updated_pose, drone.sp[2])
	return drone




# # Obstacle avoidance functions #######################################################
# def pose_update_obstacle(drone, obstacle, R):
# 	obstacle_pose = obstacle.position()[:2]
# 	drone_pose = drone.sp[:2]
# 	dist = np.linalg.norm(obstacle_pose-drone_pose)
# 	if dist<R:
# 		updated_pose = quad_prog_circle(drone_pose, obstacle_pose, R)
# 		drone.obstacle_update_status = [True, obstacle.name]
# 		drone.rad_imp.inside = True
# 	else:
# 		updated_pose = drone_pose
# 		drone.obstacle_update_status = [False, None]
# 		drone.rad_imp.inside = False
# 	drone.rad_imp.penetration = updated_pose - drone_pose
# 	# delta = updated_pose - drone_pose
# 	drone.sp = np.append(updated_pose, drone.sp[2])

# 	return drone#, delta

def quad_prog_circle(drone_pose, obstacle_pose, R):
	drone_pose = drone_pose[:2]          # in 2D
	obstacle_pose = obstacle_pose[:2]    # in 2D
	eq1 = np.array([ [obstacle_pose[0],1], [drone_pose[0],1] ])
	eq2 = np.array([obstacle_pose[1],drone_pose[1]])

	line_equation = np.linalg.solve(eq1, eq2)
	k = line_equation[0]
	b = line_equation[1]

	a_ = k**2+1
	b_ = 2*k*b  - 2*k*obstacle_pose[1] -2*obstacle_pose[0]
	c_ = obstacle_pose[1]**2 - R**2 + obstacle_pose[0]**2 - 2*b*obstacle_pose[1] + b**2

	D = (b_**2) - (4*a_*c_)
	if D>0:
		x_1 = (-b_-sqrt(D))/(2*a_)
		x_2 =  (-b_+sqrt(D))/(2*a_)

	y_1 = k * x_1 + b
	y_2 = k * x_2 + b

	point1 = np.array([ x_1, y_1])
	point2 = np.array([ x_2, y_2])

	dist_point1 = np.linalg.norm(point1 - drone_pose)
	dist_point2 = np.linalg.norm(point2 - drone_pose)

	if dist_point1 < dist_point2:
		updated_pose = point1
	else:
		updated_pose = point2

	return updated_pose



















def Pendulum(state, t, M):
	theta, omega = state
	J = 1.; b = 10.; k = 0.
	dydt = [omega, (M - b*omega - k*np.sin(theta)) / J ]
	return dydt

# theta_from_pose returns angle between 2 vectors: X and [drone_pose-obstacle_pose]' in XY-plane
def theta_from_pose(drone_pose, obstacle_pose):
	# #[0, 2pi] - range
	# if drone_pose[1] >= obstacle_pose[1]:
	# 	theta = acos( (drone_pose[0]-obstacle_pose[0]) / np.linalg.norm(drone_pose[:2] - obstacle_pose[:2]) ) 
	# else:
	# 	theta = 2*pi - acos( (drone_pose[0]-obstacle_pose[0]) / np.linalg.norm(drone_pose[:2] - obstacle_pose[:2]) )
	theta = np.sign(drone_pose[1]-obstacle_pose[1]) * acos( (drone_pose[0]-obstacle_pose[0]) / np.linalg.norm(drone_pose[:2] - obstacle_pose[:2]) ) # [-pi,pi] - range
	return theta


# THETA OBSTACLE IMPEDANCE
def impedance_obstacle_theta(theta, imp_theta_prev, imp_omega_prev, time_prev):
	M_coeff = 10 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	M = - sin(imp_theta_prev - theta) * M_coeff
	state0 = [imp_theta_prev, imp_omega_prev]
	state = odeint(Pendulum, state0, t, args=(M,))
	state = state[1]

	imp_theta = state[0]
	imp_omega = state[1]
	return imp_theta, imp_omega, time_prev


def obstacle_status(obstacle_pose_input, drone_pose_sp, imp_pose_from_theta, human_pose, R, flew_in, flew_out):
	obstacle_pose = np.array([ obstacle_pose_input[0], obstacle_pose_input[1]  ])
	drone_sp = np.array([   drone_pose_sp[0] , drone_pose_sp[1]  ])
	dist = np.linalg.norm(obstacle_pose-drone_sp)
	if imp_pose_from_theta is not None:
		drone_imp = np.array([   imp_pose_from_theta[0] , imp_pose_from_theta[1]  ])
		d_theta = theta_from_pose(drone_sp, obstacle_pose) - theta_from_pose(drone_imp, obstacle_pose)
	else:
		d_theta = pi
	#S = sin(d_theta)
	if dist<R+0.03:
		# the drone is near the obstacle
		flew_in += 1
		flew_out = 0
	#elif dist>R and (S > 0 and S < 1):
	#elif dist>R and np.linalg.norm(object_pose_input-human_pose_input)<1.1:
	elif dist>R and abs( d_theta ) < pi/3.:
		print "flew_out: "+"dist="+str(dist>R)+", d_theta="+str(180/pi*d_theta)
		flew_in = 0
		flew_out += 1
	return flew_in, flew_out





# DRONE ANGULAR VELOCITY CALCULATION
drone_time_array = np.ones(10)
drone_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def drone_w(drone_pose, R):
	for i in range(len(drone_time_array)-1):
		drone_time_array[i] = drone_time_array[i+1]
	drone_time_array[-1] = time.time()

	for i in range(len(drone_pose_array[0])-1):
		drone_pose_array[0][i] = drone_pose_array[0][i+1]
		drone_pose_array[1][i] = drone_pose_array[1][i+1]
		drone_pose_array[2][i] = drone_pose_array[2][i+1]
	drone_pose_array[0][-1] = drone_pose[0]
	drone_pose_array[1][-1] = drone_pose[1]
	drone_pose_array[2][-1] = drone_pose[2]

	vel_x = (drone_pose_array[0][-1]-drone_pose_array[0][0])/(drone_time_array[-1]-drone_time_array[0])
	vel_y = (drone_pose_array[1][-1]-drone_pose_array[1][0])/(drone_time_array[-1]-drone_time_array[0])
	vel_z = (drone_pose_array[2][-1]-drone_pose_array[2][0])/(drone_time_array[-1]-drone_time_array[0])

	drone_vel = np.array( [vel_x, vel_y, vel_z] )
	# drone_vel_n = np.dot(drone_vel, R)/(np.linalg.norm(R)**2) * R
	# drone_vel_t = drone_vel - drone_vel_n
	
	drone_w = np.cross(drone_vel, R)

	return drone_w, drone_vel




























# TACTILE ########################################################################################
prev_pattern_time = time.time()
pattern_duration = 0
area_pattern = False
left_right_pattern = False
prev_pattern = 'left_right_pattern'




#___________________________________________________________________________________________________
duration = 4
high_lev = 9
empty = np.zeros((5, 1, 2))
empty = (
[0, 1],
[0, 1],
[0, 1],
[0, 1],
[0, 1])

L = np.zeros((5, 1, 2)) #5,7,9
L = (
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration],
[0, duration])

R = np.zeros((5, 1, 2)) #5,7,9
R = (
[0, duration],
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration])

MR1 = np.zeros((5, 1, 2))
MR1 = (
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration])
MR2 = np.zeros((5, 1, 2))
MR2 = (
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration])
MR3 = np.zeros((5, 1, 2))
MR3 = (
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration])

ML1 = np.zeros((5, 1, 2))
ML1 = (
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration])
ML2 = np.zeros((5, 1, 2))
ML2 = (
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration])
ML3 = np.zeros((5, 1, 2))
ML3 = (
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration])

M1 = np.zeros((5, 1, 2))
M1 = (
[0, duration],
[high_lev, duration*2],
[high_lev, duration*2],
[high_lev, duration*2],
[0, duration])
#________________________________________________________________________________________________________

#Decreasing distance (extended state)
P9=[]
P9.append(np.copy(R))
P10=[]
P10.append(np.copy(L))

P11=[]
P11.append(np.copy(MR1))
P11.append(np.copy(MR2))
P11.append(np.copy(MR3))
P12=[]
P12.append(np.copy(ML1))
P12.append(np.copy(ML2))
P12.append(np.copy(ML3))
P13=[]
P13.append(np.copy(M1))




from std_msgs.msg import String
str_msg = String()

pub = rospy.Publisher('pattern_topic', String, queue_size=10)

def patter_publisher(pattern_type):
	str_msg.data = pattern_type
	pub.publish(str_msg)




def tactile_patterns(drone1, drone2, drone3, human, l, move_right, move_left):
		global prev_pattern_time
		global pattern_duration
		global area_pattern
		global left_right_pattern
		global prev_pattern
		# AREA calc
		# https://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
		x = np.array([drone1.sp[0], drone2.sp[0], drone3.sp[0]])
		y = np.array([drone1.sp[1], drone2.sp[1], drone3.sp[1]])
		def PolyArea(x,y):
			return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
		# print 'PolyArea(x,y)', PolyArea(x,y)
		default_area = l*l*math.sqrt(3)/4
		if (time.time()-prev_pattern_time)>(pattern_duration):

			patter_publisher('')

			extended = False
			contracted = False
			if PolyArea(x,y)>default_area*1.1 or (drone2.sp[1] - drone3.sp[1])>l*1.075:
				extended = True
			elif PolyArea(x,y)<default_area*0.9:
				contracted = True
			# centroid = centroid_calc(drone1, drone2, drone3)

			if extended and move_left:
				print 'pattern extended RIGHT'
				pattern_duration = Send(P9)
				patter_publisher('extended_right')
			if extended and move_right:
				print 'pattern extended LEFT'
				pattern_duration = Send(P10)
				patter_publisher('extended_left')


			if contracted and move_right:
				print 'pattern contracted RIGHT'
				pattern_duration = Send(P11)
				patter_publisher('contracted_right')
			if contracted and move_left:
				print 'pattern contracted LEFT'
				pattern_duration = Send(P12)
				patter_publisher('contracted_left')


			if contracted or extended:
				prev_pattern_time = time.time()



			# # Patterns manager
			# if default_area*0.80<PolyArea(x,y)<default_area*1.20:
			# 	area_pattern = False
			# else:
			# 	area_pattern = True

			# centroid = centroid_calc(drone1, drone2, drone3)
			# if -0.02<(centroid[1] - human.pose[1])<0.02:
			# 	left_right_pattern = False
			# else:
			# 	left_right_pattern = True

			# # a=1, lr=1
			# if area_pattern and left_right_pattern:
			# 	if prev_pattern == "area_pattern":
			# 		# Start left_right pattern
			# 		centroid = centroid_calc(drone1, drone2, drone3)
			# 		if PolyArea(x,y)<default_area:
			# 			contracted = True
			# 			extended = False
			# 		else:
			# 			contracted = False
			# 			extended = True
			# 		if (centroid[1] - human.pose[1])>0.02 and contracted:
			# 			print 'pattern RIGHT'
			# 			pattern_duration = right_pattern()
			# 			prev_pattern_time = time.time()
			# 		if (centroid[1] - human.pose[1])>0.02 and extended:
			# 			print 'pattern LEFT'
			# 			pattern_duration = left_pattern()
			# 			prev_pattern_time = time.time()
			# 		if (centroid[1] - human.pose[1])<-0.02 and contracted:
			# 			print 'pattern LEFT'
			# 			pattern_duration = left_pattern()
			# 			prev_pattern_time = time.time()
			# 		if (centroid[1] - human.pose[1])<-0.02 and extended:
			# 			print 'pattern RIGHT'
			# 			pattern_duration = right_pattern()
			# 			prev_pattern_time = time.time()

			# 		prev_pattern = 'left_right_pattern'


			# 	else:
			# 		# Start area pattern
			# 		if PolyArea(x,y)>default_area*1.20:
			# 			print "extended, area = ", PolyArea(x,y)
			# 			pattern_duration = extended_pattern()
			# 			prev_pattern_time = time.time()
			# 		elif default_area*0.65<PolyArea(x,y)<default_area*0.80:
			# 			print "contracted, area = ", PolyArea(x,y)
			# 			pattern_duration = contracted_pattern()
			# 			prev_pattern_time = time.time()
			# 		elif PolyArea(x,y)<default_area*0.65:
			# 			print "too contracted, area = ", PolyArea(x,y)
			# 			pattern_duration = too_contracted_pattern()
			# 			prev_pattern_time = time.time()
			# 		prev_pattern = "area_pattern"

			# # a=1 lr=0, a=0 lr=1

			# if area_pattern and not left_right_pattern:
			# 	# Start area pattern
			# 	if PolyArea(x,y)>default_area*1.20:
			# 		print "extended, area = ", PolyArea(x,y)
			# 		pattern_duration = extended_pattern()
			# 		prev_pattern_time = time.time()
			# 	elif default_area*0.65<PolyArea(x,y)<default_area*0.80:
			# 		print "contracted, area = ", PolyArea(x,y)
			# 		pattern_duration = contracted_pattern()
			# 		prev_pattern_time = time.time()
			# 	elif PolyArea(x,y)<default_area*0.65:
			# 		print "too contracted, area = ", PolyArea(x,y)
			# 		pattern_duration = too_contracted_pattern()
			# 		prev_pattern_time = time.time()
			# if left_right_pattern and not area_pattern:
			# 	# Start left_right pattern
			# 	# print "only left_right_pattern"
			# 	centroid = centroid_calc(drone1, drone2, drone3)
			# 	if PolyArea(x,y)<default_area:
			# 		contracted = True
			# 		extended = False
			# 	else:
			# 		contracted = False
			# 		extended = True
			# 	if (centroid[1] - human.pose[1])>0.02 and contracted:
			# 		print 'pattern RIGHT'
			# 		pattern_duration = right_pattern()
			# 		prev_pattern_time = time.time()
			# 	if (centroid[1] - human.pose[1])>0.02 and extended:
			# 		print 'pattern LEFT'
			# 		pattern_duration = left_pattern()
			# 		prev_pattern_time = time.time()
			# 	if (centroid[1] - human.pose[1])<-0.02 and contracted:
			# 		print 'pattern LEFT'
			# 		pattern_duration = left_pattern()
			# 		prev_pattern_time = time.time()
			# 	if (centroid[1] - human.pose[1])<-0.02 and extended:
			# 		print 'pattern RIGHT'
			# 		pattern_duration = right_pattern()
			# 		prev_pattern_time = time.time()


			# if PolyArea(x,y)>default_area*1.20:
			# 	print "extended, area = ", PolyArea(x,y)
			# 	pattern_duration = extended_pattern()
			# 	prev_pattern_time = time.time()
			# elif default_area*0.65<PolyArea(x,y)<default_area*0.80:
			# 	print "contracted, area = ", PolyArea(x,y)
			# 	pattern_duration = contracted_pattern()
			# 	prev_pattern_time = time.time()
			# elif PolyArea(x,y)<default_area*0.65:
			# 	print "too contracted, area = ", PolyArea(x,y)
			# 	pattern_duration = too_contracted_pattern()
			# 	prev_pattern_time = time.time()


			# centroid = centroid_calc(drone1, drone2, drone3)
			# if (centroid[1] - human.pose[1])>0.02:
			# 	print 'pattern RIGHT'
			# 	pattern_duration = right_pattern()
			# 	prev_pattern_time = time.time()
			# if (centroid[1] - human.pose[1])<-0.02:
			# 	print 'pattern LEFT'
			# 	pattern_duration = left_pattern()
			# 	prev_pattern_time = time.time()

			# centroid = centroid_calc(drone1, drone2, drone3)
			# if PolyArea(x,y)<default_area:
			# 	contracted = True
			# 	extended = False
			# else:
			# 	contracted = False
			# 	extended = True
			# if (centroid[1] - human.pose[1])>0.02 and contracted:
			# 	print 'pattern RIGHT'
			# 	pattern_duration = right_pattern()
			# 	prev_pattern_time = time.time()
			# if (centroid[1] - human.pose[1])>0.02 and extended:
			# 	print 'pattern LEFT'
			# 	pattern_duration = left_pattern()
			# 	prev_pattern_time = time.time()
			# if (centroid[1] - human.pose[1])<-0.02 and contracted:
			# 	print 'pattern LEFT'
			# 	pattern_duration = left_pattern()
			# 	prev_pattern_time = time.time()
			# if (centroid[1] - human.pose[1])<-0.02 and extended:
			# 	print 'pattern RIGHT'
			# 	pattern_duration = right_pattern()
			# 	prev_pattern_time = time.time()


def extended_pattern():
	# 1st column is intensity levels between 0-9
	#2nd column is timing between 0-9
	time = 4
	C = np.zeros((5, 1, 2))
	C = (
	[9, time],
	[0, time],
	[0, time],
	[0, time],
	[8, time])
	P= []
	P.append(np.copy(C))
	pattern_duration = Send(P)
	return pattern_duration
def contracted_pattern():
	# 1st column is intensity levels between 0-9
	#2nd column is timing between 0-999
	time = 3
	C = np.zeros((5, 1, 2))
	C = (
	[0, time],
	[0, time],
	[9, time],
	[0, time],
	[0, time])
	P= []
	P.append(np.copy(C))
	pattern_duration = Send(P)
	return pattern_duration
def too_contracted_pattern():
	# 1st column is intensity levels between 0-9
	#2nd column is timing between 0-999
	time = 5
	C = np.zeros((5, 1, 2))
	C = (
	[0, time],
	[9, time],
	[9, time],
	[9, time],
	[0, time])
	P= []
	P.append(np.copy(C))
	pattern_duration = Send(P)
	return pattern_duration




F = np.zeros((5, 1, 2)) #5,7,9
F = (
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration],
[0, duration])
F1 = np.zeros((5, 1, 2))
F1 = (
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration])
F2 = np.zeros((5, 1, 2))
F2 = (
[0, duration],
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration])

F_ = np.zeros((5, 1, 2))
F_ = (
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration])
F__ = np.zeros((5, 1, 2))
F__ = (
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration])

def right_pattern():
	P7=[]
	P7.append(np.copy(F))
	P7.append(np.copy(empty))#P7.append(np.copy(F_))
	P7.append(np.copy(F1))
	P7.append(np.copy(empty))#P7.append(np.copy(F__))
	P7.append(np.copy(F2))
	pattern_duration = Send(P7)
	return pattern_duration
def left_pattern():
	P8=[]
	P8.append(np.copy(F2))
	P8.append(np.copy(empty))#P8.append(np.copy(F__))
	P8.append(np.copy(F1))
	P8.append(np.copy(empty))#P8.append(np.copy(F_))
	P8.append(np.copy(F))
	pattern_duration = Send(P8)
	return pattern_duration


def startXbee():
	global serial_port
	# serial_port = serial.Serial('/dev/ttyUSB0', 9600)
	serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)
def Send(Mat):
	max = np.zeros(1)
	max[0] = 0
	for i in range(len(Mat)):
		max[0] = max[0] + np.amax(Mat[i][:,1])*100
	serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)
	t =matrix_send(Mat)
	serial_port.close()

	t2=(max[0] / 1000.0)+t
	return t2
def matrix_send(Matr):
	X = np.zeros((5, 1, 2))
	X = (
		[0, 0],
		[0, 0],
		[0, 0],
		[0, 0],
		[0, 0])

	matrix = np.copy(Matr)


	for i in range(len(matrix)):
		Z = np.copy(matrix[i])
		for k in range(len(Z)):
			item = '%s\r' % Z[k][0]
			serial_port.write(item.encode())

			# print("raw1", Z[k][0])
		for n in range(len(Z)):
			item = '%s\r' % Z[n][1]
			serial_port.write(item.encode())
			# print("raw1", Z[n][1])


	for i in range (5- len(matrix)):

		Z = np.copy(X)
		for k in range(len(Z)):
			item = '%s\r' % Z[k][0]
			serial_port.write(item.encode())

			# print("raw1", Z[k][0])
		for n in range(len(Z)):
			item = '%s\r' % Z[n][1]
			serial_port.write(item.encode())
			# print("raw1", Z[n][1])
	


	return 0.1*(5- len(matrix))






def recorder(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on):
	if tacile_glove_on:
		user_name = user_name+'_with_glove'
	else:
		user_name = user_name+'_wo_glove'
	obstacle_topics = ''
	for i in range(len(obstacle_list)):
		obstacle_topics = obstacle_topics +" /vicon/"+obstacle_list[i]+"/"+obstacle_list[i]
	os.system("rosbag record -o /home/drone/SwarmTouchData/"+user_name+" /vicon/"+cf1_name+"/"+cf1_name+" /vicon/"+cf2_name+"/"+cf2_name+" /vicon/"+cf3_name+"/"+cf3_name+" /vicon/"+human_name+"/"+human_name+obstacle_topics+" /"+cf1_name+"_sp"+" /"+cf2_name+"_sp"+" /"+cf3_name+"_sp"+' /pattern_topic')
	# os.system("rosbag record -o /home/drone/SwarmTouchData/"+user_name + " -a")
	# os.system("rosbag record -a")
def start_recording(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on):
	pose_recording = Process(target=recorder, args=(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on,))
	pose_recording.start()
def killer_of_recorder():
	print 'killing the recorder'
	node_list = os.popen("rosnode list").read()
	print node_list
	for i in range(len(node_list)):
		if node_list[i:i+5] == '/reco':
			range_from = i
			range_to = i + 27
			break
	os.system('rosnode kill '+ node_list[range_from:range_to])








# IMPEDANCE ####################################################################################
# HUMAN VELOCITY CALCULATION
hum_time_array = np.ones(10)
hum_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def hum_vel(human_pose):

	for i in range(len(hum_time_array)-1):
		hum_time_array[i] = hum_time_array[i+1]
	hum_time_array[-1] = time.time()

	for i in range(len(hum_pose_array[0])-1):
		hum_pose_array[0][i] = hum_pose_array[0][i+1]
		hum_pose_array[1][i] = hum_pose_array[1][i+1]
		hum_pose_array[2][i] = hum_pose_array[2][i+1]
	hum_pose_array[0][-1] = human_pose[0]
	hum_pose_array[1][-1] = human_pose[1]
	hum_pose_array[2][-1] = human_pose[2]

	vel_x = (hum_pose_array[0][-1]-hum_pose_array[0][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_y = (hum_pose_array[1][-1]-hum_pose_array[1][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_z = (hum_pose_array[2][-1]-hum_pose_array[2][0])/(hum_time_array[-1]-hum_time_array[0])

	hum_vel = np.array( [vel_x, vel_y, vel_z] )

	return hum_vel
# HUMAN IMPEDANCE
def MassSpringDamper(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 12.6
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]
def impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	if hum_vel[0]<0:
		hum_vel[0] = - hum_vel[0]
	F = - hum_vel * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	state0_z = [imp_pose_prev[2], imp_vel_prev[2]]
	state_z = odeint(MassSpringDamper, state0_z, t, args=(F[2],))
	state_z = state_z[1]

	imp_pose = np.array( [state_x[0], state_y[0], state_z[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1], state_z[1]] )

	return imp_pose, imp_vel, time_prev



def MassSpringDamper_rad_imp(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 20.0
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]
# Radius OBSTACLE IMPEDANCE
def radius_obstacle_impedance(drone):
	F_coeff = 12 # 7
	time_step = time.time() - drone.rad_imp.time_prev
	drone.rad_imp.time_prev = time.time()
	t = [0. , time_step]
	F = np.linalg.norm(drone.rad_imp.penetration) * F_coeff

	state0 = [drone.rad_imp.imp_pose, drone.rad_imp.imp_vel]
	state = odeint(MassSpringDamper_rad_imp, state0, t, args=(F,))
	state = state[1]

	imp_pose = state[0]
	imp_vel = state[1]

	drone.rad_imp.imp_pose = imp_pose
	drone.rad_imp.imp_vel = imp_vel

	# step towartd the center TODO: male beauty
	v = - drone.sp[:2] + drone.nearest_obstacle.pose[:2]
	v = v/np.linalg.norm(v)
	v = v*drone.rad_imp.imp_pose
	drone.sp[0] = drone.sp[0] + v[0]
	drone.sp[1] = drone.sp[1] + v[1]


	return drone 











def pub_circle_traj(x0,y0,z0,r,i):
	# i=0
	# while time_delay<delay:
	x1 = x0 + r*sin(i*1.75*pi/360) # 1
	y1 = y0 + r*cos(i*1.75*pi/360) # 1
	z1 = z0
	drone10_pose_goal = np.array([ x1,y1,z1 ])
	x2 = x0 + r*sin(i*1.75*pi/360+pi) # 2
	y2 = y0 + r*cos(i*1.75*pi/360+pi) # 2
	z2 = z0
	drone11_pose_goal = np.array([ x2,y2,z2 ])
	i = i+1
	publish_pose(drone10_pose_goal, 0, "drone10_pose_goal")
	publish_pose(drone11_pose_goal, 0, "drone11_pose_goal")
	return i, drone10_pose_goal, drone11_pose_goal
