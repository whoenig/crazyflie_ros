#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped
import std_srvs.srv
from pid import PID

class Controller:
    Idle = 0
    Automatic = 1
    TakingOff = 2
    Landing = 3

    def __init__(self, frame):
        self.frame = frame
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.listener = TransformListener()
        self.state = Controller.Idle
        self.goal = PoseStamped()
        rospy.Subscriber("goal", PoseStamped, self._poseChanged)
        rospy.Service("takeoff", std_srvs.srv.Empty, self._takeoff)
        rospy.Service("land", std_srvs.srv.Empty, self._land)

        self.MAX_ANGLE = np.pi/4
        self.MAX_THRUST = 65000.0
        self.MASS_THRUST = 42000.0
        self.kp = 20000.0
        self.kd = 10000.0
        self.ki = 0.0 #2000.0

        self.old_position = np.array([0.0, 0.0, 0.0])
        self.current_r_error_integration = np.array([0.0, 0.0, 0.0])

        self.pidYaw = PID(-50.0, 0.0, 0.0, -200.0, 200.0, "yaw")

    def getTransform(self, source_frame, target_frame):
        now = rospy.Time.now()
        success = False
        if self.listener.canTransform(source_frame, target_frame, rospy.Time(0)):
            t = self.listener.getLatestCommonTime(source_frame, target_frame)
            if self.listener.canTransform(source_frame, target_frame, t):
                position, quaternion = self.listener.lookupTransform(source_frame, target_frame, t)
                success = True
            delta = (now - t).to_sec() * 1000 #ms
            if delta > 25:
                rospy.logwarn("Latency: %f ms.", delta)
            #     self.listener.clear()
            #     rospy.sleep(0.02)
        if success:
            return position, quaternion, t

    # def pidReset(self):
    #     self.pidX.reset()
    #     self.pidZ.reset()
    #     self.pidZ.reset()
    #     self.pidYaw.reset()

    def _poseChanged(self, data):
        self.goal = data

    def _takeoff(self, req):
        rospy.loginfo("Takeoff requested!")
        # self.state = Controller.TakingOff
        self.state = Controller.Automatic
        return std_srvs.srv.EmptyResponse()

    def _land(self, req):
        rospy.loginfo("Landing requested!")
        self.state = Controller.Landing
        return std_srvs.srv.EmptyResponse()

    def run(self):
        thrust = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.state == Controller.TakingOff:
                r = self.getTransform("/world", self.frame)
                if r:
                    position, quaternion, t = r

                    if position[2] > 0.05 or thrust > 50000:
                        # self.pidReset()
                        # self.pidZ.integral = thrust / self.pidZ.ki
                        self.current_r_error_integration = thrust / self.ki
                        self.targetZ = 0.5
                        self.state = Controller.Automatic
                        thrust = 0
                    else:
                        thrust += 100
                        msg = Twist()
                        msg.linear.z = thrust
                        self.pubNav.publish(msg)
                else:
                    rospy.logerr("Could not transform from /world to %s.", self.frame)

            if self.state == Controller.Landing:
                self.goal.pose.position.z = 0.05
                r = self.getTransform("/world", self.frame)
                if r:
                    position, quaternion, t = r
                    if position[2] <= 0.1:
                        self.state = Controller.Idle
                        msg = Twist()
                        self.pubNav.publish(msg)
                else:
                    rospy.logerr("Could not transform from /world to %s.", self.frame)

            if self.state == Controller.Automatic or self.state == Controller.Landing:
                # transform target world coordinates into local coordinates
                r = self.getTransform("/world", self.frame)
                if r:
                    position, quaternion, t = r
                    dt = 0.01 # TODO

                    position = np.array(position)
                    current_velocity = (position - self.old_position) / dt
                    self.old_position = position

                    q0 = quaternion[3]
                    q1 = quaternion[0]
                    q2 = quaternion[1]
                    q3 = quaternion[2]
                    current_z_axis = np.array([
                        2*(q1*q3+q0*q2),
                        2*(q2*q3-q0*q1),
                        q0*q0+q3*q3-q1*q1-q2*q2])
                    current_z_axis = current_z_axis/np.linalg.norm(current_z_axis)
                    # print(current_z_axis)

                    target_position = np.array([
                        self.goal.pose.position.x,
                        self.goal.pose.position.y,
                        self.goal.pose.position.z])
                    target_quaternion = (
                        self.goal.pose.orientation.x,
                        self.goal.pose.orientation.y,
                        self.goal.pose.orientation.z,
                        self.goal.pose.orientation.w)
                    target_euler = tf.transformations.euler_from_quaternion(target_quaternion)
                    current_euler = tf.transformations.euler_from_quaternion(quaternion)

                    current_r_error = target_position - position
                    current_r_error_unit = current_r_error/np.linalg.norm(current_r_error)

                    self.current_r_error_integration += current_r_error*dt
                    if np.linalg.norm(self.current_r_error_integration) >= 6:
                        self.current_r_error_integration = 6.0*self.current_r_error_integration/np.linalg.norm(self.current_r_error_integration)

                    r_error_norm = np.linalg.norm(current_r_error)
                    if r_error_norm >= 5.0:
                        target_velocity = 1.3*current_r_error_unit  # The velocity in the target position direction is 1 m/s
                    else:
                        target_velocity = 1.3*(r_error_norm/5.0)*current_r_error_unit

                    # compute z-axis-desired
                    z_axis_desired = self.MASS_THRUST*np.array([0.0, 0.0, 1.0]) + self.kp*current_r_error + self.kd*(target_velocity - current_velocity) + self.ki*self.current_r_error_integration
                    angle = np.arccos(np.dot(z_axis_desired/np.linalg.norm(z_axis_desired), np.array([0.0, 0.0, 1.0]) ))
                    kp = self.kp
                    kd = self.kd
                    ki = self.ki

                    while angle >= self.MAX_ANGLE:
                        kp *= 0.9
                        kd *= 0.9
                        ki *= 0.9
                        z_axis_desired = self.MASS_THRUST*np.array([0.0, 0.0, 1.0]) + kp*current_r_error + kd*(target_velocity - current_velocity) + ki*self.current_r_error_integration
                        angle = np.arccos(np.dot(z_axis_desired/np.linalg.norm(z_axis_desired), np.array([0.0, 0.0, 1.0]) ))
                    z_axis_desired_unit = z_axis_desired/np.linalg.norm(z_axis_desired)

                    # control
                    thrust = np.dot(z_axis_desired, current_z_axis)
                    # print(thrust)
                    if thrust < 0:
                        thrust = 0.0
                    if thrust > self.MAX_THRUST:
                        thrust = self.MAX_THRUST

                    x_axis_desired = np.cross(np.array([-math.sin(target_euler[2]), math.cos(target_euler[2]), 0.0]), z_axis_desired_unit)
                    # x_axis_desired = np.cross(np.array([0.0, 1.0, 0.0]), z_axis_desired_unit)
                    x_axis_desired = x_axis_desired/np.linalg.norm(x_axis_desired)
                    y_axis_desired = np.cross(z_axis_desired_unit, x_axis_desired)

                    pitch_angle = np.arcsin(-1.0*x_axis_desired[2])*180.0/np.pi
                    yaw_angle = np.arctan2(x_axis_desired[1], x_axis_desired[0]) #*180.0/np.pi
                    roll_angle = np.arctan2(y_axis_desired[2], z_axis_desired_unit[2])*180.0/np.pi


                    msg = Twist()
                    msg.linear.x = pitch_angle
                    msg.linear.y = roll_angle
                    msg.linear.z = thrust
                    # msg.angular.z = yaw_angle
                    msg.angular.z = self.pidYaw.update(current_euler[2], yaw_angle)
                    self.pubNav.publish(msg)

                    print("{} {} {}".format(current_euler[2], yaw_angle, msg.angular.z))
                else:
                    rospy.logerr("Could not transform from /world to %s.", self.frame)

            if self.state == Controller.Idle:
                msg = Twist()
                self.pubNav.publish(msg)

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    frame = rospy.get_param("~frame")
    controller = Controller(frame)
    controller.run()
