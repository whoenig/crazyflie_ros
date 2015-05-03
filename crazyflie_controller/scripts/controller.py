#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped, Pose
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
        self.pidX = PID(35, 10, 0.0, -20, 20, "x")
        self.pidY = PID(-35, -10, -0.0, -20, 20, "y")
        self.pidZ = PID(4000, 3000.0, 2000.0, 10000, 60000, "z")
        self.pidYaw = PID(-50.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.state = Controller.Idle
        self.goal = Pose()
        rospy.Subscriber("goal", Pose, self._poseChanged)
        rospy.Service("takeoff", std_srvs.srv.Empty, self._takeoff)
        rospy.Service("land", std_srvs.srv.Empty, self._land)

    def getTransform(self, source_frame, target_frame):
        now = rospy.Time.now()
        success = False
        if self.listener.canTransform(source_frame, target_frame, rospy.Time(0)):
            t = self.listener.getLatestCommonTime(source_frame, target_frame)
            if self.listener.canTransform(source_frame, target_frame, t):
                position, quaternion = self.listener.lookupTransform(source_frame, target_frame, t)
                success = True
            delta = (now - t).to_sec() * 1000 #ms
            if delta > 50:
                rospy.logwarn("Latency: %f ms. Clearing TF Buffer.", delta)
                self.listener.clear()
                rospy.sleep(0.02)
        if success:
            return position, quaternion, t

    def pidReset(self):
        self.pidX.reset()
        self.pidZ.reset()
        self.pidZ.reset()
        self.pidYaw.reset()

    def _poseChanged(self, data):
        self.goal = data

    def _takeoff(self, req):
        rospy.loginfo("Takeoff requested!")
        self.state = Controller.TakingOff
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
                        self.pidReset()
                        self.pidZ.integral = thrust / self.pidZ.ki
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
                self.goal.position.z = 0.05
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
                    targetWorld = PoseStamped()
                    targetWorld.header.stamp = t
                    targetWorld.header.frame_id = "world"
                    targetWorld.pose = self.goal

                    targetDrone = self.listener.transformPose(self.frame, targetWorld)

                    quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)

                    msg = Twist()
                    msg.linear.x = self.pidX.update(0.0, targetDrone.pose.position.x)
                    msg.linear.y = self.pidY.update(0.0, targetDrone.pose.position.y)
                    msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                    msg.angular.z = self.pidYaw.update(0.0, euler[2])
                    self.pubNav.publish(msg)
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
