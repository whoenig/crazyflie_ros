#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
#from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Pose

class Demo():
    def __init__(self, goals):
        rospy.init_node('demo', anonymous=True)
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', Pose, queue_size=1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 0

    def run(self):

        self.listener.waitForTransform("/world", self.frame, rospy.Time(), rospy.Duration(5.0))
        while not rospy.is_shutdown():
            goal = Pose()
            goal.position.x = self.goals[self.goalIndex][0]
            goal.position.y = self.goals[self.goalIndex][1]
            goal.position.z = self.goals[self.goalIndex][2]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
            goal.orientation.x = quaternion[0]
            goal.orientation.y = quaternion[1]
            goal.orientation.z = quaternion[2]
            goal.orientation.w = quaternion[3]

            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime("/world", self.frame)
            if self.listener.canTransform("/world", self.frame, t):
                position, quaternion = self.listener.lookupTransform("/world", self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if     math.fabs(position[0] - self.goals[self.goalIndex][0]) < 0.3 \
                   and math.fabs(position[1] - self.goals[self.goalIndex][1]) < 0.3 \
                   and math.fabs(position[2] - self.goals[self.goalIndex][2]) < 0.3 \
                   and math.fabs(rpy[2] - self.goals[self.goalIndex][3]) < math.radians(10) \
                   and self.goalIndex < len(self.goals) - 1:
                        rospy.sleep(self.goals[self.goalIndex][4])
                        self.goalIndex += 1
