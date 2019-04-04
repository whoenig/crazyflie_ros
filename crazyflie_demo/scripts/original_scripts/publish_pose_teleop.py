#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from math import fabs

lastData = None

def joyChanged(data):
    global lastData
    lastData = data
    # print(data)

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    joy_topic = rospy.get_param("~joy_topic", "joy")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    yaw = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)
    rospy.Subscriber(joy_topic, Joy, joyChanged)

    while not rospy.is_shutdown():
        global lastData
        if lastData != None:
            if fabs(lastData.axes[1]) > 0.1:
                msg.pose.position.z += lastData.axes[1] / r / 2
            if fabs(lastData.axes[4]) > 0.1:
                msg.pose.position.x += lastData.axes[4] / r * 1
            if fabs(lastData.axes[3]) > 0.1:
                msg.pose.position.y += lastData.axes[3] / r * 1
            if fabs(lastData.axes[0]) > 0.1:
                yaw += lastData.axes[0] / r * 2
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            # print(pose)
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
