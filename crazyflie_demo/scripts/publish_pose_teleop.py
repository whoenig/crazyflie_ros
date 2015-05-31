#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

lastData = None

def joyChanged(data):
    global lastData
    lastData = data
    # print(data)

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    joy_topic = rospy.get_param("~joy_topic", "joy")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    yaw = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, Pose, queue_size=1)
    rospy.Subscriber(joy_topic, Joy, joyChanged)

    while not rospy.is_shutdown():
        global lastData
        if lastData != None:
            pose.position.z += lastData.axes[1] / r / 2
            pose.position.x += lastData.axes[4] / r * 2
            pose.position.y += lastData.axes[3] / r * 2
            yaw += lastData.axes[0] / r * 2
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            # print(pose)
        pub.publish(pose)
        rate.sleep()
