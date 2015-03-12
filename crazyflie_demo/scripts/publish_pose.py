#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, Pose, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()
