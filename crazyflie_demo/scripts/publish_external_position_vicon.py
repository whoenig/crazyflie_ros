#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped

def onNewTransform(transform):
    global msg
    global pub
    msg.header.frame_id = transform.header.frame_id
    msg.header.stamp = transform.header.stamp
    msg.header.seq += 1
    msg.point.x = transform.transform.translation.x
    msg.point.y = transform.transform.translation.y
    msg.point.z = transform.transform.translation.z
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vicon', anonymous=True)
    topic = rospy.get_param("~topic", "/vicon/cf/cf")

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.Subscriber(topic, TransformStamped, onNewTransform)

    rospy.spin()
