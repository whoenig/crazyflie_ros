#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped
from crazyflie_driver.msg import GenericLogData
import tf_conversions
import tf2_ros
import tf2_geometry_msgs

from time import sleep


def get_ranges(msg):
    global pub_front, pub_back, pub_left, pub_right, seq
    global prev_front, prev_back, prev_left, prev_right
    global pub_front_WC
    global tfBuffer, listener

    front = msg.values[0] / 1000
    back = -1 * msg.values[1] / 1000
    left = msg.values[3] / 1000
    right = -1 * msg.values[4] / 1000

    transform = None
    try:
        transform = tfBuffer.lookup_transform('world', rospy.get_param("~tf_prefix"), rospy.Time(0))
    except Exception as e:
        rospy.loginfo(e)

    if (front != prev_front):

        point_front = PointStamped()
        point_front.header.seq = seq
        point_front.header.stamp = rospy.Time.now()
        point_front.header.frame_id = rospy.get_param("~tf_prefix")
        point_front.point.x = front
        point_front.point.y = 0
        point_front.point.z = 0
        pub_front.publish(point_front)
        prev_front = front

        # publish another point in world coordinates
        if (transform != None):
            front_WC = tf2_geometry_msgs.do_transform_point(point_front, transform)
            front_WC.header.frame_id = "world"
            pub_front_WC.publish(front_WC)

    if (back != prev_back):

        point_back = PointStamped()
        point_back.header.seq = seq
        point_back.header.stamp = rospy.Time.now()
        point_back.header.frame_id = rospy.get_param("~tf_prefix")
        point_back.point.x = back
        point_back.point.y = 0
        point_back.point.z = 0
        pub_back.publish(point_back)
        prev_back = back

        # publish another point in world coordinates
        if (transform != None):
            back_WC = tf2_geometry_msgs.do_transform_point(point_back, transform)
            back_WC.header.frame_id = "world"
            pub_back_WC.publish(back_WC)

    if (left != prev_left):
        point_left = PointStamped()
        point_left.header.seq = seq
        point_left.header.stamp = rospy.Time.now()
        point_left.header.frame_id = rospy.get_param("~tf_prefix")
        point_left.point.x = 0
        point_left.point.y = left
        point_left.point.z = 0
        pub_left.publish(point_left)
        prev_left = left

        # publish another point in world coordinates
        if (transform != None):
            left_WC = tf2_geometry_msgs.do_transform_point(point_left, transform)
            left_WC.header.frame_id = "world"
            pub_left_WC.publish(left_WC)

    if (right != prev_right):

        point_right = PointStamped()
        point_right.header.seq = seq
        point_right.header.stamp = rospy.Time.now()
        point_right.header.frame_id = rospy.get_param("~tf_prefix")
        point_right.point.x = 0
        point_right.point.y = right
        point_right.point.z = 0
        pub_right.publish(point_right)
        prev_right = right

        # publish another point in world coordinates
        if (transform != None):
            right_WC = tf2_geometry_msgs.do_transform_point(point_right, transform)
            right_WC.header.frame_id = "world"
            pub_right_WC.publish(right_WC)

    seq += 1


if __name__ == '__main__':
    rospy.init_node('publish_point', anonymous=False)
    pub_front = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/front', PointStamped, queue_size=1)
    pub_back = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/back', PointStamped, queue_size=1)
    pub_left = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/left', PointStamped, queue_size=1)
    pub_right = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/right', PointStamped, queue_size=1)

    pub_front_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/front_WC', PointStamped, queue_size=1)
    pub_back_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/back_WC', PointStamped, queue_size=1)
    pub_left_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/left_WC', PointStamped, queue_size=1)
    pub_right_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/right_WC', PointStamped, queue_size=1)

    rospy.Subscriber('/' + rospy.get_param("~tf_prefix") + '/log_ranges', GenericLogData, get_ranges)
    prev_front = prev_back = prev_left = prev_right = 0
    seq = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()
