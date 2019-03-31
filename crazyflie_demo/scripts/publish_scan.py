#!/usr/bin/env python

# sucscribes to log_ranges and publishes PointStanmed message in LC
# if transformation between 'drone' and 'world' exists, publishes PointStamped message in WC
# publishes laserscan in 'drone' frame, and converts it to PointCloud2 too.


import rospy
import tf
from geometry_msgs.msg import PointStamped
from crazyflie_driver.msg import GenericLogData
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2

from time import sleep
from math import pi

import laser_geometry.laser_geometry as lg


def get_ranges(msg):
    global pub_front, pub_back, pub_left, pub_right, seq
    global prev_front, prev_back, prev_left, prev_right
    global tfBuffer, listener

    front = msg.values[0] / 1000
    back = msg.values[1] / 1000
    left = msg.values[3] / 1000
    right = msg.values[4] / 1000

    # inf=float('inf')
    # scan.ranges = [inf,inf,inf,inf] #TODO - fix error with inf values
    scan.ranges = [0, 0, 0, 0]

    transform = None
    try:
        transform = tfBuffer.lookup_transform('world', 'cf1', rospy.Time(0))  # todo - change frame id from hardcoded
    except Exception as e:
        rospy.loginfo(e)

    if (front != prev_front):

        scan.ranges[2] = front

        ##publish point in local coordinates
        point_front = PointStamped()
        point_front.header.seq = seq
        point_front.header.stamp = rospy.Time.now()
        point_front.header.frame_id = "cf1"  # todo - change frame id from hardcoded
        point_front.point.x = front
        point_front.point.y = 0
        point_front.point.z = 0
        pub_front.publish(point_front)
        prev_front = front

        ##publish another point in world coordinates
        if (transform != None):
            front_WC = tf2_geometry_msgs.do_transform_point(point_front, transform)
            front_WC.header.frame_id = "world"
            pub_front_WC.publish(front_WC)

    if (back != prev_back):

        scan.ranges[0] = back

        point_back = PointStamped()
        point_back.header.seq = seq
        point_back.header.stamp = rospy.Time.now()
        point_back.header.frame_id = "cf1"
        point_back.point.x = -1 * back
        point_back.point.y = 0
        point_back.point.z = 0
        pub_back.publish(point_back)
        prev_back = back

        ##publish another point in world coordinates
        if (transform != None):
            back_WC = tf2_geometry_msgs.do_transform_point(point_back, transform)
            back_WC.header.frame_id = "world"
            pub_back_WC.publish(back_WC)

    if (left != prev_left):

        scan.ranges[3] = left

        point_left = PointStamped()
        point_left.header.seq = seq
        point_left.header.stamp = rospy.Time.now()
        point_left.header.frame_id = "cf1"
        point_left.point.x = 0
        point_left.point.y = left
        point_left.point.z = 0
        pub_left.publish(point_left)
        prev_left = left

        ##publish another point in world coordinates
        if (transform != None):
            left_WC = tf2_geometry_msgs.do_transform_point(point_left, transform)
            left_WC.header.frame_id = "world"
            pub_left_WC.publish(left_WC)

    if (right != prev_right):

        scan.ranges[1] = right

        point_right = PointStamped()
        point_right.header.seq = seq
        point_right.header.stamp = rospy.Time.now()
        point_right.header.frame_id = "cf1"
        point_right.point.x = 0
        point_right.point.y = -1 * right
        point_right.point.z = 0
        pub_right.publish(point_right)
        prev_right = right

        ##publish another point in world coordinates
        if (transform != None):
            right_WC = tf2_geometry_msgs.do_transform_point(point_right, transform)
            right_WC.header.frame_id = "world"
            pub_right_WC.publish(right_WC)

    scan.header.stamp = rospy.Time.now()
    scan_pub.publish(scan)

    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(scan)

    # publish it
    pc2_pub.publish(pc2_msg)

    seq += 1


if __name__ == '__main__':
    rospy.init_node('publish_point', anonymous=False)
    pub_front = rospy.Publisher('/cf1/points/front', PointStamped, queue_size=1)
    pub_back = rospy.Publisher('/cf1/points/back', PointStamped, queue_size=1)
    pub_left = rospy.Publisher('/cf1/points/left', PointStamped, queue_size=1)
    pub_right = rospy.Publisher('/cf1/points/right', PointStamped, queue_size=1)

    pub_front_WC = rospy.Publisher('/cf1/points/front_WC', PointStamped, queue_size=1)
    pub_back_WC = rospy.Publisher('/cf1/points/back_WC', PointStamped, queue_size=1)
    pub_left_WC = rospy.Publisher('/cf1/points/left_WC', PointStamped, queue_size=1)
    pub_right_WC = rospy.Publisher('/cf1/points/right_WC', PointStamped, queue_size=1)

    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=2)
    pc2_pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=1)

    rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)
    prev_front = prev_back = prev_left = prev_right = 0
    seq = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    lp = lg.LaserProjection()

    scan = LaserScan()
    num_readings = 4
    laser_frequency = 10
    scan.header.frame_id = 'cf1'
    scan.angle_min = -pi
    scan.angle_max = pi
    scan.angle_increment = 2 * pi / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 3.5

    rospy.spin()
