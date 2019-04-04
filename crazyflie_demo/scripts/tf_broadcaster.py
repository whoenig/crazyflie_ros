#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import GenericLogData

import geometry_msgs.msg
import tf_conversions
import tf2_ros

from math import radians

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

global x, y, z, roll, pitch, yaw
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0


def get_pose(msg):
    global x, y, z
    x = msg.values[0]
    y = msg.values[1]
    z = msg.values[2]


def get_rpy(msg):
    global roll, pitch, yaw
    roll = msg.values[0]
    pitch = msg.values[1]
    yaw = msg.values[2]


def handle_pose():
    global x, y, z, roll, pitch, yaw
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/' + rospy.get_param("~tf_prefix") + '/log_pos', GenericLogData, get_pose)
    rospy.Subscriber('/' + rospy.get_param("~tf_prefix") + '/log_rpy', GenericLogData, get_rpy)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

    r = rospy.Rate(40)
    prev_x = prev_y = prev_yaw = 0
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = rospy.get_param("~tf_prefix")
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

        # compute odometry

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        vx = (x - prev_x) / dt
        vy = (y - prev_y) / dt
        vth = (radians(yaw) - radians(prev_yaw)) / dt

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"  # todo  - DRONE odom FRAME ID IS HARDCODED, CHANGE TO PARAMETER ROSPARAM
        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*q))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        prev_x = x
        prev_y = y
        prev_yaw = yaw
        last_time = current_time

        r.sleep()


if __name__ == '__main__':
    handle_pose()
