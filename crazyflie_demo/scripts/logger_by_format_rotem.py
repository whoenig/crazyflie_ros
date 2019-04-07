#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import GenericLogData

from std_msgs.msg import Float64MultiArray
from crazyflie_demo.msg import crazyflie_sensors
from crazyflie_demo.msg import crazyflie_sensors_rotem
from geometry_msgs.msg import PointStamped

from sensor_msgs.msg import Imu
import geometry_msgs.msg
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion
from math import radians, pi

global x, y, z, roll, pitch, yaw
global front, back, up, left, right, zrange
global deltaX, deltaY, outlierCount
global accX, accY, accZ, gyroX, gyroY, gyroZ

x = y = z = roll = pitch = yaw = 0.0
front = back = up = left = right = zrange = 0.0
front_x, front_y, front_z = 0, 0, 0
back_x, back_y, back_z = 0, 0, 0
right_x, right_y, right_z = 0, 0, 0
left_x, left_y, left_z = 0, 0, 0


def get_pose(msg):
    global x, y, z
    x = msg.values[0]
    y = msg.values[1]
    z = msg.values[2]


def get_rpy(msg):
    global roll, pitch, yaw
    roll = radians(msg.values[0])
    pitch = radians(msg.values[1])
    yaw = radians(msg.values[2])


def get_ranges(msg):
    global front, back, up, left, right, zrange

    front = msg.values[0] / 1000
    back = msg.values[1] / 1000
    up = msg.values[2] / 1000
    left = msg.values[3] / 1000
    right = msg.values[4] / 1000
    zrange = msg.values[5] / 1000


def get_point_front(msg):
    global front_x, front_y, front_z
    front_x = msg.point.x
    front_y = msg.point.y
    front_z = msg.point.z


def get_point_back(msg):
    global back_x, back_y, back_z
    back_x = msg.point.x
    back_y = msg.point.y
    back_z = msg.point.z


def get_point_right(msg):
    global right_x, right_y, right_z
    right_x = msg.point.x
    right_y = msg.point.y
    right_z = msg.point.z


def get_point_left(msg):
    global left_x, left_y, left_z
    left_x = msg.point.x
    left_y = msg.point.y
    left_z = msg.point.z


def handle_pose():
    global x, y, z, roll, pitch, yaw
    global back_x, back_y, back_z
    global left_x, left_y, left_z
    global right_x, right_y, right_z
    global front_x, front_y, front_z

    rospy.init_node('tf_broadcaster')
    pub = rospy.Publisher('array_recording', crazyflie_sensors_rotem, queue_size=1)
    rospy.Subscriber('/cf1/log_pos', GenericLogData, get_pose)
    rospy.Subscriber('/cf1/log_rpy', GenericLogData, get_rpy)
    rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)
    rospy.Subscriber('/cf1/points/front_WC', PointStamped, get_point_front)
    rospy.Subscriber('/cf1/points/back_WC', PointStamped, get_point_back)
    rospy.Subscriber('/cf1/points/right_WC', PointStamped, get_point_right)
    rospy.Subscriber('/cf1/points/left_WC', PointStamped, get_point_left)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        t = crazyflie_sensors_rotem()

        t.x = x
        t.y = y
        t.z = z
        t.roll = roll
        t.pitch = pitch
        t.yaw = yaw
        t.front = front
        t.back = back
        t.up = up
        t.left = left
        t.right = right
        t.zrange = zrange

        try:  # if optitrack message exists
            trans = tfBuffer.lookup_transform('world', 'cf1', rospy.Time(0))

            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)

            euler = euler_from_quaternion(q, axes='sxzy')

            # translation : x, z, y
            # rotation : x, -z , y
            t.ref_x = -1 * trans.transform.translation.z
            t.ref_y = trans.transform.translation.x
            t.ref_z = trans.transform.translation.y
            t.ref_roll = euler[0]
            t.ref_pitch = -1 * euler[2]
            t.ref_yaw = euler[1]

        except:
            rospy.loginfo("tf lookup -- cf1 not found")

        t.front_x = front_x
        t.front_y = front_y
        t.front_z = front_z
        t.back_x = back_x
        t.back_y = back_y
        t.back_z = back_z
        t.left_x = left_x
        t.left_y = left_y
        t.left_z = left_z
        t.right_x = right_x
        t.right_y = right_y
        t.right_z = right_z

        pub.publish(t)

        # rospy.loginfo(t)
        r.sleep()


if __name__ == '__main__':
    handle_pose()
