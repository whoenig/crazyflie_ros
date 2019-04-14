#!/usr/bin/env python
"""This is Cj publisher.

It publish a 'Goal' relative to 'world', per CF.
"""

import rospy
from crazyflie_driver.msg import GenericLogData

from std_msgs.msg import Float64MultiArray
from crazyflie_demo.msg import crazyflie_sensors
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion
from math import radians, pi

global x, y, z, roll, pitch, yaw
global front, back, up, left, right, zrange
global deltaX, deltaY, outlierCount
global accX, accY, accZ, gyroX, gyroY, gyroZ
global quaternion, goal_x, goal_y, goal_z, goal_q

x = y = z = roll = pitch = yaw = 0.0
front = back = up = left = right = zrange = 0.0
deltaX = deltaY = outlierCount = 0.0
accX, accY, accZ, gyroX, gyroY, gyroZ = 0, 0, 0, 0, 0, 0
quaternion = goal_x = goal_y = goal_z = goal_q = 0
quaternion = 0


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


def get_goal(msg):
    global goal_x, goal_y, goal_z, goal_q

    global x, y, z
    goal_x = msg.pose.position[0]
    goal_y = msg.pose.position[1]
    goal_z = msg.pose.position[2]
    goal_q = msg.pose.orientation


#
# def get_sensors(msg):
#     global deltaX, deltaY, outlierCount
#     global roll, pitch, yaw
#     roll = radians(msg.values[0])
#     pitch = radians(msg.values[1])
#     yaw = radians(msg.values[2])
#     deltaX = msg.values[0]
#     deltaY = msg.values[1]
#     outlierCount = msg.values[2]
#
#
# def get_imu(msg):
#     global accX, accY, accZ, gyroX, gyroY, gyroZ
#     accX = msg.linear_acceleration.x
#     accY = msg.linear_acceleration.y
#     accZ = msg.linear_acceleration.z
#     gyroX = msg.angular_velocity.x
#     gyroY = msg.angular_velocity.y
#     gyroZ = msg.angular_velocity.z


def goal_logger_handler(tf_prefix):
    """Listen to pose, listen to goal_publisher.

    Publish goal relative to world.
    """

    global goal_x, goal_y, goal_z, goal_q
    pub = rospy.Publisher('/' + tf_prefix + '/world_goal_publisher', PoseStamped, queue_size=1)
    rospy.Subscriber('/' + tf_prefix + '/log_pos', GenericLogData, get_pose)
    rospy.Subscriber('/' + tf_prefix + '/log_rpy', GenericLogData, get_rpy)
    rospy.Subscriber('/' + tf_prefix + '/cf_goal_publisher', PoseStamped, get_goal)
    #
    # rospy.Subscriber('/' + tf_prefix + '/log_sensors', GenericLogData, get_sensors)
    # rospy.Subscriber('/' + tf_prefix + '/imu', Imu, get_imu)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        t = PoseStamped()

        t.pose.position.x = x + goal_x
        t.pose.position.y = y + goal_y
        t.pose.position.z = z + goal_z
        t.pose.orientation = [0,0,0,0]

        rospy.loginfo(t)
        pub.publish(t)


        r.sleep()


if __name__ == '__main__':
    rospy.init_node("world_goal_publisher")

    tf_prefix = rospy.get_param("~tf_prefix")

    goal_logger_handler(tf_prefix)
