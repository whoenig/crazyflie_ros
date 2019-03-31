#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import GenericLogData

from std_msgs.msg import Float64MultiArray
from crazyflie_demo.msg import crazyflie_sensors
from sensor_msgs.msg import Imu
import geometry_msgs.msg
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion
from math import radians, pi

global x, y, z, roll, pitch, yaw
global front,back , up , left , right , zrange
global deltaX , deltaY , outlierCount
global accX, accY, accZ, gyroX, gyroY, gyroZ

x=y=z=roll=pitch=yaw=0.0
front=back=up=left=right=zrange= 0.0
deltaX=deltaY=outlierCount=0.0
accX, accY, accZ, gyroX, gyroY, gyroZ = 0,0,0,0,0,0

def get_pose(msg):
    global x,y,z
    x = msg.values[0]
    y = msg.values[1]
    z = msg.values[2]

def get_rpy(msg):
    global roll,pitch,yaw
    roll=radians(msg.values[0])
    pitch = radians(msg.values[1])
    yaw = radians(msg.values[2])


def get_ranges(msg):
    global front, back, up, left, right, zrange

    front=msg.values[0]/1000
    back = msg.values[1]/1000
    up = msg.values[2]/1000
    left = msg.values[3]/1000
    right = msg.values[4]/1000
    zrange = msg.values[5]/1000

def get_sensors(msg):
    global deltaX , deltaY , outlierCount
    deltaX=msg.values[0]
    deltaY = msg.values[1]
    outlierCount = msg.values[2]

def get_imu(msg):
    global accX, accY, accZ, gyroX, gyroY, gyroZ
    accX = msg.linear_acceleration.x
    accY = msg.linear_acceleration.y
    accZ = msg.linear_acceleration.z
    gyroX = msg.angular_velocity.x
    gyroY = msg.angular_velocity.y
    gyroZ = msg.angular_velocity.z


def handle_pose():
    global x,y,z,roll,pitch,yaw



    rospy.init_node('tf_broadcaster')
    pub = rospy.Publisher('array_recording', crazyflie_sensors, queue_size=1)
    rospy.Subscriber('/cf1/log_pos' , GenericLogData, get_pose)
    rospy.Subscriber('/cf1/log_rpy' , GenericLogData, get_rpy)
    rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)
    rospy.Subscriber('/cf1/log_sensors', GenericLogData, get_sensors)
    rospy.Subscriber('/cf1/imu', Imu , get_imu)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r=rospy.Rate(50)
    while not rospy.is_shutdown():

        t = crazyflie_sensors()

        t.x=x
        t.y=y
        t.z=z
        t.roll=roll
        t.pitch=pitch
        t.yaw=yaw
        t.front=front
        t.back=back
        t.up=up
        t.left=left
        t.right=right
        t.zrange=zrange
        t.deltaX=deltaX
        t.deltaY = deltaY
        t.outlierCount=outlierCount
        t.accX=accX
        t.accY=accY
        t.accZ=accZ
        t.gyroX=gyroX
        t.gyroY = gyroY
        t.gyroZ = gyroZ

        try:                                                                            #if optitrack message exists
            trans = tfBuffer.lookup_transform('world', 'cf1', rospy.Time(0))

            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)

            euler = euler_from_quaternion(q, axes='sxzy')

            # translation : x, z, y
            # rotation : x, -z , y
            t.ref_x = -1*trans.transform.translation.z
            t.ref_y = trans.transform.translation.x
            t.ref_z = trans.transform.translation.y
            t.ref_roll = euler[0]
            t.ref_pitch = -1 * euler[2]
            t.ref_yaw = euler[1]

        except:
            rospy.loginfo("tf lookup -- cf1 not found")

        pub.publish(t)

        # rospy.loginfo(t)
        r.sleep()


if __name__ == '__main__':

    handle_pose()




