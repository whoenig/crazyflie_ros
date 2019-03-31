#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

from __future__ import absolute_import, division, unicode_literals, print_function

import rospy
import crazyflie
import time
import tf
#from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import *
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client

from threading import Thread

import tty, termios
import sys

speed=0.25
initialZ=0.5

global front,back , up , left , right , zrange
front=back=up=left=right=zrange= 0.0


def get_ranges(msg):
    global front, back, up, left, right, zrange

    front=msg.values[0]/1000
    back = msg.values[1]/1000
    up = msg.values[2]/1000
    left = msg.values[3]/1000
    right = msg.values[4]/1000
    zrange = msg.values[5]/1000

    

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def keypress():
    global key
    key = getch()

def handler(cf):

    r=rospy.Rate(5)
    time.sleep(1)
    cf.takeoff(targetHeight = initialZ, duration = 5.0)
    time.sleep(5.0)

    x, y, yaw = 0, 0, 0
    z=initialZ

    global key
    key = None
    global front, back, up, left, right, zrange
    dist_threshold=0.2
    def_duration=0.5
    step_size=0.25

    try:
        rospy.loginfo("keyboard controller.")
        rospy.loginfo("press SPACE for emergency stop + land.")
        rospy.loginfo("press 's' for stop.")
        rospy.loginfo("press 'w' for forward.")
        rospy.loginfo("press 'x' for backward.")
        rospy.loginfo("press 'a' for left.")
        rospy.loginfo("press 'd' for right.")
        rospy.loginfo("press 'i' for up.")
        rospy.loginfo("press 'k' for down.")
        rospy.loginfo("press 'q','e' for yaw +-45 deg.")


        while not rospy.is_shutdown():
            if front > 0 :
                if front < dist_threshold:
                    rospy.loginfo("forward collision avoidance")
                    cf.goTo(goal=[-0.2, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                    time.sleep(def_duration)

                elif back < dist_threshold:
                    rospy.loginfo("back collision avoidance")
                    cf.goTo(goal=[0.2, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                    time.sleep(def_duration)

                elif right < dist_threshold:
                    rospy.loginfo("right collision avoidance")
                    cf.goTo(goal=[0.0, 0.2, 0.0], yaw=0, duration=def_duration, relative=True)
                    time.sleep(def_duration)

                elif left < dist_threshold:
                    rospy.loginfo("left collision avoidance")
                    cf.goTo(goal=[0.0, -0.2, 0.0], yaw=0, duration=def_duration, relative=True)
                    time.sleep(def_duration)

                elif up < dist_threshold:
                    rospy.loginfo("top collision avoidance")
                    land_duration = z * 3
                    cf.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    cf.stop()
                    break

            if key is not None:

                rospy.loginfo("************* Key pressed is " + key.decode('utf-8'))

                if key == ' ':
                    # emergency land
                    land_duration=z*3
                    cf.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration-0.5)
                    cf.stop()
                    break
                elif key == 'w':
                    # move forward
                    cf.goTo(goal=[step_size, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'x':
                    # move backward
                    cf.goTo(goal=[-1*step_size, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'd':
                    # move right
                    cf.goTo(goal=[0.0, -1*step_size, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'a':
                    # move left
                    cf.goTo(goal=[0.0, step_size, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'i':
                    # move up
                    cf.goTo(goal=[0.0, 0.0, 0.1], yaw=0, duration=def_duration, relative=True)
                elif key == 'k':
                    # move down
                    cf.goTo(goal=[0.0, 0.0, -0.1], yaw=0, duration=def_duration, relative=True)
                elif key == 'q':
                    # 45 degrees CW
                    cf.goTo(goal=[0.0, 0.0, 0.0], yaw=0.785, duration=def_duration, relative=True)
                elif key == 'e':
                    # 45 degrees CCW
                    cf.goTo(goal=[0.0, 0.0, 0.0], yaw=-0.785, duration=def_duration, relative=True)
                #elif key == 's':
                    # stop

                key = None
                t2 = Thread(target=keypress, )
                t2.start()

            #print(" gospeed x: {}, y: {}, z: {} , yaw: {} \n".format( x, y, z ,yaw))
            #cf.goSpeed(x, y, z, yaw)

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf.stop()
        #break

    except Exception as e:
        cf.stop()
        rospy.loginfo('*******keyboard input exception')
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('test_high_level')
    rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)
    prefix = '/cf1'
    cf = crazyflie.Crazyflie("/cf1", "world")
    rospy.wait_for_service(prefix + '/update_params')
    rospy.loginfo("found update_params service")

    #cf.setParam("commander/enHighLevel", 1)
    #cf.setParam("stabilizer/estimator", 2) # Use EKF

    #cf.setParam("ctrlMel/kp_z", 0.6) #reduce z wobble - default 1.25
    #cf.setParam("ctrlMel/ki_z", 0.03) #reduce z wobble - default 0.05
    #cf.setParam("ctrlMel/kd_z", 0.15) #reduce z wobble - default 0.4
    #cf.setParam("ctrlMel/i_range_z", 0.2) #reduce z wobble


    ## reset kalman
    # cf.setParam("kalman/initialX", 0)
    # cf.setParam("kalman/initialY", 0)
    # cf.setParam("kalman/initialZ", 0)
    # cf.setParam("kalman/resetEstimation", 1)
    ########

    #cf.setParam("stabilizer/controller", 2) # 2=Use mellinger controller
    time.sleep(1.0)

    rospy.loginfo("launching threads")
    t1 = Thread(target=handler, args=(cf,))
    t2 = Thread(target=keypress, )
    t1.start()
    t2.start()

