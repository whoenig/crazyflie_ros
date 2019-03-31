#!/usr/bin/env python

from __future__ import absolute_import, division, unicode_literals, print_function

import rospy
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData

from threading import Thread

import tty, termios
import sys

speed = 0.25
initialZ = 0.3

global front, back, up, left, right, zrange
front = back = up = left = right = zrange = 0.0

global key


class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return speed
        elif distance < 0:
            return -1 * speed
        else:
            return 0

    # command CF to go at
    def goSpeed(self, vx, vy, zDistance, yaw):
        self.msg.vx = vx
        self.msg.vy = vy
        self.msg.yawrate = yaw
        self.msg.zDistance = zDistance
        self.msg.header.seq += 1
        self.msg.header.stamp = rospy.Time.now()
        # rospy.loginfo("sending...")
        # rospy.loginfo(self.msg.vx)
        # rospy.loginfo(self.msg.vy)
        # rospy.loginfo(self.msg.yawrate)
        # rospy.loginfo(self.msg.zDistance)
        self.pub.publish(self.msg)

    def goTo(self, x, y, zDistance, yaw):
        duration = 0
        duration_x = 0
        duration_y = 0
        duration_z = 0
        vx = 0
        vy = 0
        z = self.msg.zDistance  # the zDistance we have before
        z_scale = self.getSpeed(z)  # the z distance each time z has to increment, will be changed

        # for x, in secs
        if x != 0:
            duration_x = abs(x / speed)
            vx = self.getSpeed(x)

        # for y, in secs
        if y != 0:
            duration_y = abs(y / speed)
            vy = self.getSpeed(y)

        duration_z = abs(z - zDistance) / speed  # speed
        durations = [duration_x, duration_y, duration_z]
        duration = max(durations)

        if duration == 0:
            return
        elif duration == duration_x:
            # x is the longest path
            vy *= abs(y / x)
            z_scale *= abs((z - zDistance) / x)
        elif duration == duration_y:
            # y is the longest path
            vx *= abs(x / y)
            z_scale *= abs((z - zDistance) / y)
        elif duration == duration_z:
            # z is the longest path
            vx *= abs(x / (z - zDistance))
            vy *= abs(y / (z - zDistance))

        print(vx)
        print(vy)
        print(z_scale)
        print(duration)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.msg.vx = vx
            self.msg.vy = vy
            self.msg.yawrate = 0.0
            self.msg.zDistance = z
            if z < zDistance:
                print(zDistance)
                print(z)
                z += z_scale
            else:
                z = zDistance
            now = rospy.get_time()
            if (now - start > duration):
                break
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(self.msg.vx)
            rospy.loginfo(self.msg.vy)
            rospy.loginfo(self.msg.yawrate)
            rospy.loginfo(self.msg.zDistance)
            self.pub.publish(self.msg)
            self.rate.sleep()

    # take off to z distance
    def takeOff(self, zDistance):
        time_range = 1 + int(10 * zDistance / 0.4)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = y / 25.0
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            for y in range(20):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break

    # land from last zDistance
    def land(self):
        # get last height
        zDistance = self.msg.zDistance

        while not rospy.is_shutdown():
            while zDistance > 0:
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
                zDistance -= 0.03
        self.stop_pub.publish(self.stop_msg)
        exit(0)


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
    r = rospy.Rate(5)
    cf.takeOff(initialZ)

    x, y, yaw = 0, 0, 0
    z = initialZ

    global key
    key = None

    global front, back, up, left, right, zrange
    dist_threshold = 0.2

    try:
        print("keyboard controller.")
        print("press SPACE for emergency stop + land.")
        print("press 's' for stop.")

        print("press 'w' for forward.")
        print("press 'x' for backward.")
        print("press 'a' for left.")
        print("press 'd' for right.")
        print("press 'i' for up.")
        print("press 'k' for down.")
        print("press 'q' for yaw.")

        while not rospy.is_shutdown():

            if front < dist_threshold:
                print("forward collision avoidance")
                cf.goTo(-0.2, 0, z, 0)
                x, y, yaw = 0, 0, 0  # stop movement due to keyboard

            elif back < dist_threshold:
                print("back collision avoidance")
                cf.goTo(0.2, 0, z, 0)
                x, y, yaw = 0, 0, 0  # stop movement due to keyboard

            elif right < dist_threshold:
                print("right collision avoidance")
                cf.goTo(0, 0.2, z, 0)
                x, y, yaw = 0, 0, 0  # stop movement due to keyboard

            elif left < dist_threshold:
                print("left collision avoidance")
                cf.goTo(0, -0.2, z, 0)
                x, y, yaw = 0, 0, 0  # stop movement due to keyboard

            elif up < dist_threshold:
                print("top collision avoidance")
                cf.land()

            elif key is not None:

                print("************* Key pressed is " + key.decode('utf-8'))

                if key == ' ':
                    # emergency land
                    cf.land()
                elif key == 'w':
                    # move forward
                    x = speed
                elif key == 'x':
                    # move backward
                    x = -1 * speed
                elif key == 'd':
                    # move right
                    y = -1 * speed
                elif key == 'a':
                    # move left
                    y = speed
                elif key == 'i':
                    # move up
                    z += 0.05
                elif key == 'k':
                    # move down
                    z -= 0.05
                elif key == 'q':
                    # yaw+
                    yaw = 25
                elif key == 's':
                    # stop
                    x = y = yaw = 0
                key = None
                t2 = Thread(target=keypress, )
                t2.start()

            # print(" gospeed x: {}, y: {}, z: {} , yaw: {} \n".format( x, y, z ,yaw))
            cf.goSpeed(x, y, z, yaw)

            r.sleep()

    except Exception as e:
        print(e)

    # cf.land()


def get_ranges(msg):
    global front, back, up, left, right, zrange

    front = msg.values[0] / 1000
    back = msg.values[1] / 1000
    up = msg.values[2] / 1000
    left = msg.values[3] / 1000
    right = msg.values[4] / 1000
    zrange = msg.values[5] / 1000


if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    # settings = termios.tcgetattr(sys.stdin)

    rospy.Subscriber('/cf1/log_ranges', GenericLogData, get_ranges)

    cf1 = Crazyflie("cf1")
    t1 = Thread(target=handler, args=(cf1,))
    t2 = Thread(target=keypress, )
    t1.start()
    t2.start()
