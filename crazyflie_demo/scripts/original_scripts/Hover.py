#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread

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


    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return 0.1
        elif distance < 0:
            return -0.1
        else:
            return 0

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    # x, y is the x, y distance relative to itself
    # z is absolute z distance
    # TODO: solve 0
    def goTo (self, x, y, zDistance, yaw):
        duration = 0
        duration_x = 0
        duration_y = 0
        duration_z = 0
        vx = 0
        vy = 0
        z = self.msg.zDistance # the zDistance we have before
        z_scale = self.getSpeed(z) # the z distance each time z has to increment, will be changed

        # for x, in secs
        if x != 0:
            duration_x = abs(x/0.1)
            vx = self.getSpeed(x)

        # for y, in secs
        if y != 0:
            duration_y = abs(y/0.1)
            vy = self.getSpeed(y)

        duration_z = abs(z-zDistance)/0.1
        durations = [duration_x, duration_y, duration_z]
        duration = max(durations)

        if duration == 0:
            return
        elif duration == duration_x:
            # x is the longest path
            vy *= abs(y/x)
            z_scale *= abs((z-zDistance)/x)
        elif duration == duration_y:
            # y is the longest path
            vx *= abs(x/y)
            z_scale *= abs((z-zDistance)/y)
        elif duration == duration_z:
            # z is the longest path
            vx *= abs(x/(z-zDistance))
            vy *= abs(y/(z-zDistance))

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
        time_range = 1 + int(10*zDistance/0.4)
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
    def land (self):
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
                zDistance -= 0.2
        self.stop_pub.publish(self.stop_msg)

def handler(cf):
    cf.takeOff(0.4)
    cf.goTo(0.4, 0.1, 0.2, 0)
    cf.land()

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)

    cf1 = Crazyflie("cf1")
    cf2 = Crazyflie("cf2")

    t1 = Thread(target=handler, args=(cf1,))
    t2 = Thread(target=handler, args=(cf2,))
    t1.start()
    t2.start()



