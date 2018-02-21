#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    # name = rospy.get_param("~name") what is name?
    # r = rospy.get_param("~rate")
    # x = rospy.get_param("~x")
    # y = rospy.get_param("~y")
    # z = rospy.get_param("~z")

    rate = rospy.Rate(10) # 10 hz
    name = "cmd_hover"

    msg = Hover()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.vx = 0.0
    msg.vy = 0.0
    msg.yawrate = 0.0
    msg.zDistance = 0.0

    pub = rospy.Publisher(name, Hover, queue_size=1)

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.5)

    # take off to 0.5 m zDistance, spend 3 secs
    start = rospy.get_time()
    # rospy.loginfo("start: ")
    # rospy.loginfo(start)

    while not rospy.is_shutdown():
        for y in range(10):
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = y / 25.0
            now = rospy.get_time()
            # if (now - start > 3.0):
            #    break
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(msg.vx)
            rospy.loginfo(msg.vy)
            rospy.loginfo(msg.yawrate)
            rospy.loginfo(msg.zDistance)
            # rospy.loginfo(now)
            pub.publish(msg)
            rate.sleep()
        for y in range(20):
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = 0.4
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(msg.vx)
            rospy.loginfo(msg.vy)
            rospy.loginfo(msg.yawrate)
            rospy.loginfo(msg.zDistance)
            # rospy.loginfo(now)
            pub.publish(msg)
            rate.sleep()
        break

    # vx = 0.1, spend 3 secs, go forward 0.3m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.1
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)
        rospy.loginfo(msg.yawrate)
        rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vy = 0.1, spend 3 secs, go right 0.3m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = 0.1
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)
        rospy.loginfo(msg.yawrate)
        rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vx = -0.1, spend 3 secs, go backward 0.3m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = -0.1
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)
        rospy.loginfo(msg.yawrate)
        rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vy = -0.1, spend 3 secs, go left 0.3m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = -0.1
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)
        rospy.loginfo(msg.yawrate)
        rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # land, spend 3 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.0
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)
        rospy.loginfo(msg.yawrate)
        rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()
