#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import Stop
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

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

    stop_pub = rospy.Publisher("cmd_stop", Stop, queue_size=1)
    stop_msg = Stop()
    stop_msg.header.seq = 0
    stop_msg.header.stamp = rospy.Time.now()
    stop_msg.header.frame_id = worldFrame

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.5)

    # take off
    while not rospy.is_shutdown():
        for y in range(10):
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = y / 25.0
            now = rospy.get_time()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            #rospy.loginfo("sending...")
            #rospy.loginfo(msg.vx)
            #rospy.loginfo(msg.vy)
            #rospy.loginfo(msg.yawrate)
            #rospy.loginfo(msg.zDistance)
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
            #rospy.loginfo("sending...")
            #rospy.loginfo(msg.vx)
            #rospy.loginfo(msg.vy)
            #rospy.loginfo(msg.yawrate)
            #rospy.loginfo(msg.zDistance)
            # rospy.loginfo(now)
            pub.publish(msg)
            rate.sleep()
        break

    # vx = 0.2, spend 3 secs, go forward 0.6m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.2
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        #rospy.loginfo(msg.vx)
        #rospy.loginfo(msg.vy)
        #rospy.loginfo(msg.yawrate)
        #rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vy = 0.2, spend 3 secs, go right 0.6m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = 0.2
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        #rospy.loginfo(msg.vx)
        #rospy.loginfo(msg.vy)
        #rospy.loginfo(msg.yawrate)
        #rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vx = -0.2, spend 3 secs, go backward 0.6m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = -0.2
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        ##rospy.loginfo(msg.vx)
        #rospy.loginfo(msg.vy)
        #rospy.loginfo(msg.yawrate)
        #rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # vy = -0.2, spend 3 secs, go left 0.6m
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = -0.2
        msg.yawrate = 0.0
        msg.zDistance = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        #rospy.loginfo(msg.vx)
        #rospy.loginfo(msg.vy)
        #rospy.loginfo(msg.yawrate)
        #rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = 0.0
        msg.vy = 0.0
        msg.yawrate = 0.0
        msg.zDistance = 0.0
        now = rospy.get_time()
        if (now - start > 1.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        #rospy.loginfo(msg.vx)
        #rospy.loginfo(msg.vy)
        #rospy.loginfo(msg.yawrate)
        #rospy.loginfo(msg.zDistance)
        pub.publish(msg)
        rate.sleep()

    stop_pub.publish(stop_msg)
