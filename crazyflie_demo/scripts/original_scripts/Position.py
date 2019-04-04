#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    rate = rospy.Rate(10) # 10 hz
    name = "cmd_position"

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = 0.0
    msg.y = 0.0
    msg.z = 0.0
    msg.yaw = 0.0

    pub = rospy.Publisher(name, Position, queue_size=1)

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

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
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.z = y / 25.0
            now = rospy.get_time()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(msg.x)
            rospy.loginfo(msg.y)
            rospy.loginfo(msg.z)
            rospy.loginfo(msg.yaw)
            # rospy.loginfo(now)
            pub.publish(msg)
            rate.sleep()
        for y in range(20):
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.z = 0.4
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(msg.x)
            rospy.loginfo(msg.y)
            rospy.loginfo(msg.z)
            rospy.loginfo(msg.yaw)
            # rospy.loginfo(now)
            pub.publish(msg)
            rate.sleep()
        break

    # go to x: 0.2 y: 0.2
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.x = 0.2
        msg.y = 0.2
        msg.yaw = 0.0
        msg.z = 0.4
        now = rospy.get_time()
        if (now - start > 3.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.x)
        rospy.loginfo(msg.y)
        rospy.loginfo(msg.z)
        rospy.loginfo(msg.yaw)
        pub.publish(msg)
        rate.sleep()

    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.yaw = 0.0
        now = rospy.get_time()
        if (now - start > 1.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.x)
        rospy.loginfo(msg.y)
        rospy.loginfo(msg.z)
        rospy.loginfo(msg.yaw)
        pub.publish(msg)
        rate.sleep()

    stop_pub.publish(stop_msg)
