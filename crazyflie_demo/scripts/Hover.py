#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import Stop
from crazyflie_driver.srv import UpdateParams

# determine direction of speed based on distance
def getSpeed(distance):
    if distance > 0:
        return 0.1
    elif distance < 0:
        return -0.1
    else:
        return 0

# x, y is the x, y distance relative to itself
# z is absolute z distance
# TODO: solve 0
def goTo (x, y, zDistance, yaw, msg, rospy, pub):
    duration = 0
    duration_x = 0
    duration_y = 0
    duration_z = 0
    vx = 0
    vy = 0
    z = msg.zDistance # the zDistance we have before
    z_scale = getSpeed(z) # the z distance each time z has to increment, will be changed

    # for x, in secs
    if x != 0:
        duration_x = abs(x/0.1)
        vx = getSpeed(x)

    # for y, in secs
    if y != 0:
        duration_y = abs(y/0.1)
        vy = getSpeed(y)

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
        msg.vx = vx
        msg.vy = vy
        msg.yawrate = 0.0
        msg.zDistance = z
        if z < zDistance:
            print(zDistance)
            print(z)
            z += z_scale
        else:
            z = zDistance
        now = rospy.get_time()
        if (now - start > duration):
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

# take off to z distance
def takeOff(zDistance, rospy, msg, pub):
    time_range = 1 + int(10*zDistance/0.4)
    while not rospy.is_shutdown():
        for y in range(time_range):
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = y / 25.0
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
        for y in range(20):
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = zDistance
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
        break

# land from last zDistance
def land (msg, pub, stop_msg, stop_pub, rospy):
    # get last height
    zDistance = msg.zDistance

    while not rospy.is_shutdown():
        while zDistance > 0:
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yawrate = 0.0
            msg.zDistance = zDistance
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
            zDistance -= 0.2
    stop_pub.publish(stop_msg)

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    rate = rospy.Rate(10) # 10 hz
    name = "cmd_hover"

    msg = Hover()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.yawrate = 0

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

    # take off to 0.4
    takeOff(0.4, rospy, msg, pub)

    goTo(0.4, 0.1, 0.2, 0, msg, rospy, pub)

    land(msg, pub, stop_msg, stop_pub, rospy)

