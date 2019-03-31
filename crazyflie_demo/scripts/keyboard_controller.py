#!/usr/bin/env python

from __future__ import print_function

import roslib
import threading

roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty


class CrazyCommands:
    """Class for holding drone commands. """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.kill = False


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def callback(msg):
    global VelZ
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))
    move_mm_sec = msg.linear.x * 1000  # conversion from m/s to mm/s
    turn_mm_sec = msg.angular.z * robot_base_circum / (2 * pi)
    leftwheel_clicks = mm_to_clicks(move_mm_sec + turn_mm_sec)
    rightwheel_clicks = mm_to_clicks(move_mm_sec - turn_mm_sec) * -1
    print("DEBUG left: ", leftwheel_clicks, " right: ", rightwheel_clicks)
    resp = move(leftwheel_clicks, rightwheel_clicks)
    VelZ = msg.linear.x

    # print "VelZ callback=", VelZ


def listener():
    rospy.Subscriber("/tf", Twist, callback)


def fly(crazy_commander,pub):
    """Crazy thread, will send crazyflie updated movement commands. """

    while crazy_commander.kill is False:
        # Make a twist message, and publish it.
        twist = Twist()
        twist.linear.x = crazy_commander.x
        twist.linear.y = crazy_commander.y
        twist.linear.z = crazy_commander.z
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0  # th # * turn
        pub.publish(twist)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Publish commands via cmd_vel topic.
    pub = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=2)

    rospy.init_node('keyboard_controller')
    r = rospy.Rate(1)  # 100hz

    crazy_command = CrazyCommands();
    crazy_command.x = 0
    crazy_command.y = 0
    crazy_command.z = 0

    x,y,z = 0,0,0

    # we're just going to wait for user input while adding up time once...
    print("Starting thread")
    threading.Thread(target=fly, args=[crazy_command, pub]).start()

    th = 0
    status = 0
    v_0 = 1

    try:
        print("This is keyboard controller.")
        print("press 'e' for emergency stop.")
        print("press 'q' for maximum power takeoff.")
        print("press 'w' for up.")
        print("press 's' for down.")
        print("press 'a' for left.")
        print("press 'd' for right.")
        print("press 'i' for forward.")
        print("press 'k' for backward.")
        print("press '1' for v_0.")
        print("press '1' for v_0*2.")
        print("press '1' for v_0*3.")
        print("press '1' for v_0*4.")
        print("v_0: {}, x: {}, y: {}, z: {}".format(v_0, x, y, z))
        while (1):
            key = getKey()

            if key == 'q' and z < 65000:
                # take off - full power
                z = 60000
            if key == '1':
                # set v_0 to normal
                v_0 = 1
            if key == '2':
                # set v_0 to x2
                v_0 = 2
            if key == '3':
                # set v_0 to x3
                v_0 = 3
            if key == '4':
                # set v_0 to x4
                v_0 = 4

            if key == 'e' and z > 0:
                # emergency shut-off
                z = 0
                y = 0
                x = 0

            if key == 'w' and z < 65000:
                # move up
                z = z + 1000*v_0
            elif key == 's' and z > 0:
                # move down
                z = z - 1000*v_0
            elif key == 'd' and y < 100:
                # move left
                y = y + 1*v_0
            elif key == 'a' and y > -100:
                # move right
                y = y - 1*v_0
            elif key == 'i' and x < 100:
                # move forward
                x = x + 1*v_0
            elif key == 'k' and x > -100:
                # move backward
                x = x - 1*v_0
            elif key == 'p':
                print("Good bye.")
                crazy_command.kill = True
                break

            crazy_command.x = x
            crazy_command.y = y
            crazy_command.z = z
            print("v_0: {}, x: {}, y: {}, z: {}".format(v_0, x, y, z))
            r.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)


        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
