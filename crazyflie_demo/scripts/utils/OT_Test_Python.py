#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def Print_data(msg):
    print
    msg.pose.position
    print
    ""


def laser_listener():
    rospy.Subscriber("/vrpn_client_node/Crazyflie_3/pose", PoseStamped, Print_data)


def main():
    rospy.init_node('laser_listener', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        laser_listener()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
