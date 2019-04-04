#!/usr/bin/env python


import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf.transformations import euler_from_quaternion
import math


def parser(msg):
    global point_list
    global x, y, z, roll, pitch, yaw
    # convert incoming laserscan message to individual points
    # we can access the point list with an index, each element is a namedtuple

    point_list = pc2.read_points_list(msg, skip_nans=True)
    print(point_list)

    # another method to access the points -

    # point_generator = pc2.read_points(msg)
    # for point in point_generator:
    #   if not math.isnan(point):
    #       print(point)

    try:  # if drone location is available (only after takeoff)

        trans = tfBuffer.lookup_transform('world', 'cf1', rospy.Time(0))
        rospy.loginfo("DEBUG tf lookup success - cf1")

        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)

        euler = euler_from_quaternion(q, axes='sxyz')

        # translation : x, z, y
        # rotation : r , p , y
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        rospy.loginfo("DEBUG current location x=", x, "y=", y)

    except:
        rospy.loginfo("DEBUG tf lookup failed - cf1")
        x = y = z = roll = pitch = yaw = 0


if __name__ == '__main__':
    rospy.init_node("drone_data_parser")
    pc_pub = rospy.Subscriber("point_cloud", PointCloud2, parser)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()
