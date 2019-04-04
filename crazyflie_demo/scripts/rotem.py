#!/usr/bin/env python
"""This scrips capture's all "ross" data of a specific CF, and contain it in a pythonic object, for later use. """

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
    rospy.loginfo("point_list: {}".format(point_list))
    print(point_list)

    # another method to access the points -

    # point_generator = pc2.read_points(msg)
    # for point in point_generator:
    #   if not math.isnan(point):
    #       print(point)

    try:  # if drone location is available (only after takeoff)

        trans = tfBuffer.lookup_transform('world', rospy.get_param("~tf_prefix"),
                                          rospy.Time(0))  # todo: make it from rosparams, and save it in a CF_object
        rospy.loginfo("\n\nDEBUG tf lookup success - {}".format(rospy.get_param("~tf_prefix")))

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

        rospy.loginfo("\n\nDEBUG current location x=", x, "y=", y)

    except:
        rospy.loginfo("\n\nDEBUG tf lookup failed - cf4\n\n")
        x = y = z = roll = pitch = yaw = 0


class CrazyflieFlightData:
    """A class for containing a crazyflie flight data.

    than later on, it will be used for rotem's alg'.
    """

    def __init__(self, tf_prefix):
        self.point_cloud_list
        self.tf_prefix = tf_prefix  # crazyflie name

    def update_point_cloud_data(self, point_list):
        # overwrite previeus point_list with
        self.point_cloud_list = point_list

        # Or, append to preveius point_cloud updtate
        # self.point_cloud_list.append(point_list)


if __name__ == '__main__':
    rospy.init_node("drone_data_parser")

    # get cf name
    prefix = rospy.get_param("~tf_prefix")
    pc_pub = rospy.Subscriber("/" + prefix + "/point_cloud", PointCloud2,
                              parser)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()
