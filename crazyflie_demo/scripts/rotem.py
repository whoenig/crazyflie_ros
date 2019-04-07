#!/usr/bin/env python
"""This scrips capture's all "ross" data of a specific CF, and contain it in a pythonize object, for later use. """

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
from crazyflie_driver.msg import GenericLogData

import tf2_ros
from tf.transformations import euler_from_quaternion
import math

"""
P*j - position vector (after filtering / conditioning . transformations) as ROS  structure - TransformStamped
S*j - distance sensor x,y,z point cloud 3*6 (after filtering / conditioning) as ROS  structure - PointCloud2 (global coordinates)

list of topics per drone:
    /cfX/imu
    /cfX/laserScan
    /cfX/log_pos                <----- Where listening to this
    /cfX/log_ranges
    /cfX/log_rpy
    /cfX/odom
    /cfX/point_cloud            <----- Where listening to this
    /cfX/points/back
    /cfX/points/back_WC
    /cfX/points/front
    /cfX/points/front_WC
    /cfX/points/right
    /cfX/points/right_WC


"""


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data


# convert incoming laserscan message to individual points
# we can access the point list with an index, each element is a namedtuple

# point_list = pc2.read_points_list(msg, skip_nans=True)
# rospy.loginfo("\n\npoint_list: {}".format(point_list))

# another method to access the points -

# point_generator = pc2.read_points(msg)
# for point in point_generator:
#   if not math.isnan(point):
#       print(point)

# try:  # if drone location is available (only after takeoff)
#
#     trans = tfBuffer.lookup_transform('world', rospy.get_param("~tf_prefix"),
#                                       rospy.Time(0))  # todo: make it from rosparams, and save it in a CF_object
#     rospy.loginfo("\n\nDEBUG tf lookup success - {}".format(rospy.get_param("~tf_prefix")))
#
#     q = (trans.transform.rotation.x,
#          trans.transform.rotation.y,
#          trans.transform.rotation.z,
#          trans.transform.rotation.w)
#
#     euler = euler_from_quaternion(q, axes='sxyz')
#
#     # translation : x, z, y
#     # rotation : r , p , y
#     x = trans.transform.translation.x
#     y = trans.transform.translation.y
#     z = trans.transform.translation.z
#     roll = euler[0]
#     pitch = euler[1]
#     yaw = euler[2]
#
#     rospy.loginfo("\n\nDEBUG current location x=", x, "y=", y)
#
# except:
#     rospy.loginfo("\n\nDEBUG tf lookup failed - cfX\n\n")
#     x = y = z = roll = pitch = yaw = 0


class CrazyflieFlightData:
    """A class for containing a crazyflie flight data.

    than later on, it will be used for rotem's alg'.
    """

    def __init__(self, tf_prefix):
        """Point_cloud contain a Point(x,y,z, index) per TOF sensors on CF_board.

        mean wile we have 4 TOF, sot len(point_cloud) = 4.
        index is the index of TOF sensor onboard.

        """
        self.point_cloud_last_timestamp = None
        self.point_cloud = []
        # crazyflie name
        self.tf_prefix = tf_prefix

        # Init listeners
        self.pc_sub = rospy.Subscriber("/" + prefix + "/point_cloud", PointCloud2,
                                       self.point_cloud_parser)
        self.pos_sub = rospy.Subscriber("/" + prefix + "/log_pos", GenericLogData,
                                        self.pos_parser)

    def point_cloud_parser(self, msg):
        """Each publicitation, theres' an array of 10 points."""
        self.point_cloud_last_timestamp = msg.header
        self.point_cloud = pc2.read_points_list(msg, skip_nans=True)
        rospy.loginfo("\n\nDEBUG point_cloud_parser: {}".format(msg.header))
        # for i in range(0, len(msg.data)):
        #     rospy.loginfo("\n\nin index{}: data is: {}".format(i, "test"))

    def pos_parser(self, msg):
        rospy.loginfo("\n\nlog_pos: {}".format(msg))
        pass

    # def update_point_cloud_data(self, point_list):
    #     # overwrite previeus point_list with
    #     self.point_cloud_list = point_list
    #
    #     # Or, append to preveius point_cloud updtate
    #     # self.point_cloud_list.append(point_list)


if __name__ == '__main__':
    rospy.init_node("drone_data_parser")

    # get cf name
    prefix = rospy.get_param("~tf_prefix")
    CrazyflieFlightData(prefix)

    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()
