#!/usr/bin/env python
""" Dynamic route is a incapsulation motivated mini_project.
    date: 2018-12-13

    Explanation:
        Coordination
        Publisher
        Listener
        GoalListener
    """

__author__ = "Gal Brandwine"
__version__ = "1.0"
__email__ = "gal080592@gmail.com"

import time
import logging
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped

from utils import csv_route_reader


# Getting goal from keyboard (future develop)

class Publisher:
    """A publisher class for PoseStamp messages. """

    def __init__(self, coordination_updater_input, crazy_logger):
        self.logger = logging.getLogger('crazy_logger')
        self.coordination_updater = coordination_updater_input
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = coordination_updater.worldframe
        self.msg.pose.position.x = coordination_updater.x
        self.msg.pose.position.y = coordination_updater.y
        self.msg.pose.position.z = coordination_updater.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

        self.pub = rospy.Publisher(coordination_updater.name, PoseStamped, queue_size=1)

    def update(self, x_input, y_inpuit, z_input):
        # self.logger.info("received Goal update: {}".format((x,y,z)))
        """Update PoseStamp coordination."""
        self.msg.pose.position.x = x_input
        self.msg.pose.position.y = y_inpuit
        self.msg.pose.position.z = z_input
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

    def publish(self):
        self.msg.header.seq += 1
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)


# coordination Class
class Coordination:
    """Class for holding drone updated coordination from vrpn. """

    def __init__(self, name, frame, tf_prefix, crazy_logger):
        self.logger = logging.getLogger('crazy_logger')
        # Name of message to subscribe to (the goal message, usual a "goal" massage).
        self.name = name
        # Name of frame from the VRPN, relevant per Crazyflie.
        self.frame = frame
        self.tf_prefix = tf_prefix  # Prefix of Crazyflie from lunch file.
        self.worldframe = None
        # capture initiation time
        self.timer = time.time()
        self.time_capture = True
        self.arrived_to_point = False
        self.point_accuracy = 0.25  # accuracy in meters.
        self.rate = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.theta = 0
        self.kill = False
        self.error_w = 0
        self.auc_dist = 0
        self.in_sphere_timer = 0.1

    def goal_update(self, x, y, z, w):
        # If arrived to desired coordination, update self coordination.
        # Update new_arrived_coordination:
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def location_update(self, x, y, z, w):
        """Update error.

        error = goal - vrpn.
        """

        # Calculate auclidian distance.
        self.auc_dist = np.linalg.norm(np.array((x, y, z)) - np.array((self.x, self.y, self.z)))
        self.error_w = self.w - w
        # print("points: {} {}".format((x, y, z), (self.x, self.y, self.z)))
        self.logger.info("dist: {}".format(self.auc_dist))
        if self.auc_dist < self.point_accuracy and self.arrived_to_point is False:
            # If entered accuracy sphere.
            if self.time_capture is True:
                # If entered just now to accuracy sphere:
                # Capture time of arrival to designated point, only once per point
                self.timer = time.time()
                self.time_capture = False
                self.logger.info("Coordination object: time_capture captured")

            elif time.time() > self.timer + self.in_sphere_timer:
                # I inside accuracy sphere more than X sec:
                # elif self.auc_dist < self.point_accuracy and time.time() > self.timer + 5:

                # Set 'arrived_to_point' flag to True, so Coordination_updater could ask for the next coordination
                self.arrived_to_point = True
                self.timer = 0
                self.logger.info("arrived to point: {}".format((self.x, self.y, self.z)))
        else:
            # If not inside accuracy sphere, a time capture is needed next time going inside accuracy sphere.
            self.time_capture = True


class Listener:
    """Class for holding a Coordination-shared-object.

    listening for PoseStamp messages from relevant vrpn child, to know its position according to
    vrpn's (0,0,0), and updating Coordination object.
    """

    def __init__(self, coordination_obj, crazy_logger):
        self.logger = logging.getLogger('crazy_logger')
        self.coordination_holder = coordination_obj

        temp_listener = "/" + coordination_updater.tf_prefix + "/vrpn/" + coordination_updater.frame + "/pose"
        rospy.Subscriber(temp_listener, PoseStamped, self.callback)
        # rospy.Subscriber("/crazyflie1/vrpn/Crazyflie_5/pose", PoseStamped, self.callback)
        self.logger.info("added vrpn listener: {}".format(temp_listener))

    def callback(self, recieved_msg):
        # want to work on the point in the "world" frame
        self.coordination_holder.location_update(recieved_msg.pose.position.x, recieved_msg.pose.position.y,
                                                 recieved_msg.pose.position.z, 0)


class GoalListener:
    """A class for listening to goal topic.

    the main goal is to update Coordination_holder about the now GOAL.
    """

    def __init__(self, coordination_obj, crazy_logger):
        self.coordination_holder = coordination_obj
        self.logger = logging.getLogger('crazy_logger')
        temp_listener = "/" + coordination_obj.tf_prefix + "/" + coordination_obj.name
        rospy.Subscriber(temp_listener, PoseStamped, self.goal_callback)
        self.logger.info("added Goal listener: {}".format(temp_listener))

    def goal_callback(self, recieved_goal_msg):
        # want to work on the point in the "world" frame
        self.coordination_holder.goal_update(recieved_goal_msg.pose.position.x, recieved_goal_msg.pose.position.y,
                                             recieved_goal_msg.pose.position.z, 0)


if __name__ == '__main__':
    rospy.init_node('dynamic_route', anonymous=False)

    # Create loggers.
    crazy_logger = logging.getLogger(rospy.get_param("~frame", "cf1"))
    ch = logging.StreamHandler()
    # create formatter and add it to the handlers.
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    # add the handlers to loggers.
    crazy_logger.addHandler(ch)

    # Init an coordination object.
    coordination_updater = Coordination(rospy.get_param("~name", "/cf1"), rospy.get_param("~frame", "cf1"),
                                        rospy.get_param("~tf_prefix", "cf1"), crazy_logger)

    # Init listener
    listener = Listener(coordination_updater, crazy_logger)
    # Init Goal listener
    goal_listener = GoalListener(coordination_updater, crazy_logger)

    # Get initial route start_point. aka first goal point
    coordination_updater.worldframe = rospy.get_param("~worldFrame", "/world")
    coordination_updater.r = rospy.get_param("~rate", 5)
    # coordination_updater.x = rospy.get_param("~x")
    # coordination_updater.y = rospy.get_param("~y")
    # coordination_updater.z = rospy.get_param("~z")

    # Init Publisher
    publisher = Publisher(coordination_updater, crazy_logger)

    rate = rospy.Rate(coordination_updater.r)

    # Dynamic square route parameters.
    # TODO: change to : load dynamic routes from csv.
    square_edge = 0.5
    z = coordination_updater.z
    square_route = [(coordination_updater.x - square_edge, coordination_updater.y - square_edge, z),
                    (coordination_updater.x + square_edge, coordination_updater.y - square_edge, z),
                    (coordination_updater.x + square_edge, coordination_updater.y + square_edge, z),
                    (coordination_updater.x - square_edge, coordination_updater.y + square_edge, z)]
    point_index = 0
    scale = 2

    # route = csv_route_reader.get_route(rospy.get_param("~route"))
    # Load relevant route from swarm_route.csv
    route = csv_route_reader.get_route(rospy.get_param("~route"), coordination_updater.tf_prefix)
    # crazy_logger.info("for {} the next route wase loaded: {}".format(coordination_updater.tf_prefix, route))

    # Publish first rally point:
    next_x, next_y, next_z = tuple([scale * coord for coord in map(float, route[point_index])])
    coordination_updater.x = next_x
    coordination_updater.y = next_y
    coordination_updater.z = next_z
    crazy_logger.info("First point: ({},{},{})".format(next_x, next_y, next_z))
    publisher.update(next_x, next_y, next_z)
    publisher.publish()
    point_index += 1

    while not rospy.is_shutdown():

        if coordination_updater.arrived_to_point is True:
            # If true, means that we arrived point accuracy threshold, and we can move on to the next coordination.
            coordination_updater.arrived_to_point = False

            # next_x, next_y, next_z = tuple(map(float, route[point_index]['coordination'].split(', ')))
            next_x, next_y, next_z = tuple([scale * coord for coord in map(float, route[point_index])])
            crazy_logger.info("Now go to: ({},{},{})".format(next_x, next_y, next_z))
            if point_index < len(route):
                point_index += 1

            # If route is roundish, and want to circle indefinitely.
            # point_index = point_index % len(route)

            # Update next point in route
            # crazy_logger.info("Now go to: {}".format(square_route[point_index]))
            # crazy_logger.info("Now go to: {}".format(route[point_index]['coordination']))
            publisher.update(next_x, next_y, next_z)

        publisher.publish()
        rate.sleep()
