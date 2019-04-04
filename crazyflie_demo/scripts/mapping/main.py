#!/usr/bin/env python

import rospy
import roslib
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf
import math
from math import sin, cos, pi, tan, atan2
import numpy as np
from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt
from scipy import interpolate

from localmap import localmap

pose = [0.0, 0.0, 0.0]


# ***********************************************************************
def handle_robot_pose(parent, child, pose ...
