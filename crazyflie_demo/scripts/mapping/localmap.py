#!/usr/bin/env python

import bresenham
from math import sin, cos, pi, tan, atan2, log
import math
from itertools import groupby
from operator import itemgetter
import tf
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped


class localmap:
    def __init__(self, height, width, resolution, morigin):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.punknown = -1.0
        self.localmap = [self.punknown] * int(self.width / self.resolution) * int(self.height / self.resolution)
        self.logodds = [0.0] * int(self.width / self.resolution) * int(self.height / self.resolution)
        self.origin = int(math.ceil(morigin[0] / resolution)) + int(
            math.ceil(width / resolution) * math.ceil(morigin[1] / resolution))
        self.pfree = log(0.3 / 0.7)
        self.pocc = log(0.9 / 0.1)
        self.prior = log(0.5 / 0.5)
        self.max_logodd = 100.0
        self.max_logodd_belief = 10.0
        self.max_scan_range = 1.0
        self.map_origin = morigin

    def updatemap(self, scandata, angle_min, angle_max, angle_increment, range_min, range_max, pose):

        robot_origin = int(pose[0]) + int(math.ceil(self.width / self.resolution) * pose[1])
        centreray = len(scandata) / 2 + 1
        for i in range(len(scandata)):
            if not math.isnan(scandata[i]):
                beta = (i - centreray) * angle_increment
                px = int(float(scandata[i]) * cos(beta - pose[2]) / self.resolution)
                py = int(float(scandata[i]) * sin(beta - pose[2]) / self.resolution)

                l = bresenham.bresenham([0, 0], [px, py])
                for j in range(len(l.path)):
                    lpx = self.map_origin[0] + pose[0] + l.path[j][0] * self.resolution
                    lpy = self.map_origin[1] + pose[1] + l.path[j][1] * self.resolution

                    if (0 <= lpx < self.width and 0 <= lpy < self.height):
                        index = self.origin + int(l.path[j][0] + math.ceil(self.width / self.resolution) * l.path[j][1])
                        if scandata[i] < self.max_scan_range * range_max:
                            if (j < len(l.path) - 1):
                                self.logodds[index] += self.pfree
                            else:
                                self.logodds[index] += self.pocc
                        else:
                            self.logodds[index] += self.pfree
                        if self.logodds[index] > self.max_logodd:
                            self.logodds[index] = self.max_logodd
                        elif self.logodds[index] < -self.max_logodd:
                            self.logodds[index] = -self.max_logodd
                        if self.logodds[index] > self.max_logodd_belief:
                            self.localmap[index] = 100
                        else:
                            self.localmap[index] = 0
                        self.localmap[self.origin] = 100.0
