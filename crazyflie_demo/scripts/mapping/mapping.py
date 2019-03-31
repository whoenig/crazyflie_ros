#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization.
Subscribed topics:/scan
Published topics:/map /map_metadata
Modification on Nathan Sprague's code 2/13/14
Author: Kuya Takami"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from nav_msgs.msg import Odometry
import time
import numpy as np
from scipy.stats import norm
import message_filters
from bresenham import bresenham
import gistfile1
myRes = 0.1

class Map(object):
    """
    The Map class stores an occupancy grid as a two dimensional numpy array.
    Public instance variables          :
        width -- Number of columns in the occupancy grid.
        height -- Number of rows in the occupancy grid.
        resolution -- Width of each grid square in meters.
        origin_x -- Position of the grid cell (0,0) in
        origin_y -- in the map coordinate system.
        grid -- numpy array with height rows and width columns.
    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=0., origin_y=0., resolution=myRes,
                 width=30, height=30):
        """ Construct an empty occupancy grid.
        Arguments : origin_x, origin_y -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,height -- The grid will have height rows and width
                                columns cells. width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
         The default arguments put (0,0) in the center of the grid.
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width/resolution
        self.height = height/resolution
        self.grid = 0.5*np.ones((height/resolution, width/resolution))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0.),
                               Quaternion(0., 0., 0., 1.))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100.
        grid_msg.data = list(np.round(flat_grid, decimals = 3))
        return grid_msg

    def set_cell(self, x, y, val)      :
        """ Set the value of a cell in the grid.

        Arguments                      :
            x, y - This is a point in the map coordinate frame.
            val - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method! Feel free to throw out
        point that land outside of the grid.
        """
        pass

class Mapper(object)                   :
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self)                 :

        """ Start the mapper. """
        rospy.init_node('mapper')
        self._map = Map()
        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages. This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans. Better to just drop
        # old scans and always work with the most recent available.
        self.position = [0.,0.,0.]

        gridaabb = gistfile1.AABB(np.array([0,0]), np.array([self._map.width,self._map.height]))
        grid = gistfile1.Grid(gridaabb)

        def callback(odom,scan):
            # rospy.Subscriber('odom',
                                    #  Odometry, self.odom_callback, queue_size=1)
            # rospy.Subscriber('base_scan_1',
                                    #  LaserScan, self.scan_callback, queue_size=1)

            # receive the sensor data for processing
            pos =  odom.pose.pose.position
            self.position = pos
            orientation = odom.pose.pose.orientation
            quaternion = (orientation.x,orientation.y,orientation.z, orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            self.rotation =  yaw
            Lresol = 1./myRes
            r = scan.ranges[0]
            xt = [self.position.x+22.5, self.position.y+3.5, self.rotation]

            scanAngles = np.linspace(scan.angle_min,scan.angle_max,len(scan.ranges),-1)
            lidar_local = np.array([xt[0]+scan.ranges*np.cos(scanAngles+xt[2]), xt[1]+(scan.ranges*np.sin(scanAngles+xt[2]))])

            xtg = [int(np.ceil(xt[0]*Lresol)),int(np.ceil(xt[1]*Lresol))]
            self._map.grid[xtg[1],xtg[0]]=0 # set the robot position grid as empty

            for k in range(0,len(scan.ranges)):

              rtl = np.ceil(lidar_local[:,k]*Lresol)
              rtli = [0,0]
              rtli[0] = int(rtl[0])
              rtli[1] = int(rtl[1])

              ray =  gistfile1.Ray()
              ray.origin = np.array([xtg[0],xtg[1]])

              ray.direction = np.array([np.cos(scanAngles[k]+xt[2]),np.sin(scanAngles[k]+xt[2])])
              ray.norm()


              end_pt = rtli
              traversal = gistfile1.AmanatidesTraversal(grid, ray, end_pt, scan.ranges[k]*Lresol)
              voxel = gistfile1.Voxel(grid)
              path = []
              pk = 0
              if traversal.initialize():
                while(True):
                    # Ignore while t_max is negative.
                    if traversal.get_t_interval()[1] < 0:
                        if not traversal.step():
                            break
                        continue
                    # ray.plot(axes, traversal.get_t_interval()[0], traversal.get_t_interval()[1])
                    ray_tmax_y = ray.origin + traversal.get_t_interval()[1]*ray.direction
                    # axes.plot(ray_tmax_y[0],ray_tmax_y[1], 'or', ms=10);

                    # print("voxel:", traversal.get_voxel())

                    path.append(traversal.get_voxel())
                    if np.linalg.norm(xtg-path[pk]) > scan.ranges[k]*Lresol:
                      break
                    # voxel.plot(axes, traversal.get_voxel()[0],traversal.get_voxel()[1])
                    if not traversal.step():
                        break
                    pk += 1
                # l =
                # if scanAngles[k]>1./2*np.pi or scanAngles[k]<-1./2*np.pi:
                #     l = bresenham(xtg,rtli)
                # print l.path
                # l.path = l.path[:,::-1]
                # temp = l.path
                #
                # if scanAngles[k]+self.position[2]>=3.0/4*np.pi or scanAngles[k]+self.position[2]<=-1.0/4*np.pi:
                #     for j in range(1,len(temp)):
                #         temp[j] = l.path[len(temp)-j]
                # l.path = temp
                # print l.path
                # for j in range(0,len(l.path)):
                if not len(path)==0:
                  self.AISM(path,scan.ranges[k],scan.ranges[k] < scan.range_max)

            # Now that the map is updated, publish it!
            rospy.loginfo("Scan is processed, publishing updated map.")
            self.publish_map()

        # perform synchronization

        odom_sub = message_filters.Subscriber('odom',Odometry)
        scan_sub = message_filters.Subscriber('base_scan',LaserScan)
        ts = message_filters.TimeSynchronizer([odom_sub,scan_sub],1)
        ts.registerCallback(callback)

        # for k in range(0,np.int(self._map.width*self._map.resolution) - 1):
        #     for j in range(0,np.int(self._map.height*self._map.resolution) - 1):
        #         if self._map.grid[k][j] > 0.75:
        #             self._map.grid[j][k] = 1


        # Latched publishers are used for slow changing topics like
        # maps. Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid,queue_size=10 ,latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata',
                                             MapMetaData, queue_size=10,latch=True)

        rospy.spin()

    def EISM(self,cell_path,r_s,oorange):
        n = len(cell_path)
        Prtl = np.zeros(n)
        Pnrtl = np.zeros(n)
        for k in range(0,n-1):
            index = cell_path[k]
            Prtl[k] = self._map.grid[index[1]][index[0]]
        Pnrtl = 1. - Prtl
        # initialize
        Prtl[0] = 0.
        Pnrtl[0] = 1.
        sigma_s = myRes*3.
        # optain Forward sensor model
        if not oorange:
          pz_xr = self.sensorFM(n,r_s,sigma_s)
        else:
          pz_xr = self.sensorFM(n,r_s*3.,sigma_s*3.)

        a = np.zeros(n)
        b = np.zeros(n)
        c = np.zeros(n)
        d = np.zeros(n)
        Pr_zxz = np.zeros(n)
        Pnr_zxz = np.zeros(n)
        for k in range(0, n - 1):
            if k == 0:
                a[0] = 0.
                b[0] = 1.
                c[0] = pz_xr[0]
            else:
                a[k] = a[k-1] + b[k-1]*pz_xr[k-1]*Prtl[k-1]
                b[k] = b[k-1]*Pnrtl[k-1]
                c[k] = b[k]*pz_xr[k]

        d[n-1] = 0.
        for k in range(n - 2, 0, -1):
            d[k] = d[k + 1] + b[k]*pz_xr[k + 1]*Prtl[k + 1]

        for k in range(0,n - 1):
            Pr_zxz[k] = a[k] + c[k]
            Pnr_zxz[k] = a[k] + d[k]

        for k in range(0, n - 1):
            index = cell_path[k]
            e = Prtl[k]*Pr_zxz[k]
            f = Pnrtl[k]*Pnr_zxz[k]
            if not (e+f) == 0.: #np.isnan(e/(e+f)):
              if oorange:
                self._map.grid[index[1]][index[0]] = e/(e+f)
              else:
                self._map.grid[index[1]][index[0]] = 0.1
            else:
              if not oorange:
                self._map.grid[index[1]][index[0]] = 0.1
    def AISM(self, cell_path, r_s, oorange):
        n = len(cell_path)
        Prtl = np.zeros(n)
        Pnrtl = np.zeros(n)
        for k in range(0,n-1):
            index = cell_path[k]
            Prtl[k] = self._map.grid[index[1]][index[0]]
        Pnrtl = 1. - Prtl
        rangelim = 4
        Prtl[0] = 1.0e-10
        Pnrtl[0] = 1.-Prtl[0]
        sigma = myRes*3.
        rho = 0.6
        sigma = 0.6
        # Prtl[n] = 1
        pz_xr = self.sensorFM(n,r_s*3.,sigma*3.)
        pr_zx = np.zeros(n)
        for k in range(0,n-1):
            if r_s*k/n < rangelim:
                pr_zx[k] = 0.3 + (rho/(sigma*np.sqrt(2*np.pi))+0.2)*np.exp(-1/2*(((rangelim-r_s*k/n))/sigma)**2)
            else:
                pr_zx[k] = 0.5 + (rho/(sigma*np.sqrt(2*np.pi)))*np.exp(-1/2*((rangelim-r_s*k/n)/sigma)**2)
        for k in range(0,n-1):
            ogmap = np.log(Prtl[k]/(1. - Prtl[k])) + np.log(pr_zx[k]/(1. - pr_zx[k]))
            if ogmap == float('-inf'):
                print Prtl[k] , 'Pr_zx', pr_zx[k]
            self._map.grid[index[1]][index[0]] = 1. - 1./(1. + np.exp(ogmap))

    def sensorFM(self,n,r_s,sigma_s):
        x = np.linspace(0.,r_s,n)
        model = norm.pdf(x,loc=r_s,scale=sigma_s)
        # return model
        model[0:n-3] = 1.0e-10
        return model*5.

    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)
        # rospy.signal_shutdown("stop spin")

    def odom_callback(self,odom):
        # global myOdom = odom
        pos =  odom.pose.pose.position
        self.position[0] = pos.x
        self.position[1] = pos.y
        orientation = odom.pose.pose.orientation
        quaternion = (orientation.x,orientation.y,orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.position[2] =  yaw


    def scan_callback(self, scan)      :
        """ Update the map on every scan callback. """
        # Fill some cells in the map just so we can see that something is
        # being published.
        Lresol = 1/myRes
        r = scan.ranges[0]
        xt = [self.position[0]+1, self.position[1]+1, self.position[2]]
        # for k in range(0,len(scan.ranges)-1):
        scanAngles = np.linspace(scan.angle_max,scan.angle_min,len(scan.ranges))
        lidar_local = np.array([xt[0]+scan.ranges*np.cos(scanAngles+xt[2]), xt[1]-(scan.ranges*np.sin(scanAngles+xt[2]))])

        # print len(lidar_local[1])
        xtg = [int(np.ceil(xt[0]*Lresol)),int(np.ceil(xt[1]*Lresol))]
        self._map.grid[xtg[1],xtg[0]]=0 # set the robot position grid as empty

        for k in range(0,len(scan.ranges)-1):
            if scan.ranges[k]<scan.range_max:
                rtl = np.ceil(lidar_local[:,k]*Lresol)
                rtli = [0,0]
                rtli[0] = int(rtl[0])
                rtli[1] = int(rtl[1])
                l = bresenham(xtg,rtli)
                self.EISM(l.path,scan.ranges[k])
        # Now that the map is updated, publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()



if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass