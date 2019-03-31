import matplotlib.pyplot as plt
import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches
import pylab

pylab.rcParams['figure.figsize'] = (10.0, 10.0)

class Ray:
  def __init__(self):
    self.origin = np.zeros((2))
    self.direction = np.zeros((2))

  def norm(self):
    self.direction = self.direction / np.linalg.norm(self.direction)

  def plot(self,axes,t_min,t_max, color=(0, 0, 0), zorder=10):
    x0 = self.origin[0]+t_min*self.direction[0]
    y0 = self.origin[1]+t_min*self.direction[1]
    d0 = self.direction[0]*(t_max-t_min)
    d1 = self.direction[1]*(t_max-t_min)
    axes.quiver(x0, y0, d0, d1, angles='xy', scale_units='xy', scale=1, width=0.003, zorder=zorder, color=color)

class AABB:
  def __init__(self, low, high):
    self.low = low
    self.high = high

  def intersects(self, ray):
    r_t_min = 0
    r_t_max = 0
    t_min = 0
    t_max = 0
    ty_min = 0
    ty_max = 0

    size = self.high - self.low;


    bounds_x1 = self.low[0];
    bounds_x2 = self.low[0];
    bounds_y1 = self.low[1];
    bounds_y2 = self.low[1];
    irayd = 1./ray.direction
    if (irayd[0] >= 0):
        bounds_x2 += size[0];
    else:
        bounds_x1 += size[0];
    if (irayd[1] >= 0):
        bounds_y2 += size[1];
    else:
        bounds_y1 += size[1];

    t_min =  (bounds_x1 - ray.origin[0]) * irayd[0];
    t_max =  (bounds_x2 - ray.origin[0]) * irayd[0];
    ty_min = (bounds_y1 - ray.origin[1]) * irayd[1];
    ty_max = (bounds_y2 - ray.origin[1]) * irayd[1];

    t_min = max(t_min, ty_min);
    t_max = min(t_max, ty_max);

    if (t_min < t_max) and (t_max > 0):
      r_t_min = t_min;
      r_t_max = t_max;
      return (True,r_t_min,r_t_max)
    return (False,-1,-1)


class Grid:
    def __init__(self, aabb):
        self.width = aabb.high[0]-aabb.low[0]
        self.height = aabb.high[1]-aabb.low[1]
        self.grid = np.zeros((self.width+1,self.height+1))
        self.aabb = aabb

    def plot(self, axes):
        for ix in range(self.aabb.low[0],self.aabb.high[0]):
            for iy in range(self.aabb.low[1], self.aabb.high[1]):
                low_x = ix
                low_y = iy
                high_x = (ix+1)
                high_y = (iy+1)
                verts = [
                    (low_x, low_y), # left, bottom
                    (low_x, high_y), # left, top
                    (high_x, high_y), # right, top
                    (high_x, low_y), # right, bottom
                    (0., 0.), # ignored
                    ]

                codes = [Path.MOVETO,
                         Path.LINETO,
                         Path.LINETO,
                         Path.LINETO,
                         Path.CLOSEPOLY,
                         ]

                path = Path(verts, codes)
                patch = patches.PathPatch(path, facecolor='white', lw=1)
                axes.add_patch(patch)
        #axes.set_xlim((-0.5,1.5))
#        axes.set_ylim((-0.5,1.5))

# Find the distance between "frac(s)" and "1" if ds > 0, or "0" if ds < 0.
def diff_distance(s,ds):
    if s < 0:
        s = s - int(-1+s)
    else:
        s = s - int(s)
    if ds > 0:
        return (1.-s)/ds
    else:
        ds = -ds
        return s/ds

class AmanatidesTraversal:
    def __init__(self, grid, ray, end_pt, srange):
        self.range = srange
        self.grid = grid
        self.ray = Ray()
        self.end_pt = end_pt
        self.ray.origin = ray.origin.copy()
        self.ray.direction = ray.direction.copy()
        if self.ray.direction[0] == 0:
            self.ray.direction[0] = np.finfo(float).eps#sys.float_info.min
        if self.ray.direction[1] == 0:
            self.ray.direction[1] = np.finfo(float).eps#sys.float_info.min
        self.t_min = 0

    def initialize(self):
        (cube_result, cube_hit_t_min, cube_hit_t_max) = self.grid.aabb.intersects(self.ray)
        if cube_result:
            cube_hit_point = self.ray.origin + (cube_hit_t_min) * self.ray.direction
            self.t_min = cube_hit_t_min
            self.cube_hit_t_min = cube_hit_t_min

#            print "DDA: Cube Hit Point:", cube_hit_point

            self.step_x = np.copysign(1., self.ray.direction[0])
            self.step_y = np.copysign(1., self.ray.direction[1])

            self.t_delta_x = (self.step_x / self.ray.direction[0])
            self.t_delta_y = (self.step_y / self.ray.direction[1])

            self.t_max_x = diff_distance(cube_hit_point[0], self.ray.direction[0])
            self.t_max_y = diff_distance(cube_hit_point[1], self.ray.direction[1])

            if cube_hit_point[0] < 0:
                cube_hit_point[0] -= 1
            if cube_hit_point[1] < 0:
                cube_hit_point[1] -= 1
            self.voxel = np.array(cube_hit_point, dtype=int)
            # print("DDA: Initial Voxel:" , self.voxel)
            '''
            this conditional solves the problem where the "cube_hit_point" is just
            outside the grid because of floating point imprecision.
            '''
            while self.voxel[0] < self.grid.aabb.low[0] or self.voxel[1] < self.grid.aabb.low[1]\
            or self.voxel[0] >= self.grid.aabb.high[0] or self.voxel[1] >= self.grid.aabb.high[1]:
                print("DDA: Skyping:", self.voxel)
                if not self.step():
                    return False

            return True
        else:
            return False

    def step(self):
        self.t_min = (min(self.t_max_x,self.t_max_y) + self.cube_hit_t_min)
        if (self.t_max_x < self.t_max_y):
            self.t_max_x += self.t_delta_x
            self.voxel[0] += self.step_x
            if self.voxel[0] >= self.grid.aabb.high[0] or self.voxel[0] < self.grid.aabb.low[0]:
                return False
        else:
            self.t_max_y += self.t_delta_y
            self.voxel[1] += self.step_y
            if self.voxel[1] >= self.grid.aabb.high[1] or self.voxel[1] < self.grid.aabb.low[1]:
                return False
        # print np.linalg.norm(self.ray.origin-self.voxel), " range: ",self.range
        # if (self.voxel[1] ==self.end_pt[1] and self.voxel[0] == self.end_pt[0]):
        #   return False
        # print self.ray.origin
        # print self.voxel
        # if np.linalg.norm(self.ray.origin-self.voxel) < self.range:
        #   print "distance: ", np.linalg.norm(self.ray.origin-self.voxel), "range: ", self.range

        return True

    def get_voxel(self):
        return self.voxel.copy()

    def get_t_interval(self):
        # transform to unit space again
        return (self.t_min,(min(self.t_max_x,self.t_max_y) + self.cube_hit_t_min))
    def get_range(self):
      return np.linalg.norm(self.ray.origin-self.voxel)

class Voxel:
    def __init__(self, grid):
        self.grid = grid

    def plot(self, axes, x, y):
        low_x = x
        low_y = y
        high_x = (x+1)
        high_y = (y+1)
        verts = [
            (low_x, low_y), # left, bottom
            (low_x, high_y), # left, top
            (high_x, high_y), # right, top
            (high_x, low_y), # right, bottom
            (0., 0.), # ignored
            ]

        codes = [Path.MOVETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.CLOSEPOLY,
                 ]

        path = Path(verts, codes)
        patch = patches.PathPatch(path, facecolor='white', edgecolor='r', lw=1)
        axes.add_patch(patch)

if __name__ == "__main__":
  fig = plt.figure()
  axes = fig.add_subplot(1,1,1)

  ray = Ray()
  ray.origin = np.array([-2.5,1.27])
  ray.direction = np.array([+1.0,1.0])
  ray.norm()

  gridaabb = AABB(np.array([-3,-2]), np.array([50,50]))
  grid = Grid(gridaabb)
  print("Grid N. Quadrants: (", grid.width, ",", grid.height, ")")
  grid.plot(axes)
  #ray.plot(axes, 0, 1.5, zorder=2)
  traversal = AmanatidesTraversal(grid, ray)

  voxel = Voxel(grid)

  if traversal.initialize():
      # this ray should be plotted outside the grid
      ray.plot(axes, 0, traversal.get_t_interval()[0])
      cube_hit = ray.origin + traversal.get_t_interval()[0]*ray.direction
      axes.plot(cube_hit[0],cube_hit[1], 'og', ms=10)

      while(True):
          # Ignore while t_max is negative.
          if traversal.get_t_interval()[1] < 0:
              if not traversal.step():
                  break
              continue
          ray.plot(axes, traversal.get_t_interval()[0], traversal.get_t_interval()[1])

          ray_tmax_y = ray.origin + traversal.get_t_interval()[1]*ray.direction
          axes.plot(ray_tmax_y[0],ray_tmax_y[1], 'or', ms=10);

          print("voxel:", traversal.get_voxel())
          voxel.plot(axes, traversal.get_voxel()[0],traversal.get_voxel()[1])
          if not traversal.step():
              break

  # increase plot limits by 10%
  xlim = np.array(axes.get_xlim())
  ylim = np.array(axes.get_ylim())
  xlim[0] -= np.linalg.norm(xlim)/10.0
  xlim[1] += np.linalg.norm(xlim)/10.0
  ylim[0] -= np.linalg.norm(ylim)/10.0
  ylim[1] += np.linalg.norm(ylim)/10.0
  axes.set_xlim(xlim)
  axes.set_ylim(ylim)
