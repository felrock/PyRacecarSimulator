import numpy as np
import range_libc
import math
import sys

"""
    A 2D Lidar scan simulator using ray marching
"""

class ScanSimulator2D:

    def __init__(self, num_rays, fov, scan_std, theta_disc=2000):
        """
            num_rays    - Number of beams for the simulated LiDAR.
            fov         - Field of view
            scan_std    - The standard deviation of the scan
            theta_disc  - Theta Discretization
        """

        # these were used in the old scan 2d
        self.num_rays = num_rays
        self.fov = fov
        self.scan_std = scan_std
        self.theta_disc = theta_disc

        # often used
        self.twopi = math.pi * 2

        # cache vectors to send to gpu
        self.output_vector = np.ones(self.num_rays-1, dtype=np.float32)
        self.input_vector = np.zeros((self.num_rays-1, 3), dtype=np.float32)

        self.hasMap = False

    def setMap(self, ros_map, max_range_px, resolution, origin):
        """
            ros_map : Numpy array with map grid values(as float)
            resolution : Resolution of the map
            origin : State of car first
        """

        self.omap = ros_map

        print origin[0], origin[1]
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.origin_c = math.cos(origin[2])
        self.origin_s = math.sin(origin[2])

        self.mrx = max_range_px
        self.res = resolution
        self.hasMap = True

        print self.mrx

    def setRaytracingMethod(self, method="CDDT"):
        """
            Set which Ray tracing method to use
        """

        if not self.hasMap:
            print "for set RaytracingMethod use setMap first"
            return

        if method == "CDDT":
            self.scan_method = range_libc.PyCDDTCast(self.omap, self.mrx, self.theta_disc)
            self.scan_method.prune()

        elif method == "RM":
            self.scan_method = range_libc.PyRayMarching(self.omap, self.mrx)


    def updateMap(self, ros_map):
        """
            Update current map, fix this
        """

        self.ros_map = ros_map

    def scan(self, x, y, theta):
        """
            x - Position x on the map
            y - Position y on the map
            theta - Direction on the map
        """

        if not self.hasMap:
            print "Doing a scan without a defined map"

        theta_min = theta - self.fov/2.0
        theta_max = theta +self.fov/2.0

        # create numpy array
        self.input_vector[:, 0] = x
        self.input_vector[:, 1] = y
        self.input_vector[:, 2] = np.linspace(theta_min, theta_max, self.num_rays-1)

        # run ray marching
        self.scan_method.calc_range_many(self.input_vector,
                                         self.output_vector)

        #for i in xrange(self.num_rays):
        #    self.output_vector[i] = self.scan_method.calc_range(*self.input_vector[i])

        # add some noise to the output
        self.noise = np.random.uniform(low=-self.scan_std,
                                       high=self.scan_std,
                                       size=self.num_rays-1)

        return self.output_vector #+ np.array(self.noise, dtype=np.float32)

    def transformToGrid(self, x, y, theta):
        """
            Range_libc seems to already do this..
        """
        x_t = x - self.origin_x
        y_t = y - self.origin_x

        x_rot = x_t*self.origin_c + y_t*self.origin_s
        y_rot = -x_t*self.origin_s + y_t*self.origin_c

        return x_rot, y_rot

if __name__ == '__main__':
    pass
