import numpy as np
import range_libc
import math
import sys
import time

"""
    A 2D Lidar scan simulator using ray marching
"""

class ScanSimulator2D:

    def __init__(self, num_rays, fov, scan_std, batch_size=100):
        """
            num_rays    - Number of beams for the simulated LiDAR.
            fov         - Field of view
            scan_std    - The standard deviation of the scan
            theta_disc  - Theta Discretization
        """

        self.batch_size = batch_size

        # these were used in the old scan 2d
        self.num_rays = num_rays
        self.fov = fov
        self.scan_std = scan_std

        # often used
        self.twopi = math.pi * 2

        # cache vectors to send to gpu
        self.output_vector = np.zeros(self.num_rays, dtype=np.float32)
        self.noise = np.zeros(self.num_rays, dtype=np.float32)
        self.input_vector = np.zeros((self.num_rays, 3), dtype=np.float32)

        # cache vectors to send to gpu
        self.output_vector_many = np.zeros(batch_size*self.num_rays,
                                            dtype=np.float32)
        self.input_vector_many = np.zeros((batch_size*self.num_rays, 3),
                                            dtype=np.float32)

        self.hasMap = False

    def setMap(self, ros_map, max_range_px, resolution, origin):
        """
            ros_map : Numpy array with map grid values(as float)
            resolution : Resolution of the map
            origin : State of car first
        """

        self.omap = ros_map

        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.origin_c = math.cos(origin[2])
        self.origin_s = math.sin(origin[2])

        self.mrx = max_range_px
        self.res = resolution
        self.hasMap = True

    def setRaytracingMethod(self, method="RM"):
        """
            Set which Ray tracing method to use
        """

        if not self.hasMap:
            print "for set RaytracingMethod use setMap first"
            return

        if method == "RM":
            self.scan_method = range_libc.PyRayMarching(self.omap,
                                                        self.mrx)
        elif method == "RMGPU":
            self.scan_method = range_libc.PyRayMarchingGPU(self.omap,
                                                           self.mrx)
        else:
            print "Only ray marching is supported"
            sys.exit()

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

        self.input_vector[0, 0] = x
        self.input_vector[0, 1] = y
        self.input_vector[0, 2] = theta

        # run ray marching
        self.scan_method.calc_range_many(self.input_vector,
                                         self.output_vector,
                                         self.fov,
                                         self.num_rays)

        # add some noise to the output
        #self.noise = np.array(np.random.normal(0, self.scan_std, self.num_rays), dtype=np.float32)

        return self.output_vector #+ self.noise

    def scanMany(self, poses):
        """
            Take many poses on the map and simulate them
            ignore adding noise
        """

        for i in xrange(self.batch_size):

            x = poses[i][0]
            y = poses[i][1]
            theta = poses[i][2]

            self.input_vector_many[i*self.num_rays, 0] = x
            self.input_vector_many[i*self.num_rays, 1] = y
            self.input_vector_many[i*self.num_rays, 2] = theta

        # scanning all
        self.scan_method.calc_range_many(self.input_vector_many,
                                        self.output_vector_many,
                                        self.fov,
                                        self.num_rays)

        return self.output_vector_many

if __name__ == '__main__':
    pass
