import numpy as np
import range_libc
import math
import sys

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

    def setRaytracingMethod(self, method="CDDT"):
        """
            Set which Ray tracing method to use
        """

        if not self.hasMap:
            print "for set RaytracingMethod use setMap first"
            return

        if method == "RM":
            self.scan_method = range_libc.PyRayMarching(self.omap, self.mrx)

        elif method == "RMGPU":
            self.scan_method = range_libc.PyRayMarchingGPU(self.omap, self.mrx)

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
        self.input_vector[:, 2] = np.linspace(theta_min, theta_max, self.num_rays)

        # run ray marching
        self.scan_method.calc_range_many(self.input_vector,
                                         self.output_vector)

        # add some noise to the output
        self.noise = np.array(np.random.normal(0, self.scan_std, self.num_rays), dtype=np.float32)

        return self.output_vector + self.noise

    def scanMany(self, poses):
        """
            Take many poses on the map and simulate them
            ignore adding noise
        """

        for i in xrange(batch_size):
            x = poses[0]
            y = poses[1]
            theta = poses[2]

            theta_min = theta - self.fov/2.0
            theta_max = theta + self.fov/2.0

            # create numpy array
            span = (i*self.num_rays, (i+1)*self.num_rays)
            self.input_vector_many[span, 0] = x
            self.input_vector_many[span, 1] = y
            self.input_vector_many[span, 2] = np.linspace(theta_min,
                                             theta_max, self.num_rays)

        self.scan_method.calc_range_many(self.input_vector_many,
                                        self.output_vector_many)

        return self.output_vector_many

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
