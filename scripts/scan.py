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
        self.theta_inc = fov/num_rays

        # often used
        self.twopi = math.pi * 2

        # cache vectors to send to gpu
        self.output_vector = np.zeros(self.num_rays, dtype=np.float32)
        self.noise = np.zeros(self.num_rays, dtype=np.float32)
        self.input_vector = np.zeros((self.num_rays, 3), dtype=np.float32)


    def build(self, map_msg, mrx, theta_disc):
        """
            ros_map : Numpy array with map grid values(as float)
            resolution : Resolution of the map
            origin : State of car first
        """

        self.omap = range_libc.PyOMap(map_msg)
        self.scan_method = range_libc.PyCDDTCast(self.omap, mrx, theta_disc)


    def scan(self, x, y, theta):
        """
            x - Position x on the map
            y - Position y on the map
            theta - Direction on the map
        """

        # create ray list
        max_theta = theta + self.fov/2.0
        min_theta = theta - self.fov/2.0
        thetas = np.arange(min_theta,
                           max_theta,
                           self.theta_inc,
                           dtype=np.float32)

        self.input_vector[:, 0] = x
        self.input_vector[:, 1] = y
        self.input_vector[:, 2] = thetas

        # run ray marching
        self.scan_method.calc_range_many(self.input_vector,
                                         self.output_vector)

        return self.output_vector

if __name__ == '__main__':
    pass
