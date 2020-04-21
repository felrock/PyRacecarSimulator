import numpy as np
import range_libc
import math
import random

"""
    A 2D Lidar scan simulator using ray marching
"""

class ScanSimulator2D:

    def __init__(self, num_rays, fov, scan_std, rt_e, theta_disc):
        """
            num_beams  - Number of beams for the simulated LiDAR.
            fov        - Field of view
            scan_std   - The standard deviation of the scan
            rt_e       - Ray tracing Epsilon
            theta_disc - Theta Discretization
        """

        # these were used in the old scan 2d
        self.num_rays   = num_rays
        self.fov        = fov
        self.scan_std   = scan_std
        self.rt_e       = rt_e
        self.theta_disc = theta_disc
        self.twopi      = math.pi * 2

        # the vars above should be ros params..

        # get map stuff here
        # wait until map loaded...
        # or pass map as a param...



        self.scan_method = range_libc.PyRayMarchingGPU(oMap, self.max_rpx, self.theta_disc)

    def initMap(self, map_msg):
        # init map first time
        pass

    def setMap(self, map_msg):
        # there should be more stuff here
        self.omap = range_libc.PyOMap(map_msg)

    def scan(self, x, y, pose):

        theta_index = self.theta_disc*(pose.theta - self.fov/2.0)/self.twopi
        thetas = []
        for _ in xrange(self.num_rays):

            thetas.append(theta_index)

            theta_index += self.theta_disc
            # wrap around
            while theta_index >= self.theta_disc:
                theta_index -= self.theta_disc

        # create input vector
        # create output vector


        # run ray marching
        output = self.scan_method.numpy_calc_range((x, y, pose))
        return output


if __name__ == '__main__':
    pass
