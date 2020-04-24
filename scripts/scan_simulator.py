import numpy as np
import range_libc
import math
import random

"""
    A 2D Lidar scan simulator using ray marching
"""

class ScanSimulator2D:

    def __init__(self, num_rays, fov, scan_std, ros_map, res, scan_max_range, theta_disc=2000):
        """
            num_rays    - Number of beams for the simulated LiDAR.
            fov         - Field of view
            scan_std    - The standard deviation of the scan
            theta_disc  - Theta Discretization
            ros_map     - Map of the track
            res         - Resolution of the map
            max_range_m - Max distance in meters
        """

        # these were used in the old scan 2d
        self.num_rays = num_rays
        self.fov = fov
        self.scan_std = scan_std
        self.theta_disc = theta_disc
        self.ros_map = ros_map
        self.scan_max_range = scan_max_range
        self.res = res

        # often used
        self.twopi = math.pi * 2

        # create map object
        self.omap = range_libc.PyOMap(ros_map)
        self.mrx = int(self.scan_max_range / self.res)

        # create ray marching object
        self.scan_method = range_libc.PyCDDTCast(self.omap, self.mrx, theta_disc)
        #self.scan_method = range_libc.PyRayMarching(self.omap, self.mrx)

        # cache vectors to send to gpu
        self.output_vector = np.ones(self.num_rays, dtype=np.float32)
        self.input_vector = np.reshape(np.ones(self.num_rays*3), (self.num_rays, 3))

    def updateMap(self, ros_map):
        """
            Update current map
        """

        self.ros_map = ros_map

    def scan(self, x, y, theta):
        """
            x - Position x on the map
            y - Position y on the map
            theta - Direction on the map
        """

        theta_index = self.theta_disc*(theta - self.fov/2.0)/self.twopi
        thetas = []
        for _ in xrange(self.num_rays):

            thetas.append(theta_index)
            theta_index += self.theta_disc

            # wrap around
            while theta_index >= self.theta_disc:
                theta_index -= self.theta_disc

        # create numpy array
        input_vector = np.array(map(lambda t:(x, y, t), thetas), dtype=np.float32)

        # run ray marching
        self.scan_method.calc_range_many(input_vector,
                                         self.output_vector)

        # add some noise to the output
        self.output_vector = np.random.normal(self.output_vector,
                                              scale=self.scan_std)
        return self.output_vector


if __name__ == '__main__':
    pass
