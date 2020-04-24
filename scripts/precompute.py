import numpy as np
import math

def get_car_distances(scan_beams, wheelbase, width, scan_dist_to_base, angle_min, scan_ang_inc):
    """
        Precompute distances from lidar to the edge of the car
    """

    distances = [0]*scan_beams
    d_s = width/2.0 # distance sides
    d_f = wheelbase - scan_dist_to_base # distance front
    d_b = scan_dist_to_base # distance back
    pi = math.pi

    for i in xrange(scan_beams):
        ang = angle_min + i*scan_ang_inc

        if ang > 0:
            if ang < math.pi / 2.0:
                distances[i] = min(d_s / math.sin(ang), d_f / math.cos(ang))
            else:
                distances[i] = min(d_s / math.sin(ang - pi/2.0), d_b /math.cos(ang - pi/2.0))
        else:
            if ang > -math.pi / 2.0:
                distances[i] = min(d_s / math.sin(-ang), d_f / math.cos(-ang))
            else:
                distances[i] = min(d_s / math.sin(-ang - pi/2.0), d_f / math.cos(-ang - pi/2.0))

    # return as an numpy array
    return np.array(distances)

def get_cosines(num_rays, angle_min, scan_ang_inc):
    """
        Pre-compute lidar angles
    """

    cosines = [0]*num_rays

    for i in xrange(num_rays):
        cosines[i] = math.cos(angle_min + i*scan_ang_inc)

    return np.array(cosines)


if __name__ == '__main__':
    pass
