import math
import numpy as np

# Euclidian Distance Transform c++ wrapper
import edt

"""
    Distance Transform,

    this is solved by range_libc
"""

# some of these parameter maybe should be used
def distance_2d(image, width, height, res, bound_val):

    dt = edt.edt(image, anisotropy=(res, res))

    return dt

if __name__ == '__main__':
    pass
