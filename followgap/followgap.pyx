from libcpp cimport vector
import numpy as np
cimport numpy as np


cdef extern from "followgap.hpp":

    cdef cppclass FollowGap:


        FollowGap(int ws, float md, float ma, float angle_inc)

        float eval(float* lidar, int size)

        int window_size
        float max_distance
        float max_angle
        float angle_inc


cdef class PyFollowGap:
    cdef FollowGap *thisptr
    def __cinit__(self, int ws, float md, float ma, float angle_inc):
        self.thisptr = new FollowGap(ws, md, ma, angle_inc)

    def __dealloc__(self):
        del self.thisptr

    cpdef float eval(self, np.ndarray[float, ndim=1, mode="c"] lidar, int size):
        return self.thisptr.eval(&lidar[0], size)

