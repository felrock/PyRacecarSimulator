from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
import numpy as np
cimport numpy as np

cdef extern from "include/racecar.hpp":

    ctypedef struct CarState:
        pass

    cdef cppclass Car:

        Car(double WB, double FC, double H_CG, double L_F, double L_R, double CS_F,
            double CS_R, double MASS, double I_Z, double WIDTH, double LENGTH,
            double CRASH_THRESH, double MAX_STEER_VEL, double MAX_STEER_ANG,
            double MAX_SPEED, double MAX_ACCEL, double MAX_DECEL)

        # public
        void updatePosition(double dt)
        void control(double speed, double steer_vel)
        void computeFromInput()
        int isCrashed(float* rays, int num_rays, int poses)
        void setCarEdgeDistances(double num_rays, double ang_min,
                                double scan_ang_inc, double scan_dist_to_base)
        void updateSingle(double dt)
        void updateNormal(double dt)

        void setState(double* state)
        void getState(double* state)
        void getScanPose(double scan_dist_to_base, double* pose)
        double getMeanVelocity()
        double getTravelDistance()
        void getBound(int num_rays, double* bound_points)

        # for driving
        double total_velo
        double travel_dist
        double update_count

        double input_speed
        double input_steer

        double accel
        double steer_ang_vel

        CarState cs
        vector[double] edge_distances

        # constants
        double width
        double height

        double MAX_ACCEL
        double MAX_DECEL
        double MAX_STEER_VEL
        double MAX_STEER_ANG
        double KP

        double WB
        double FC
        double H_CG
        double L_F
        double L_R
        double CS_F
        double CS_R
        double MASS
        double I_Z
        double CRASH_THRESH

        double K_THRESH
        double ST_THRESH
        double G

cdef class PyCar:
    cdef Car *thisptr
    def __cinit__(self, double WB, double FC, double H_CG, double L_F, double L_R,
                  double CS_F, double CS_R, double MASS, double I_Z,
                  double CRASH_THRESH, double WIDTH, double LENGTH,
                  double MAX_STEER_VEL, double MAX_STEER_ANG, double MAX_SPEED,
                  double MAX_ACCEL, double MAX_DECEL):
        self.thisptr = new Car(WB, FC, H_CG, L_F, L_R, CS_F, CS_R, MASS, I_Z,
                              CRASH_THRESH, WIDTH, LENGTH, MAX_STEER_VEL,
                              MAX_STEER_ANG, MAX_SPEED, MAX_ACCEL, MAX_DECEL)
    def __dealloc__(self):
        del self.thisptr
    cpdef void updatePosition(self, double dt):
        self.thisptr.updatePosition(dt)
    cpdef void control(self, double speed, double steer):
        self.thisptr.control(speed, steer)
    cpdef void computeFromInput(self):
        self.thisptr.computeFromInput()
    cpdef int isCrashed(self, np.ndarray[float, ndim=1, mode="c"] rays,
                        int num_rays,
                        int poses):
        return self.thisptr.isCrashed(&rays[0], num_rays, poses)
    cpdef void setCarEdgeDistances(self, int num_rays, double ang_min,
                                  double scan_ang_inc, double scan_dist_to_base):
        self.thisptr.setCarEdgeDistances(num_rays, ang_min, scan_ang_inc,
                                        scan_dist_to_base)
    cpdef void getState(self, np.ndarray[double, ndim=1, mode="c"] state):
        self.thisptr.getState(&state[0])
    cpdef void setState(self, np.ndarray[double, ndim=1, mode="c"] state):
        self.thisptr.setState(&state[0])
    cpdef void getScanPose(self, double scan_dist_to_base,
                          np.ndarray[double, ndim=1, mode="c"] pose):
        self.thisptr.getScanPose(scan_dist_to_base, &pose[0])
    cpdef double getMeanVelocity(self):
        return self.thisptr.getMeanVelocity()
    cpdef double getTravelDistance(self):
        return self.thisptr.getTravelDistance()
    cpdef void getBound(self, int num_rays,
                                np.ndarray[double, ndim=1, mode="c"] bound_points):
        self.thisptr.getBound(num_rays, &bound_points[0])
