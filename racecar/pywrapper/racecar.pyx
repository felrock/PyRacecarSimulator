from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
import numpy as np
cimport numpy as np

cdef extern from "include/racecar.hpp":

    ctypedef struct CarState:
        pass

    cdef cppclass Car:

        Car(float WB, float FC, float H_CG, float L_F, float L_R, float CS_F,
            float CS_R, float MASS, float I_Z, float WIDTH, float LENGTH,
            float CRASH_THRESH, float MAX_STEER_VEL, float MAX_STEER_ANG,
            float MAX_SPEED, float MAX_ACCEL, float MAX_DECEL)

        # public
        void updatePosition(float dt)
        void control(float speed, float steer_vel)
        void computeFromInput()
        bool isCrashed(float* rays, int num_rays)
        void setCarEdgeDistances(float num_rays, float ang_min,
                                float scan_ang_inc, float scan_dist_to_base)
        void updateSingle(float dt)
        void updateNormal(float dt)
        void setState(float* state)
        void getState(float* state)
        void getScanPose(float scan_dist_to_base, float* pose)

        # for driving
        float input_speed
        float input_steer

        float accel
        float steer_ang_vel

        CarState cs
        vector[float] edge_distances

        # constants
        float width
        float height

        float MAX_ACCEL
        float MAX_DECEL
        float MAX_STEER_VEL
        float MAX_STEER_ANG
        float KP

        float WB
        float FC
        float H_CG
        float L_F
        float L_R
        float CS_F
        float CS_R
        float MASS
        float I_Z
        float CRASH_THRESH

        float K_THRESH
        float ST_THRESH
        float G

cdef class PyCar:
    cdef Car *thisptr
    def __cinit__(self, float WB, float FC, float H_CG, float L_F, float L_R,
                  float CS_F, float CS_R, float MASS, float I_Z,
                  float CRASH_THRESH, float WIDTH, float LENGTH,
                  float MAX_STEER_VEL, float MAX_STEER_ANG, float MAX_SPEED,
                  float MAX_ACCEL, float MAX_DECEL):
        self.thisptr = new Car(WB, FC, H_CG, L_F, L_R, CS_F, CS_R, MASS, I_Z,
                              CRASH_THRESH, WIDTH, LENGTH, MAX_STEER_VEL,
                              MAX_STEER_ANG, MAX_SPEED, MAX_ACCEL, MAX_DECEL)
    def __dealloc__(self):
        del self.thisptr
    cpdef void updatePosition(self, float dt):
        self.thisptr.updatePosition(dt)
    cpdef void control(self, float speed, float steer):
        self.thisptr.control(speed, steer)
    cpdef void computeFromInput(self):
        self.thisptr.computeFromInput()
    cpdef bool isCrashed(self, np.ndarray[float, ndim=1, mode="c"] rays,
                        int num_rays):
        return self.thisptr.isCrashed(&rays[0], num_rays)
    cpdef void setCarEdgeDistances(self, int num_rays, float ang_min,
                                  float scan_ang_inc, float scan_dist_to_base):
        self.thisptr.setCarEdgeDistances(num_rays, ang_min, scan_ang_inc,
                                        scan_dist_to_base)
    cpdef void getState(self, np.ndarray[float, ndim=1, mode="c"] state):
        self.thisptr.getState(&state[0])
    cpdef void setState(self, np.ndarray[float, ndim=1, mode="c"] state):
        self.thisptr.setState(&state[0])
    cpdef void getScanPose(self, float scan_dist_to_base,
                          np.ndarray[float, ndim=1, mode="c"] pose):
        self.thisptr.getScanPose(scan_dist_to_base, &pose[0])
