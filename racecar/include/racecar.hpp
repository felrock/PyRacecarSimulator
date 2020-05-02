#pragma once
#include <vector>
#include <cmath>

/*
 * This file contains the object that will occupy the map at
 * any given time. LiDAR simulations should be independant of
 * which object it is cast from.
 */

typedef struct CarState
{
    float x, y;
    float theta;
    float velocity;
    float steer_angle;
    float angular_velocity;
    float slip_angle;
    bool st_dyn;
};

class Car
{
    public:

        // constructor
        Car(float WB, float FC, float H_CG, float L_F, float L_R,\
           float CS_F, float CS_R, float MASS, float I_Z,\
           float CRASH_THRESH, float WIDTH, float LENGTH, float MAX_STEER_VEL,\
           float MAX_STEER_ANG, float MAX_SPEED, float MAX_ACCEL,\
           float MAX_DECEL);

        // update kinectics of the car
        void updatePosition(float dt);

        // driver input
        void control(float speed, float steer_vel);
        void computeFromInput();

        // from simulator.cpp
        bool isCrashed(float* rays, int num_rays);

        // from precompute
        void setCarEdgeDistances(int num_rays,
                                float ang_min,
                                float scan_ang_inc,
                                float scan_dist_to_base);

        // st and ks kinectic updates
        void updateSingle(float dt);
        void updateNormal(float dt);

        // get and set states, handles as float arrays becuase of
        // cython
        void getState(float* state);
        void setState(float* state);
        void getScanPose(float scan_dist_to_base, float* pose);

    private:

        // driver inputs
        float input_speed;
        float input_steer;

        // directions from driver inputs
        float accel;
        float steer_ang_vel;

        // for kinetic calculation
        struct CarState cs; // current state
        std::vector<float> edge_distances; // distance to cars edge

        // for map occupation
        float WIDTH;
        float HEIGHT;

        // limits
        float MAX_ACCEL, MAX_DECEL, MAX_SPEED;
        float MAX_STEER_VEL, MAX_STEER_ANG;

        // pid value
        float KP;

        // parameters set at start up, does not change
        float WB; // wheelbase
        float FC; // Friction coefficient
        float H_CG; // height of car's CG
        float L_F; // length from CG to front axle
        float L_R; // length from CG to rear axle
        float CS_F; // cornering stiffness coeff for front axle
        float CS_R; // cornering stiffness coeff for rear axle
        float MASS; // car weight
        float I_Z; // moment of inertia about z axit from CG
        float CRASH_THRESH; // Threshold for crashed distance

        // use to be in functions..
        // Threshold for using normal kinetic update over single
        const float K_THRESH = 0.5;
        // deadband threshold to avoid flip flop
        const float ST_THRESH = 0.53;

        const float G = 9.82;
        const float PI = 3.145;
};
