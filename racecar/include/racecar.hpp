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
    double x, y;
    double theta;
    double velocity;
    double steer_angle;
    double angular_velocity;
    double slip_angle;
    bool st_dyn;
    // extended
    double travel_dist;
    double total_velo;
    int update_count;
};

class Car
{
    public:

        // constructor
        Car(double WB, double FC, double H_CG, double L_F, double L_R,\
           double CS_F, double CS_R, double MASS, double I_Z,\
           double CRASH_THRESH, double WIDTH, double LENGTH, double MAX_STEER_VEL,\
           double MAX_STEER_ANG, double MAX_SPEED, double MAX_ACCEL,\
           double MAX_DECEL);

        // update kinectics of the car with actions
        void updatePosition(double dt);

        // driver input
        void control(double speed, double steer_vel);
        void computeFromInput();

        // from simulator.cpp
        int isCrashed(float* rays, int num_rays, int poses);

        // from precompute
        void setCarEdgeDistances(int num_rays,
                                double ang_min,
                                double scan_ang_inc,
                                double scan_dist_to_base);

        // st and ks kinectic updates
        void updateSingle(double dt);
        void updateNormal(double dt);

        // get and set states, handles as double arrays becuase of
        // cython
        void getState(double* state);
        void setState(double* state);
        void getScanPose(double scan_dist_to_base, double* pose);
        double getMeanVelocity();
        double getTravelDistance();

        // for drawing
        void getBound(int num_rays, double* bound_points);

    private:

        // added for rewards
        double total_velo; // used to get avg speed over updates
        double travel_dist; // total distance traveled
        int update_count;

        // driver inputs
        double input_speed;
        double input_steer;

        // directions from driver inputs
        double accel;
        double steer_ang_vel;

        // for kinetic calculation
        struct CarState cs; // current state
        std::vector<double> edge_distances; // distance to cars edge

        // for map occupation
        double WIDTH;
        double LENGTH;

        // limits
        double MAX_ACCEL, MAX_DECEL, MAX_SPEED;
        double MAX_STEER_VEL, MAX_STEER_ANG;

        // pid value
        double KP;

        // parameters set at start up, does not change
        double WB; // wheelbase
        double FC; // Friction coefficient
        double H_CG; // height of car's CG
        double L_F; // length from CG to front axle
        double L_R; // length from CG to rear axle
        double CS_F; // cornering stiffness coeff for front axle
        double CS_R; // cornering stiffness coeff for rear axle
        double MASS; // car weight
        double I_Z; // moment of inertia about z axit from CG
        double CRASH_THRESH; // Threshold for crashed distance

        // use to be in functions..
        // Threshold for using normal kinetic update over single
        const double K_THRESH = 0.5;
        // deadband threshold to avoid flip flop
        const double ST_THRESH = 0.53;

        const double G = 9.81;
        const double PI = 3.145;
};
