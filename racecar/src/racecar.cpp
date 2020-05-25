#pragma once

#include "include/racecar.hpp"
#include <iostream>

/* ************************************************************************** */
/* **********************************<class Car>***************************** */
/* ************************************************************************** */

Car::Car(double WB, double FC, double H_CG, double L_F, double L_R, double CS_F,
        double CS_R, double MASS, double I_Z, double CRASH_THRESH, double WIDTH,
        double LENGTH, double MAX_STEER_VEL, double MAX_STEER_ANG, double MAX_SPEED,
        double MAX_ACCEL, double MAX_DECEL)
{
    /*
     * Contructor for the Car class, for now it takes the car parameters as
     * function parameters.
     *
     */

    // Init car state
    cs = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0,\
          .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false,\
          .travel_dist=0.0, total_velo=0.0, update_count=0};
    accel = 0.0;
    steer_ang_vel = 0.0;
    input_speed = 0.0;
    input_steer = 0.0;

    // set car parameters
    this->WB = WB;
    this->FC = FC;
    this->H_CG = H_CG;
    this->L_F = L_F;
    this->L_R = L_R;
    this->CS_F = CS_F;
    this->CS_R = CS_R;
    this->MASS = MASS;
    this->I_Z = I_Z;
    this->CRASH_THRESH = CRASH_THRESH;
    this->WIDTH = WIDTH;
    this->LENGTH = LENGTH;
    this->MAX_STEER_VEL = MAX_STEER_VEL;
    this->MAX_STEER_ANG = MAX_STEER_ANG;
    this->MAX_SPEED = MAX_SPEED;
    this->MAX_ACCEL = MAX_ACCEL;
    this->MAX_DECEL = MAX_DECEL;

    // speed regulate
    KP = 2.0 * MAX_ACCEL / MAX_SPEED;
}

void Car::updatePosition(double dt)
{
    /*
        double dt : time delta

        Updates the car one step
    */

    computeFromInput();

    // save previous x and y
    double p_x = cs.x;
    double p_y = cs.y;

    // change threshold to avoid flip flop
    double thresh;
    if(cs.st_dyn)
    {
        thresh = ST_THRESH;
    }
    else
    {
        thresh = K_THRESH;
    }
    // low speeds use kinetic update without slip
    if(cs.velocity < thresh)
    {
        updateNormal(dt);
    }
    else
    {
        updateSingle(dt);
    }

    // get difference for travel distance
    double d_x = p_x - cs.x;
    double d_y = p_y - cs.y;
    cs.travel_dist += std::sqrt(d_x*d_x + d_y*d_y);
    cs.total_velo += cs.velocity;
    cs.update_count++;

    // limit values steering and speed
    cs.velocity = std::min(std::max(cs.velocity, -MAX_SPEED), MAX_SPEED);
    cs.steer_angle = std::min(std::max(cs.steer_angle, -MAX_STEER_ANG),\
                                                        MAX_STEER_ANG);
}

double Car::getMeanVelocity()
{
    /*
     * The mean value over all the velo's seen in updatePose
     */

    return cs.total_velo / static_cast<double>(cs.update_count);
}

double Car::getTravelDistance()
{
    /*
     * The total distance travel from car init
     */

    return cs.travel_dist;
}

void Car::computeFromInput()
{
    /*
     * Sets the actions values to bounded values and acts like a pid
     * controller for acceleration.
     */

    // update accel and steer with inputs
    double dif_speed = input_speed - cs.velocity;
    if(cs.velocity > 0)
    {
        if(dif_speed > 0)
        {
            accel = std::min(std::max(KP*dif_speed, -MAX_ACCEL), MAX_ACCEL);
        }
        else
        {
            accel = -MAX_DECEL;
        }
    }
    else
    {
        if(dif_speed > 0)
        {
            accel = MAX_DECEL;
        }
        else
        {
            accel = std::min(std::max(KP*dif_speed, -MAX_ACCEL), MAX_ACCEL);
        }
    }

    // rate of steering is fixed, either max right or left
    double dif_steer = input_steer - cs.steer_angle;
    if(std::abs(dif_steer) > 0.0001)
    {
        if(dif_steer > 0)
        {
            steer_ang_vel = MAX_STEER_VEL;
        }
        else
        {
            steer_ang_vel = -MAX_STEER_VEL;

        }

    }
    else
    {
        steer_ang_vel = 0;
    }
}

void Car::updateNormal(double dt)
{
    /*
     * dt : time delta
     * Single track dynamics, the same as racecar_simulator
     */

    // compute first derivatives
    double x_dot = cs.velocity * std::cos(cs.theta);
    double y_dot = cs.velocity * std::sin(cs.theta);
    double theta_dot = cs.velocity / WB * std::tan(cs.steer_angle);
    double v_dot = accel;
    double steer_ang_dot = steer_ang_vel;

    // update
    cs.x += x_dot * dt;
    cs.y += y_dot * dt;
    cs.theta += theta_dot * dt;
    cs.velocity += v_dot * dt;
    cs.steer_angle += steer_ang_dot * dt;
    cs.angular_velocity = 0;
    cs.slip_angle = 0;
    cs.st_dyn = false;
}

void Car::updateSingle(double dt)
{
    /*
     * Simpler Single track dynamic, excluded slippage
     */

    // compute first derivatives
    double x_dot = cs.velocity * std::cos(cs.theta + cs.slip_angle);
    double y_dot = cs.velocity * std::sin(cs.theta + cs.slip_angle);
    double theta_dot = cs.angular_velocity;
    double v_dot = accel;
    double steer_ang_dot = steer_ang_vel;

    // calculations to reduce line length
    double r_val = G * L_R - accel * H_CG;
    double f_val = G * L_F + accel * H_CG;
    double vel_ratio = cs.angular_velocity / cs.velocity;
    double first_term = FC / (cs.velocity * (L_R + L_F));


    double theta_double_dot = (FC * MASS / (I_Z * WB)) *\
                              (L_F * CS_F * cs.steer_angle * r_val + \
                               cs.slip_angle * (L_R * CS_R * f_val - L_F * CS_F * r_val) -\
                              vel_ratio * (std::pow(L_F, 2) * CS_F * r_val +\
                              std::pow(L_R, 2) * CS_R * f_val));

    double slip_angle_dot = first_term *\
                            (CS_F * cs.steer_angle * (r_val) -\
                            cs.slip_angle * (CS_R * f_val + CS_F * r_val) +\
                            vel_ratio * (CS_R * L_R * f_val - CS_F * L_F *r_val))-\
                            cs.angular_velocity;

    // update current state
    cs.x += x_dot * dt;
    cs.y += y_dot * dt;
    cs.theta += theta_dot * dt;
    cs.velocity += v_dot * dt;
    cs.steer_angle += steer_ang_dot * dt;
    cs.angular_velocity += theta_double_dot * dt;
    cs.slip_angle += slip_angle_dot * dt;
    cs.st_dyn = true;
}

void Car::setCarEdgeDistances(int num_rays, double min_ang, double scan_ang_inc,
                             double scan_dist_to_base)
{
    /*
     * Gets distanced from the center of the car(+offset) to the edges.
     * This is to know where the lidar is outside the car.
     */

    edge_distances = std::vector<double>(num_rays);
    double d1, d2;
    double dist_to_side = WIDTH / 2.0;
    double dist_to_front = WB - scan_dist_to_base;
    double dist_to_back = scan_dist_to_base;
    double cur_ang = min_ang;

    for(int i=0; i < num_rays; ++i)
    {
        cur_ang += scan_ang_inc;

        if(cur_ang > 0.0)
        {
            if(cur_ang < PI /2.0)
            {
                d1 = dist_to_side / std::sin(cur_ang);
                d2 = dist_to_front / std::cos(cur_ang);
                edge_distances[i] = std::min(d1, d2);
            }
            else
            {
                d1 = dist_to_side / std::sin(cur_ang-PI/2.0);
                d2 = dist_to_back / std::cos(cur_ang-PI/2.0);
                edge_distances[i] = std::min(d1, d2);
            }
        }
        else
        {
            if(cur_ang == 0.0)
                cur_ang += 0.0001; // avoid zero division

            if(cur_ang > -PI /2.0)
            {
                d1 = dist_to_side / std::sin(-cur_ang);
                d2 = dist_to_front / std::cos(-cur_ang);
                edge_distances[i] = std::min(d1, d2);
            }
            else
            {
                d1 = dist_to_side / std::sin(-cur_ang-PI/2.0);
                d2 = dist_to_back / std::cos(-cur_ang-PI/2.0);
                edge_distances[i] = std::min(d1, d2);
            }
        }
    }
}

void Car::control(double speed, double steer_vel)
{
    /*
     * speed : desired speed as an action
     * steer : desired angle of the "steering wheel"
     */

    input_speed = speed;
    input_steer = steer_vel;
}

int Car::isCrashed(float* rays, int num_rays, int poses)
{
    /*
     * rays is the same length as edge_distances.
     * if a ray is small enough to be within the
     * edge distances set, the car has crashed
     *
     * Poses is how many scans ray cotains, makes checking
     * more than one scene for crashes faster
     */
    int i;
    for(i=1; i < poses+1; ++i)
    {
        for(size_t j=0; j < num_rays; ++j)
        {
            if((rays[(i-1)*num_rays + j] - edge_distances[j]) < CRASH_THRESH)
            {
                return i-1;
            }
        }
    }

    return -i;
}

void Car::setState(double *state)
{
    /*
     * Set state to the CarState, used as a float array to
     * pass it to python(numpy)
     */

    cs.x = state[0];
    cs.y = state[1];
    cs.theta = state[2];
    cs.velocity = state[3];
    cs.steer_angle = state[4];
    cs.angular_velocity = state[5];
    cs.slip_angle = state[6];
    if(state[7] > 0.0)
    {
        cs.st_dyn = true;
    }
    else
    {
        cs.st_dyn = false;
    }
    cs.travel_dist = state[8];
    cs.total_velo = state[9];
    cs.update_count = static_cast<int>(state[10]);
}

void Car::getState(double *state)
{
    /*
     * Same as the set function
     */

    // write state
    state[0] = cs.x;
    state[1] = cs.y;
    state[2] = cs.theta;
    state[3] = cs.velocity;
    state[4] = cs.steer_angle;
    state[5] = cs.angular_velocity;
    state[6] = cs.slip_angle;
    state[7] = static_cast<bool>(cs.st_dyn);
    state[8] = cs.travel_dist;
    state[9] = cs.total_velo;
    state[10] = static_cast<int>(cs.update_count);

}

void Car::getScanPose(double scan_dist_to_base, double* pose)
{
    /*
     * Get a position with an offset to the center
     */

    pose[0] = cs.x + scan_dist_to_base * std::cos(cs.theta);
    pose[1] = cs.y + scan_dist_to_base * std::sin(cs.theta);
    pose[2] = cs.theta;
}

void Car::getBound(int num_rays, double* bound_points)
{
    /*
     * Similar to setCarEdge but the edge ecloses the car with bound_points
     * with the density specified with num_rays
     */

    // get the bound_points that enclose the car
    double dx, dy, t_dist;
    double dist_to_side = WIDTH / 2.0;
    double dist_to_front = LENGTH / 2.0;
    double dist_to_back = LENGTH / 2.0;
    double cur_ang = -cs.theta;
    double ang_inc = 2.0*PI/num_rays; // 360 degrees over rays

    for(int i=0; i < num_rays; ++i)
    {
        cur_ang += ang_inc;

        if(cur_ang > 0.0)
        {
            if(cur_ang < PI /2.0)
            {
                t_dist = std::min(dist_to_side / std::sin(cur_ang),\
                                 dist_to_front / std::cos(cur_ang));
                dx = t_dist * std::cos(cur_ang);
                dy = t_dist * std::sin(cur_ang);

                bound_points[i] = dx;
                bound_points[i+1] = dy;

            }
            else
            {
                t_dist = std::min(dist_to_side / std::sin(cur_ang-PI/2.0),\
                                 dist_to_back / std::cos(cur_ang-PI/2.0));
                dx = t_dist * std::cos(cur_ang);
                dy = t_dist * std::sin(cur_ang);

                bound_points[i] = dx;
                bound_points[i+1] = dy;
            }
        }
        else
        {
            if(cur_ang == 0.0)
                cur_ang += 0.0001; // avoid zero division

            if(cur_ang > -PI /2.0)
            {
                t_dist = std::min(dist_to_side / std::sin(-cur_ang),\
                                dist_to_front / std::cos(-cur_ang));
                dx = t_dist * std::cos(cur_ang);
                dy = t_dist * std::sin(cur_ang);

                bound_points[i] = dx;
                bound_points[i+1] = dy;
            }
            else
            {
                t_dist = std::min(dist_to_side / std::sin(-cur_ang-PI/2.0),\
                                dist_to_back / std::cos(-cur_ang-PI/2.0));
                dx = t_dist * std::cos(cur_ang);
                dy = t_dist * std::sin(cur_ang);
                bound_points[i] = dx;
                bound_points[i+1] = dy;
            }
        }
    }

}

/* ************************************************************************** */
/* *********************************</class Car>***************************** */
/* ************************************************************************** */
