#pragma once

#include "include/racecar.hpp"

/* ************************************************************************** */
/* **********************************<class Car>***************************** */
/* ************************************************************************** */

Car::Car(float WB, float FC, float H_CG, float L_F, float L_R, float CS_F,
        float CS_R, float MASS, float I_Z, float CRASH_THRESH, float WIDTH,
        float LENGTH, float MAX_STEER_VEL, float MAX_STEER_ANG, float MAX_SPEED,
        float MAX_ACCEL, float MAX_DECEL)
{

    // Init car state
    cs = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0,\
          .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
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
    this->MAX_STEER_VEL = MAX_STEER_VEL;
    this->MAX_STEER_ANG = MAX_STEER_ANG;
    this->MAX_SPEED = MAX_SPEED;
    this->MAX_ACCEL = MAX_ACCEL;
    this->MAX_DECEL = MAX_DECEL;

    // speed regulate
    KP = 2.0 * MAX_ACCEL / MAX_SPEED;
}

void Car::updatePosition(float dt)
{
    /*
        float dt : time delta

        determine which kinetic method to use based on which was used
        previously(st_dyn) and the current velocity. When the current
        velocity is low, slippage is marginal.
    */

    // change according to input values
    computeFromInput();

    float thresh;
    if(cs.st_dyn)
    {
        thresh = K_THRESH;
    }
    else
    {
        thresh = ST_THRESH;
    }

    if(cs.velocity < thresh)
    {
        updateNormal(dt);
    }
    else
    {
        updateSingle(dt);
    }

    // limit values steering and speed
    cs.velocity = std::min(std::max(cs.velocity, -MAX_SPEED), MAX_SPEED);
    cs.steer_angle = std::min(std::max(cs.steer_angle, -MAX_STEER_VEL),\
                                                        MAX_STEER_VEL);
}

void Car::computeFromInput()
{
    // update accel and steer with inputs
    float dif_speed = cs.velocity - input_speed;
    accel = std::min(std::max(KP*dif_speed, -MAX_DECEL), MAX_ACCEL);

    // rate of steering is fixed, either max right or left
    float dif_steer = input_steer - cs.steer_angle;
    if(std::abs(dif_steer) > 0.0001)
    {
        if(dif_steer < 0)
        {
            steer_ang_vel = -MAX_STEER_VEL;
        }
        else
        {
            steer_ang_vel = MAX_STEER_VEL;

        }
    }
    else
    {
        steer_ang_vel = 0;
    }

}

void Car::updateNormal(float dt)
{
    // compute first derivatives
    float x_dot = cs.velocity * std::cos(cs.theta);
    float y_dot = cs.velocity * std::sin(cs.theta);
    float theta_dot = cs.velocity / WB * std::tan(cs.steer_angle);
    float v_dot = accel;
    float steer_ang_dot = steer_ang_vel;

    // update
    cs.x += x_dot * dt;
    cs.y += y_dot * dt;
    cs.theta += theta_dot * dt;
    cs.velocity += v_dot * dt;
    cs.steer_angle += steer_ang_dot * dt;
    cs.theta += theta_dot * dt;
    cs.angular_velocity = 0;
    cs.slip_angle = 0;
    cs.st_dyn = false;
}

void Car::updateSingle(float dt)
{
    // compute first derivatives
    float x_dot = cs.velocity * std::cos(cs.theta + cs.slip_angle);
    float y_dot = cs.velocity * std::sin(cs.theta + cs.slip_angle);
    float theta_dot = cs.angular_velocity;
    float v_dot = accel;
    float steer_ang_dot = steer_ang_vel;

    // calculations to reduce line length
    float r_val = G * L_R - accel * H_CG;
    float f_val = G * L_F + accel * H_CG;
    float vel_ratio = cs.angular_velocity / cs.velocity;
    float first_term = FC / (cs.velocity * (L_R + L_F));


    float theta_float_dot = (FC * MASS / (I_Z * WB)) * (L_F * CS_F * \
                              cs.steer_angle * r_val + cs.slip_angle *\
                              (L_R * CS_R * f_val - L_F * CS_F * r_val) -\
                              vel_ratio * (std::pow(L_F, 2) * CS_F * r_val +\
                              std::pow(L_R, 2) * CS_R * f_val));

    float slip_angle_dot = first_term * (CS_F * cs.steer_angle * r_val -\
                            cs.slip_angle * (CS_R * f_val + CS_F * r_val) +\
                            vel_ratio * (CS_R * L_R * f_val - CS_F * L_F *\
                            r_val)) - cs.angular_velocity;

    // update current state
    cs.x += x_dot * dt;
    cs.y += y_dot * dt;
    cs.theta += theta_dot * dt;
    cs.velocity += v_dot * dt;
    cs.steer_angle += steer_ang_dot * dt;
    cs.theta += theta_dot * dt;
    cs.angular_velocity += theta_float_dot * dt;
    cs.slip_angle += slip_angle_dot * dt;
    cs.st_dyn = true;
}

void Car::setCarEdgeDistances(int num_rays, float min_ang, float scan_ang_inc,
                             float scan_dist_to_base)
{
    edge_distances = std::vector<float>(num_rays);
    float d1, d2;
    float dist_to_side = WIDTH / 2.0;
    float dist_to_front = WB - scan_dist_to_base;
    float dist_to_back = scan_dist_to_base;
    float cur_ang = min_ang;

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

void Car::control(float speed, float steer_vel)
{
    input_speed = speed;
    input_steer = steer_vel;
}

bool Car::isCrashed(float* rays, int num_rays)
{
    /*
     * rays is the same length as edge_distances.
     * if a ray is small enough to be within the
     * edge distances set, the car has crashed
     *
     */


    // we can speed this up by only checking the rays
    // that are within our steering range, and also if
    // an object is close we don't really have to check
    // every ray, every other would be fine.
    //
    // TODO

    for(size_t i=0; i < num_rays; ++i)
    {
        if(rays[i] - edge_distances[i] < CRASH_THRESH)
        {
            return true;
        }
    }
    return false;
}

void Car::setState(float *state)
{
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
}

void Car::getState(float *state)
{
    // write state
    state[0] = cs.x;
    state[1] = cs.y;
    state[2] = cs.theta;
    state[3] = cs.velocity;
    state[4] = cs.steer_angle;
    state[5] = cs.angular_velocity;
    state[6] = cs.slip_angle;
    state[7] = static_cast<float>(cs.st_dyn);
}

void Car::getScanPose(float scan_dist_to_base, float* pose)
{
    pose[0] = cs.x + scan_dist_to_base * std::cos(cs.theta);
    pose[1] = cs.y + scan_dist_to_base * std::sin(cs.theta);
    pose[2] = cs.theta;
}

/* ************************************************************************** */
/* *********************************</class Car>***************************** */
/* ************************************************************************** */
