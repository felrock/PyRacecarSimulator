#! /usr/bin/python2

from car_config import CarState, CarParams
import math

# Implementation based off of Single Track Dynamics defined in CommonRoad: Vehicle Models
# https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

def ST_update(start, accel, steer_ang_vel, p, dt):
    """
        start - CarState of starting state
        accel - Acceleration
        steer_ang_vel - Steering Angle velocity
        p - CarParams
        dt - time step
    """

    thresh = 0.5 # cut off to avoid singular behavior
    err   = 0.03 # deadband to avoid flip flop
    g = 9.81 # gravity

    if not start.st_dyn:
        thresh += err

    # if velocity is low or negative, use normal kinematics
    if start.velocity < thresh:

        return ST_update_K(start,
                           accel,
                           steer_ang_vel,
                           p,
                           dt)

    # first derivatives
    x_dot = start.velocity * math.cos(start.theta + start.slip_angle)
    y_dot = start.velocity * math.sin(start.theta + start.slip_angle)
    v_dot = accel
    steer_angle_dot = steer_ang_vel # steer angle dot
    theta_dot = start.angular_velocity # theta dot

    rear_val = (g * p.l_r) - (accel * p.h_cg)
    front_val = (g * p.l_f) + (accel * p.h_cg)

    if start.velocity == 0.0:
        vel_ratio = 0.0
        first_term = 0.0
    else:
        vel_ratio = start.angular_velocity / start.velocity
        first_term = p.f_c / (start.velocity * (p.l_r + p.l_f))

    # use extra paranth for multi line
    theta_double_dot = ((p.f_c * p.mass / (p.I_z * p.wb)) *
                       (p.l_f * p.cs_f * start.steer_angle * (rear_val) +
                       start.slip_angle * (p.l_r * p.cs_r * (front_val) -
                       p.l_f * p.cs_f * (rear_val)) - vel_ratio *
                       (math.pow(p.l_f, 2) * p.cs_f * (rear_val) +
                       math.pow(p.l_r, 2) * p.cs_r * (front_val))))
    #
    slip_angle_dot  = ((first_term) * (p.cs_f * start.steer_angle * (rear_val) -
                      start.slip_angle * (p.cs_r * (front_val) + p.cs_f *
                      (rear_val) + vel_ratio * (p.cs_r * p.l_r * (front_val) -
                      p.cs_f * p.l_r * (rear_val)))) - start.angular_velocity)

    # return computed state
    return CarState({
            'x': start.x + x_dot * dt,
            'y': start.y + y_dot * dt,
            'theta': start.theta + theta_dot * dt,
            'velocity': start.velocity + v_dot * dt,
            'steer_angle': start.steer_angle + steer_angle_dot *dt,
            'angular_velocity': start.angular_velocity + theta_double_dot * dt,
            'slip_angle': start.slip_angle + slip_angle_dot * dt,
            'st_dyn': True
    })


def ST_update_K(start, accel, steer_ang_vel, p, dt):
    """
        start - CarState of starting state
        accel - Acceleration
        steer_ang_vel - Steering Angle velocity
        p - CarParams
        dt - time step
    """

    x_dot = start.velocity * math.cos(start.theta)
    y_dot = start.velocity * math.sin(start.theta)
    v_dot = accel
    steer_angle_dot = steer_ang_vel # steer angle dot
    theta_dot = start.velocity / p.wb * math.tan(start.steer_angle)

    theta_double_dot = (accel / p.wb * math.tan(start.steer_angle) +
                       start.velocity * steer_ang_vel / (p.wb *
                       math.pow(math.cos(start.steer_angle), 2)))

    slip_angle_dot = 0

    # return computed state
    return CarState({
            'x': start.x + x_dot * dt,
            'y': start.y + y_dot * dt,
            'theta': start.theta + theta_dot * dt,
            'velocity': start.velocity + v_dot * dt,
            'steer_angle': start.steer_angle + steer_angle_dot *dt,
            'angular_velocity': 0, #start.angular_velocity + theta_double_dot * dt,
            'slip_angle': 0, #start.slip_angle + slip_angle_dot * dt,
            'st_dyn': False
    })


# Implementation based off of Kinematic Single Track Dynamics defined in CommonRoad: Vehicle Models
# https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

def KS_update(start, accel, steer_ang_vel, p, dt):
    """
        start - CarState of starting state
        accel - Acceleration
        steer_ang_vel - Steering Angle velocity
        p - CarParams
        dt - time step
    """

    x_dot = start.velocity * math.cos(start.theta)
    y_dot = start.velocity * math.sin(start.theta)
    v_dot = accel
    steer_angle_dot = steer_ang_vel
    theta_dot = start.velocity / p.wb * math.tan(start.steer_angle)

    return CarState({
            'x': start.x + x_dot * dt,
            'y': start.y + y_dot * dt,
            'theta': start.theta + theta_dot * dt,
            'velocity': start.velocity + v_dot * dt,
            'steer_angle': start.steer_angle + steer_angle_dot *dt,
            'angular_velocity': start.angular_velocity,
            'slip_angle': start.slip_angle,
            'st_dyn': False
    })


if __name__ == '__main__':
    pass
