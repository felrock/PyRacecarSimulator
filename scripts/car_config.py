#! /usr/bin/python2

class CarParams:
    """
        This class is a container for parameters
    """
    def __init__(self, d):
        self.wb = d['wheelbase']
        self.f_c = d['friction_coeff']
        self.h_cg = d['height_cg']
        self.l_f = d['l_cg2front']
        self.l_r = d['l_cg2rear']
        self.cs_f = d['C_S_front']
        self.cs_r = d['C_S_rear']
        self.mass = d['mass']
        self.I_z = d['moment_inertia']

class CarState(object):
    """
        This class is a container for simulation states
    """
    def __init__(self, d):
        self.x = d['x']
        self.y = d['y']
        self.theta = d['theta']
        self.velocity = d['velocity']
        self.steer_angle = d['steer_angle']
        self.angular_velocity = d['angular_velocity']
        self.slip_angle = d['slip_angle']
        self.st_dyn = d['st_dyn']


if __name__ == '__name__':
    pass
