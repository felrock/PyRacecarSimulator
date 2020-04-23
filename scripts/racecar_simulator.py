import * from car_config
import numpy as np
import math
import copy
from scan_simulator import ScanSimulator2D
import precompute
import kinematics

class RacecarSimulator():

    def __init__(self, config, params, ros_map, verbose=False):

        self.verbose = verbose

        self.map_frame = "map"
        self.base_frame = "base_link"
        self.scan_frame = "laser"

        # Init car state
        self.state = CarState({
            "x": 0,
            "y": 0,
            "theta": 0,
            "velocity": 0,
            "steer_angle": 0,
            "angular_velocity": 0,
            "slip_angle": 0,
            "st_dyn": False
        })
        self.desired_speed = 0.0
        self.desired_steer_ang = 0.0
        self.accel = 0.0
        self.steer_ang_vel = 0.0

        self.scan_dist_to_base = config["scan_dist_to_base"]
        self.max_speed = config["max_speed"]
        self.max_accel = config["max_accel"]
        self.max_steer_ang = config["max_steer_ang"]
        self.max_steer_vel = config["max_steer_vel"]
        self.max_decel = config["max_decel"]

        self.width = config["width"]

        self.scan_beams = config["scan_beams"]
        self.scan_fov =  config["scan_fov"] #4.71 #6.2831853 # radians
        self.scan_std = config["scan_std"]

        self.free_thresh = config["free_thresh"]
        self.ttc_thresh = config["ttc_thresh"]

        self.params = params

        self.scan_simulator = ScanSimulator2D(
                                scan_beams,
                                scan_fov,
                                scan_std_dev)


        #Safety margins for collision
        self.TTC = False

        #precompute cosines of scan angles
        self.scan_ang_incr = self.scan_fov/(self.scan_beams-1)
        self.cosines = precompute.get_cosines(
                            self.scan_beams,
                            -self.scan_fov/2.0,
                            self.scan_ang_incr)

        self.car_distances = precompute.get_car_distances(
                                self.scan_beams, self.params.wb, self.width,
                                self.scan_distance_to_base_link,
                                -self.scan_fov/2.0, self.scan_ang_incr)

        #For obstacle collision
        self.map_width = ros_map.width
        self.map_height = ros_map.height
        self.map_resolution = ros_map.resolution
        self.origin_x = ros_map.origin_x
        self.origin_y = ros_map.origin_y

        if self.verbose:
            print "Simulator constructed"

    def setState(state):
        """
            Set state to any previous state
        """

        self.state = state

    def getState():
        """
            Maybe use this w/o deep copy
        """

        return copy.deepcopy(state)

    def drive(self, desired_speed, desired_steer_ang):
        """
            Update desired states
        """

        self.desired_speed = desired_speed
        self.desired_steer_ang = desired_steer_ang

    def update_pose(self, dt=0.001):
        """
            Make one step in the simulation
        """

        self.accel = self.compute_accel(self.desired_speed)
        self.steer_ang_vel = self.compute_steer_vel(self.desired_steer_ang)

        #update pose
        self.state = kinematics.ST_update(
                        self.state,
                        self.accel,
                        self.steer_ang_vel,
                        self.params,
                        dt) # current-prev
        self.state.velocity = set_bounded(self.state.velocity, self.max_speed)
        self.state.steer_angle = set_bounded(self.state.steer_angle, self.max_steer_ang)

        x = self.state.x + self.scan_distance_to_base_link * math.cos(self.state.theta)
        y = self.state.y + self.scan_distance_to_base_link * math.sin(self.state.theta)

        scan = self.scan_simulator.scan(x, y, self.state.theta)

        no_collision = True
        if self.state.velocity != 0:
            for i in xrange(len(scan)):
                proj_velocity = self.state.velocity * self.cosines[i]
                ttc = (scan[i] - self.car_distances[i]) / proj_velocity

                if ttc < self.ttc_threshold and ttc >= 0.0:
                    if not ttc:
                        self.stop_car()

                    no_collision = False
                    self.TTC = True

                    print "Collision detected"

        if no_collision:
            self.TTC = False


    @static
    def set_bounded(value, max_value):
        """
            Will bound value between (-max_value, max_value)
        """

        return min(max(value, -max_value), max_value)

    def compute_steer_vel(self, desired_angle):
        """
            Steer towards desired_angle
        """

        dif = desired_angle - self.state.steer_angle
        # bounded value if it is a significant difference
        if abs(dif) > 0.0001:
            return set_bounded(dif / abs(dif) * self.max_steer_vel,
                               self.max_steer_vel)

        else:
            return set_bounded(0, self.max_steer_vel)

    def compute_accel(self, desired_velocity):
        """
            Accelerate to desired_velocity
        """

        dif = desired_velocity - self.state.velocity

        if self.state.velocity > 0:
            if dif > 0:
                kp = 2.0 * self.max_accel / self.max_speed
                return set_bounded(kp*dif, self.max_accel)

            else:
                return = -self.max_decel #brake

        else:
            if dif > 0:
                return -self.max_decel #brake

            else:
                kp = 2.0 * self.max_accel / self.max_speed
                return set_bounded(kp*dif, self.max_accel)

    def stop_car(self):
        """
            Completely stop vehicle
        """

        self.state.velocity = 0.0
        self.state.angular_velocity = 0.0
        self.state.slip_angle = 0.0
        self.state.steer_angle = 0.0
        self.steer_ang_vel = 0.0
        self.accel = 0.0
        self.desired_speed = 0.0
        self.desired_steer_ang = 0.0
