import * from car_config
import numpy as np
import math
from scan_simulator import ScanSimulator2D
import precompute
import kinematics

class RacecarSimulator():

    def __init__(self, raw_map):
        self.map_frame = "map"
        self.base_frame = "base_link"
        self.scan_frame = "laser"

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
        self.scan_distance_to_base_link = 0.275
        self.max_speed = 4.0
        self.max_steering_angle = 0.4189
        self.max_accel = 3.0
        self.max_steering_vel = 5.0
        self.max_decel = 20.0
        self.desired_speed = 0.0
        self.desired_steer_ang = 0.0
        self.accel = 0.0
        self.steer_angle_vel = 0.0

        self.params = CarParams({
            "wheelbase": 0.3302,
            "friction_coeff": 1.0,
            "height_cg": 0.08255,
            "l_cg2front": 0.15875,
            "l_cg2rear": 0.17145,
            "C_S_front": 2.3,
            "C_S_rear": 2.3,
            "mass": 3.17,
            "moment_inertia": .0398378
        })
        self.width = 0.2032

        self.update_pose_rate = 0.001
        self.scan_beams = 1081
        self.scan_fov =  4.71 #6.2831853 # radians
        self.scan_std_dev = 0.01
        self.scan_simulator = ScanSimulator2D(
                                scan_beams,
                                scan_fov,
                                scan_std_dev)
        self.map_free_threshold = 0.8


        #Safety margins for collision
        self.TTC = False
        self.ttc_threshold = 0.01
        #precompute cosines of scan angles
        self.PI = 3.14159
        self.scan_ang_incr = self.scan_fov/(self.scan_beams-1)
        self.cosines = precompute.get_cosines(
                            self.scan_beams,
                            -self.scan_fov/2.0,
                            self.scan_ang_incr)
        self.car_distances = precompute.get_car_distances(
                                self.scan_beams, self.params.wb, self.width,
                                self.scan_distance_to_base_link,
                                -self.scan_fov/2.0, self.scan_ang_incr)

        self.original_map = raw_map.map
        self.current_map = raw_map.map

        #For obstacle collision
        self.map_width = raw_map.width
        self.map_height = raw_map.height
        self.map_resolution = raw_map.resolution
        self.origin_x = raw_map.origin_x
        self.origin_y = raw_map.origin_y

        print "Simulator constructed"


        def update_pose(self):
            self.compute_accel(self.desired_speed)
            self.compute_steer_vel(self.desired_steer_ang)

            #update pose
            self.state = kinematics.ST_update(
                            self.state,
                            self.accel,
                            self.steer_ang_vel,
                            self.params,
                            0.001)# current-prev
            self.state.velocity = set_bounded(self.state.velocity, self.max_speed)
            self.state.steer_angle = set_bounded(self.state.steer_angle, self.max_steering_angle)

            x = self.state.x + self.scan_distance_to_base_link * math.cos(self.state.theta)
            y = self.state.y + self.scan_distance_to_base_link * math.sin(self.state.theta)

            scan = self.scan_simulator.scan(x, y, self.state.theta)

            no_collision = True

            if(self.state.velocity != 0):
                for( x in range(len(scan))):
                    proj_velocity = self.state.velocity * self.cosines[x]
                    ttc = (scan[x] - self.car_distances[x]) / proj_velocity
                if((ttc < self.ttc_threshold) and (ttc >= 0.0)):
                    if not(ttc):
                        self.first_ttc_actions()
                    no_collision = False
                    self.TTC = True

                    print("Collision detected")

            if(no_collision):
                self.TTC = False


        #SET FUNCTION
        @static
        def set_bounded(value, max_value):
            return min(max(value, -max_value), max_value)

        #HELPER FUNCTIONS
        def compute_steer_vel(self, desired_angle):
            dif = (desired_angle - self.state.steer_angle)

            if(abs(dif) > 0.0001):
                self.steer_angle_vel = set_bounded((dif / abs(dif) * self.max_steering_vel), self.max_steering_vel)
            else:
                self.steer_angle_vel = set_bounded(0, self.max_steering_vel)

        def compute_accel(self, desired_velocity):
            dif = desired_velocity - self.state.velocity

            if(self.state.velocity > 0):
                if(dif > 0):
                    kp = 2.0 * self.max_accel / self.max_speed
                    self.accel = set_bounded(kp*dif, self.max_accel)
                else:
                    accel = -self.max_decel #brake
            else:
                if (dif > 0):
                    accel = -self.max_decel #brake
                else:
                    kp = 2.0 * self.max_accel / self.max_speed
                    self.accel = set_bounded(kp*dif, self.max_accel)

        def first_ttc_actions(self):
            #completely stop vehicle
            self.state.velocity = 0.0
            self.state.angular_velocity = 0.0
            self.state.slip_angle = 0.0
            self.state.steer_angle = 0.0
            self.steer_angle_vel = 0.0
            self.accel = 0.0
            self.desired_speed = 0.0
            self.desired_steer_ang = 0.0
