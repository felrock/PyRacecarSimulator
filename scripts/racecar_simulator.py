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

        this.state = CarState({
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

        this.params = CarParams({
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
        this.width = 0.2032

        this.update_pose_rate = 0.001
        this.scan_beams = 1081
        this.scan_fov =  4.71 #6.2831853 # radians
        this.scan_std_dev = 0.01
        this.scan_simulator = ScanSimulator2D(
                                scan_beams,
                                scan_fov,
                                scan_std_dev)
        this.map_free_threshold = 0.8


        #Safety margins for collision
        this.TTC = False
        this.ttc_threshold = 0.01
        #precompute cosines of scan angles
        this.PI = 3.14159
        this.scan_ang_incr = this.scan_fov/(this.scan_beams-1)
        this.cosines = precompute.get_cosines(
                            this.scan_beams,
                            -this.scan_fov/2.0,
                            this.scan_ang_incr)
        this.car_distances = precompute.get_car_distances(
                                this.scan_beams, this.params.wb, this.width,
                                this.scan_distance_to_base_link,
                                -this.scan_fov/2.0, this.scan_ang_incr)

        this.original_map = raw_map.map
        this.current_map = raw_map.map

        #For obstacle collision
        this.map_width = raw_map.width
        this.map_height = raw_map.height
        this.map_resolution = raw_map.resolution
        this.origin_x = raw_map.origin_x
        this.origin_y = raw_map.origin_y

        print("Simulator constructed")


        def update_pose(this):
            this.compute_accel(this.desired_speed)
            this.compute_steer_vel(this.desired_steer_ang)

            #update pose
            this.state = kinematics.ST_update(
                            this.state,
                            this.accel,
                            this.steer_ang_vel,
                            this.params,
                            0.001)# current-prev
            this.state.velocity = set_bounded(this.state.velocity, this.max_speed)
            this.state.steer_angle = set_bounded(this.state.steer_angle, this.max_steering_angle)

            x = this.state.x + this.scan_distance_to_base_link * math.cos(this.state.theta)
            y = this.state.y + this.scan_distance_to_base_link * math.sin(this.state.theta)

            scan = this.scan_simulator.scan(x, y, this.state.theta)

            no_collision = True

            if(this.state.velocity != 0):
                for( x in range(len(scan))):
                    proj_velocity = this.state.velocity * this.cosines[x]
                    ttc = (scan[x] - this.car_distances[x]) / proj_velocity
                if((ttc < this.ttc_threshold) and (ttc >= 0.0)):
                    if not(ttc):
                        this.first_ttc_actions()
                    no_collision = False
                    this.TTC = True

                    print("Collision detected")
                    
            if(no_collision):
                this.TTC = False


        #SET FUNCTION
        @static
        def set_bounded(value, max_value):
            return min(max(value, -max_value), max_value)

        #HELPER FUNCTIONS
        def compute_steer_vel(self, desired_angle):
            dif = (desired_angle - this.state.steer_angle)

            if(abs(dif) > 0.0001):
                this.steer_angle_vel = set_bounded((dif / abs(dif) * this.max_steering_vel), this.max_steering_vel)
            else:
                this.steer_angle_vel = set_bounded(0, this.max_steering_vel)

        def compute_accel(self, desired_velocity):
            dif = desired_velocity - this.state.velocity

            if(this.state.velocity > 0):
                if(dif > 0):
                    kp = 2.0 * this.max_accel / this.max_speed
                    this.accel = set_bounded(kp*dif, this.max_accel)
                else:
                    accel = -this.max_decel #brake
            else:
                if (dif > 0):
                    accel = -this.max_decel #brake
                else:
                    kp = 2.0 * this.max_accel / this.max_speed
                    this.accel = set_bounded(kp*dif, this.max_accel)

        def first_ttc_actions(self):
            #completely stop vehicle
            this.state.velocity = 0.0
            this.state.angular_velocity = 0.0
            this.state.slip_angle = 0.0
            this.state.steer_angle = 0.0
            this.steer_angle_vel = 0.0
            this.accel = 0.0
            this.desired_speed = 0.0
            this.desired_steer_ang = 0.0
