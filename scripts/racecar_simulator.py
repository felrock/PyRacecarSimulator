from car_config import CarState, CarParams
import numpy as np
import math
import copy
from scan_simulator import ScanSimulator2D
import precompute
import kinematics
import time

class RacecarSimulator():

    def __init__(self, config, params, verbose=False):

        self.verbose = verbose

        self.map_frame = "map"
        self.base_frame = "base_link"
        self.scan_frame = "laser"

        self.config = config
        self.params = params

        # Init car state
        self.state = CarState({
            "x": 0,
            "y": 0, "theta": 0,
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
        self.scan_max_range = config["scan_max_range"]

        self.free_thresh = config["free_thresh"]
        self.ttc_thresh = config["ttc_thresh"]

        self.params = params

        self.scan_simulator = ScanSimulator2D(
                                    self.scan_beams,
                                    self.scan_fov,
                                    self.scan_std)

        #Safety margins for collision, time to collision
        self.TTC = False

        #precompute cosines of scan angles
        self.scan_ang_incr = self.scan_fov/(self.scan_beams-1)
        self.cosines = precompute.get_cosines(
                            self.scan_beams,
                            -self.scan_fov/2.0,
                            self.scan_ang_incr)

        self.car_distances = precompute.get_car_distances(
                                self.scan_beams, self.params.wb, self.width,
                                self.scan_dist_to_base,
                                -self.scan_fov/2.0, self.scan_ang_incr)

        self.scan = None

        self.scan_time = 0
        self.scan_time_cnt = 0

        self.coll_time = 0
        self.coll_time_cnt = 0

        self.update_time = 0
        self.update_time_cnt = 0

        if self.verbose:
            print "Simulator constructed"

    def setState(self, state):
        """
            Set state to any previous state
        """

        self.state = state

    def getState(self):
        """
            Maybe use this w/o deep copy
        """

        return self.state

    def getScan(self):
        """
            get the 2d lidar scan of the most resent update
        """

        return self.scan

    def runScan(self):
        """
            Run a scan
        """

        x = self.state.x + self.scan_dist_to_base * math.cos(self.state.theta)
        y = self.state.y + self.scan_dist_to_base * math.sin(self.state.theta)
        theta = self.state.theta

        self.scan = self.scan_simulator.scan(x,y,theta)

    def drive(self, desired_speed, desired_steer_ang):
        """
            Update desired states
        """

        self.desired_speed = desired_speed
        self.desired_steer_ang = desired_steer_ang

    def updatePose(self, dt=None):
        """
            Make one step in the simulation
        """
        t1 = time.time()

        self.accel = self.compute_accel(self.desired_speed)
        self.steer_ang_vel = self.compute_steer_vel(self.desired_steer_ang)

        #update pose
        self.state = kinematics.ST_update(
                        self.state,
                        self.accel,
                        self.steer_ang_vel,
                        self.params,
                        self.config["update_pose_rate"]) # current-prev

        # limit steer and speed
        self.state.velocity = self.set_bounded(self.state.velocity, self.max_speed)
        self.state.steer_angle = self.set_bounded(self.state.steer_angle, self.max_steer_ang)
        t2 = time.time()
        self.update_time += (t2-t1)
        self.update_time_cnt += 1

        print "Avg Time in updatePose(): %f" % (self.update_time/self.update_time_cnt)

        # this could be moved out of here
        self.checkCollision()

    def checkCollision(self):
        """
            This takes some computation to perform.

            Scan coulds be checked for collision in parrallel

            If any on the scans indicate a crash then run stop_car()
        """
        t1 = time.time()

        no_collision = True
        if self.state.velocity != 0:
            # 180-900 is the steering span
            for i in xrange(180, 900, 10):

                proj_velocity = self.state.velocity * self.cosines[i]
                ttc = (self.scan[i] - self.car_distances[i]) / proj_velocity

                if ttc < self.ttc_thresh and ttc >= 0.0:
                    if not self.TTC:
                        self.stop_car()

                    no_collision = False
                    self.TTC = True

                    if self.verbose:
                        print "Collision detected"

        if no_collision:
            self.TTC = False

        t2 = time.time()
        self.coll_time += (t2-t1)
        self.coll_time_cnt += 1
        print "Time spend in checkCollision(): %f" %(self.coll_time/self.coll_time_cnt)

    def set_bounded(self, value, max_value):
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
        # change direction for steering velo
        if abs(dif) > 0.0001:
            if dif > 0:
                return self.max_steer_vel
            else:
                return -self.max_steer_vel
        else:
            return 0

    def compute_accel(self, desired_velocity):
        """
            Accelerate to desired_velocity
        """

        dif = desired_velocity - self.state.velocity

        if self.state.velocity > 0.0:
            if dif > 0.0:
                kp = 2.0 * self.max_accel / self.max_speed
                return self.set_bounded(kp*dif, self.max_accel)

            else:
                return -self.max_decel #brake

        else:
            if dif > 0.0:
                return self.max_decel # MAX SPEED

            else:
                kp = 2.0 * self.max_accel / self.max_speed
                return self.set_bounded(kp*dif, self.max_accel)

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

    def setMap(self, ros_map, resolution, origin):
        """
            Pass map along to Scan simulator
        """

        # map, max range in pixels, origin
        max_range_px = int(self.scan_max_range/resolution)
        self.scan_simulator.setMap(ros_map, max_range_px, resolution, origin)

    def setRaytracingMethod(self, method="CDDT"):
        """
            Pass ray-tracing method to Scan simulator
        """

        self.scan_simulator.setRaytracingMethod(method)

if __name__ == '__main__':
    pass
