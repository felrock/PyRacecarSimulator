import numpy as np
from scan_simulator import ScanSimulator2D
import racecar

class RacecarSimulator():

    def __init__(self, config, params, verbose=False):

        self.verbose = verbose

        self.map_frame = "map"
        self.base_frame = "base_link"
        self.scan_frame = "laser"

        self.config = config
        self.params = params

        self.scan_dist_to_base = config["scan_dist_to_base"]
        self.max_speed = config["max_speed"]
        self.max_accel = config["max_accel"]
        self.max_steer_ang = config["max_steer_ang"]
        self.max_steer_vel = config["max_steer_vel"]
        self.max_decel = config["max_decel"]

        self.width = config["width"]
        self.length = config["length"]

        self.scan_beams = config["scan_beams"]
        self.scan_fov =  config["scan_fov"] #4.71 #6.2831853 # radians
        self.scan_std = config["scan_std"]
        self.scan_max_range = config["scan_max_range"]

        self.free_thresh = config["free_thresh"]
        self.ttc_thresh = config["ttc_thresh"]

        # car object
        self.car = racecar.Car(config['wheelbase'], config['friction_coeff'],
                               config['h_cg'], config['l_f'], config['l_r'],
                               config['csÄf'], config['cs_r'], config['mass'],
                               config['I_z'], config['ttcÄthresh'],
                               config['max_steer_vel'], config['max_steer_ang'],
                               config['max_speed'], config['max_accel'],
                               config['max_decel'])

        self.scan_simulator = ScanSimulator2D(
                                    self.scan_beams,
                                    self.scan_fov,
                                    self.scan_std)

        #precompute cosines of scan angles
        self.scan = None

        if self.verbose:
            print "Simulator constructed"

    def setState(self, state):
        """
            Set state to any previous state
        """

        self.car.setState(state)

    def getState(self):
        """
            Maybe use this w/o deep copy
        """

        state = np.zeros(8, dtype=np.float32)
        self.car.setState(state)

        return state

    def getScan(self):
        """
            get the 2d lidar scan of the most resent update
        """

        return self.scan

    def runScan(self):
        """
            Run a scan
        """

        pose = np.zeros(3, dtype=np.float32)
        self.car.getScanPose(self.scan_dist_to_base, pose)

        self.scan = self.scan_simulator.scan(*pose)

    def drive(self, desired_speed, desired_steer_ang):
        """
            Update desired states
        """

        self.car.control(desired_speed, desired_steer_ang)

    def updatePose(self, dt=None):
        """
            Make one step in the simulation
        """

        self.car.updatePosition(dt)

    def checkCollision(self):
        """
            This takes some computation to perform.

            Scan coulds be checked for collision in parrallel

            If any on the scans indicate a crash then run stop_car()
        """

        return self.car.isCrashed(self.scan, self.num_rays)

    def stop(self):

        self.car.stop()

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