import numpy as np
from scan_simulator import ScanSimulator2D
import racecar

class RacecarSimulator():

    def __init__(self, config, verbose=False):

        self.verbose = verbose

        self.map_frame = "map"
        self.base_frame = "base_link"
        self.scan_frame = "laser"

        self.config = config

        self.scan_dist_to_base = config["scan_dist_to_base"]
        self.max_speed = config["max_speed"]
        self.max_accel = config["max_accel"]
        self.max_steer_ang = config["max_steer_ang"]
        self.max_steer_vel = config["max_steer_vel"]
        self.max_decel = config["max_decel"]

        self.width = config["width"]
        self.length = config["length"]
        self.batch_size = config['batch_size']

        self.num_rays = config["scan_beams"]
        self.scan_fov =  config["scan_fov"] #4.71 #6.2831853 # radians
        self.scan_std = config["scan_std"]
        self.scan_max_range = config["scan_max_range"]

        self.free_thresh = config["free_thresh"]
        self.ttc_thresh = config["ttc_thresh"]

        # car object
        self.car = racecar.PyCar(config['wb'], config['fc'],
                               config['h_cg'], config['l_f'], config['l_r'],
                               config['cs_f'], config['cs_r'], config['mass'],
                               config['I_z'], config['ttc_thresh'],
                               config['width'], config['length'],
                               config['max_steer_vel'], config['max_steer_ang'],
                               config['max_speed'], config['max_accel'],
                               config['max_decel'])

        # set where lidar cuts in the car
        self.car.setCarEdgeDistances(self.num_rays,
                                    -self.scan_fov/2.0,
                                    self.scan_fov/self.num_rays,
                                    self.scan_dist_to_base)

        # create lidar simulator
        self.scan_simulator = ScanSimulator2D(
                                    self.num_rays,
                                    self.scan_fov,
                                    self.scan_std,
                                    self.batch_size)

        self.scan = np.zeros(self.num_rays, dtype=np.float32)

        # user input
        self.desired_speed = 0.0
        self.desired_steer_ang = 0.0


        if self.verbose:
            print "Simulator constructed"

    def setState(self, state):
        """
            Set state to any previous state
        """

        self.car.setState(state)
        #del state

    def getState(self):
        """
            Maybe use this w/o deep copy
        """

        state = np.zeros(11, dtype=np.float64)
        self.car.getState(state)

        return state

    def getMeanVelocity(self):
        """
            The mean velocity of the current state
        """

        return self.car.getMeanVelocity()

    def getTravelDistance(self):
        """
            Distance traveled, acumulate over updatePose's
        """

        return self.car.getTravelDistance()

    def getScan(self):
        """
            get the 2d lidar scan of the most resent update
        """

        return self.scan

    def runScan(self):
        """
            Run a scan
        """

        pose = np.zeros(3, dtype=np.float64)
        self.car.getScanPose(self.scan_dist_to_base, pose)

        self.scan = self.scan_simulator.scan(*pose)

    def drive(self, desired_speed, desired_steer_ang):
        """
            Update desired states
        """

        self.desired_speed = desired_speed
        self.desired_steer_ang = desired_steer_ang

    def updatePose(self, dt=0.01):
        """
            Make one step in the simulation
        """

        self.car.control(self.desired_speed, self.desired_steer_ang)
        self.car.updatePosition(dt)

    def checkCollision(self):
        """
            This takes some computation to perform.

            Scan coulds be checked for collision in parrallel

            If any on the scans indicate a crash then run stop_car()
        """

        scan_ = np.array(self.scan, dtype=np.float32)
        return self.car.isCrashed(scan_, self.num_rays, 1)

    def checkCollisionMany(self, poses):
        """
            Scan and check collision for several poses.
            This is a speed up for the rollout process.
        """

        # run all scans on gpu
        output_scans = self.scan_simulator.scanMany(poses)

        """
        # check collisions in order
        for i in xrange(self.batch_size):
            # TODO:fix with poses
            scan = output_scans[i*self.num_rays:(i+1)*self.num_rays]
            if self.car.isCrashed(scan, self.num_rays):
                return i-1

        # no crash was found
        return self.batch_size-1

        """
        return self.car.isCrashed(output_scans, self.num_rays, self.batch_size)

    def stop(self):
        """
            Halt the car
        """

        state = self.getState()
        state[0] = 0.0
        state[1] = 0.0
        state[2] = 0.0
        state[3] = 0.0
        state[4] = 0.0
        state[5] = 0.0
        state[6] = 0.0
        state[7] = 0.0
        state[8] = 0.0
        state[9] = 0.0
        state[10] = 0.0
        self.setState(state)
        self.desired_speed = 0.0
        self.desired_steer_ang = 0.0

    def setMap(self, ros_map, resolution, origin):
        """
            Pass map along to Scan simulator
        """

        # map, max range in pixels, origin
        max_range_px = int(self.scan_max_range/resolution)
        self.scan_simulator.setMap(ros_map, max_range_px, resolution, origin)

    def setRaytracingMethod(self, method="RMGPU"):
        """
            Pass ray-tracing method to Scan simulator
        """

        self.scan_simulator.setRaytracingMethod(method)

if __name__ == '__main__':
    pass
