import numpy as np
import copy
from scan import ScanSimulator2D
import racecar

class RacecarSimulatorTwoPlayer():

    def __init__(self, config, players, verbose=False):

        self.verbose = verbose

        # default settings for all cars
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
        self.car_outline = self.num_rays
        self.scan_fov =  config["scan_fov"]
        self.scan_std = config["scan_std"]
        self.scan_max_range = config["scan_max_range"]

        self.free_thresh = config["free_thresh"]
        self.ttc_thresh = config["ttc_thresh"]

        # create car objects
        self.cars = {}
        for name, pos in players:

            car = racecar.PyCar(config['wb'], config['fc'],
                               config['h_cg'], config['l_f'], config['l_r'],
                               config['cs_f'], config['cs_r'], config['mass'],
                               config['I_z'], config['ttc_thresh'],
                               config['width'], config['length'],
                               config['max_steer_vel'], config['max_steer_ang'],
                               config['max_speed'], config['max_accel'],
                               config['max_decel'])

            # set where lidar cuts in the car
            car.setCarEdgeDistances(self.num_rays,
                                    -self.scan_fov/2.0,
                                    self.scan_fov/self.num_rays,
                                    self.scan_dist_to_base)
            # set origin position
            state = np.zeros(11, dtype=np.float64)
            state[0] = pos[0]
            state[1] = pos[1]
            car.setState(state)

            # store players car, desired_speed, desired_angle
            self.cars[name] = [car, 0.0, 0.0]


        # create lidar simulator, used for all players
        self.scan_simulator = ScanSimulator2D(
                                    self.num_rays,
                                    self.scan_fov,
                                    self.scan_std,
                                    self.batch_size)

        # output scan vector
        self.scan = np.zeros(self.num_rays)

        if self.verbose:
            print "Simulator constructed"

    def setState(self, state, player):
        """
            Set state to any previous state
        """

        self.cars[player][0].setState(state)

    def getState(self, player):
        """
            Maybe use this w/o deep copy
        """

        state = np.zeros(11, dtype=np.float64)
        self.cars[player][0].getState(state)

        return state

    def getScan(self):
        """
            get the 2d lidar scan of the most resent update
        """

        return self.scan

    def runScan(self, player):
        """
            Run a scan, with player in the map
        """

        pose = np.zeros(3, dtype=np.float32)
        bound = np.zeros(self.car_outline, dtype=np.float32)
        self.cars[player][0].getScanPose(self.scan_dist_to_base, pose)
        self.cars[player][0].getBound(self.car_outline, bound)

        # add car outline to original map
        self.ego_map[:] = self.org_map
        for i in xrange(0, self.car_outline-10, 2):
            x, y = self.floatPosToPix(bound[i], bound[i+1])
            if x*self.map_width + y < self.map_width*self.map_height:
                self.ego_map[x*self.map_width + y] = 255
            else:
                print i, x, y

        self.map_msg.data = tuple(self.ego_map)

        # create new scan object
        self.scan_simulator.build(self.map_msg, self.mrx, 112)

        # scan
        self.scan = self.scan_simulator.scan(*pose)

        return self.scan

    def drive(self, player, desired_speed, desired_steer_ang):
        """
            Update desired states
        """

        self.cars[player][1] = desired_speed
        self.cars[player][2] = desired_steer_ang

    def updatePose(self, player, dt=0.01):
        """
            Make one step in the simulation
        """

        self.cars[player][0].control(self.cars[player][1],self.cars[player][2])
        self.cars[player][0].updatePosition(dt)

    def checkCollision(self):
        """
            This takes some computation to perform.

            Scan coulds be checked for collision in parrallel

            If any on the scans indicate a crash then run stop_car()
        """

        scan_ = np.array(self.scan, dtype=np.float32)
        return self.cars[player][0].isCrashed(scan_, self.num_rays, 1)

    def stop(self, player):
        """
            Halt the car
        """

        state = self.getState(player)
        state[0] = 0.0
        state[1] = 0.0
        state[2] = 0.0
        state[3] = 0.0
        state[4] = 0.0
        state[5] = 0.0
        state[6] = 0.0
        state[7] = 0.0
        self.setState(player, state)
        self.cars[player][1] = 0.0
        self.cars[player][2] = 0.0

    def setMap(self, map_msg):
        """
            Pass map along to Scan simulator
        """

        # map, max range in pixels, origin
        self.org_map = np.array(map_msg.data)
        self.ego_map = np.array(map_msg.data)
        self.map_msg = map_msg
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.resolution = map_msg.info.resolution

        # save origins for adding and removing later
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y

        self.mrx = int(self.scan_max_range/self.resolution)

    def floatPosToPix(self, x, y):
        x0 = int(((x + self.origin_x)/self.resolution))
        y0 = int(((y + self.origin_y)/self.resolution))
        print x0+self.map_width, y0+self.map_height
        return x0+self.map_width, y0+self.map_height

if __name__ == '__main__':
    pass
