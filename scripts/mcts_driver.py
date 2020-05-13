#! /usr/bin/python
import sys
import os
import rospkg
import rospy
import numpy as np
import range_libc
import time

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Transform
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Header, Float32MultiArray
from nav_msgs.srv import GetMap
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_simulator_v2 import RacecarSimulator
from mcts import Node, MCTS
from policy import Policy

class MCTSdriver:

    def __init__(self, verbose=True):
        self.beta = 0

        self.verbose = verbose

        # parameters for simulation
        self.update_pose_rate = rospy.get_param("~update_pose_rate")
        self.update_action_rate = rospy.get_param("~update_action_rate")

        self.drive_topic = rospy.get_param("~drive_topic")
        self.map_topic = rospy.get_param("~map_topic")
        self.scan_topic = rospy.get_param("~scan_topic")
        self.odom_topic = rospy.get_param("~odom_topic")

        # parameters for car/s
        self.car_config = {
            "scan_beams": rospy.get_param("~scan_beams"),
            "scan_fov": rospy.get_param("~scan_fov"),
            "scan_std": rospy.get_param("~scan_std"),
            "free_thresh": rospy.get_param("~free_thresh"),
            "scan_dist_to_base": rospy.get_param("~scan_dist_to_base"),
            "max_speed": rospy.get_param("~max_speed"),
            "max_accel": rospy.get_param("~max_accel"),
            "max_decel": rospy.get_param("~max_decel"),
            "max_steer_ang": rospy.get_param("~max_steer_ang"),
            "max_steer_vel": rospy.get_param("~max_steer_vel"),
            #"col_thresh": rospy.get_param("~coll_threshold"),
            "ttc_thresh": rospy.get_param("~ttc_thresh"),
            "width": rospy.get_param("~width"),
            "length": rospy.get_param("~length"),
            "scan_max_range": rospy.get_param("~scan_max_range"),
            "update_pose_rate": self.update_pose_rate,
            "wb": rospy.get_param("~wheelbase"),
            "fc": rospy.get_param("~friction_coeff"),
            "h_cg": rospy.get_param("~height_cg"),
            "l_r": rospy.get_param("~l_cg2rear"),
            "l_f": rospy.get_param("~l_cg2front"),
            "cs_f": rospy.get_param("~C_S_front"),
            "cs_r": rospy.get_param("~C_S_rear"),
            "I_z": rospy.get_param("~moment_inertia"),
            "mass": rospy.get_param("~mass"),
            "batch_size": rospy.get_param("~batch_size")
        }

        # create policy session
        pack = rospkg.RosPack()
        self.graph_path = (pack.get_path('PyRacecarSimulator') +
                rospy.get_param("~graph_path"))
        self.ps = Policy(self.graph_path)

        # racecar object
        self.rcs = RacecarSimulator(self.car_config)

        # read and create OMap
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        new_map = []
        for i in xrange(len(map_msg.data)):
            if map_msg.data[i] > 0:
                new_map.append(255)
            else:
                new_map.append(0)
        map_msg.data = tuple(new_map)
        self.mapCallback(map_msg)

        # set ray tracing method
        self.rcs.setRaytracingMethod(rospy.get_param("~scan_method"))

        # run first scan
        self.rcs.runScan()

        # subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        self.lidar_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.lidarCallback)
        self.action_update = rospy.Timer(rospy.Duration(self.update_action_rate), self.createActionCallback)

        # publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)

        # to keep track of position
        self.odom_state = self.rcs.getState()
        self.scan = None
        self.action = 0.0

        if self.verbose:
            print "Driver constructed"

    def writeTreePoints(self, file, tree_node):

        if not tree_node.hasChildren():
            file.write("0, %f, %f\n" % (tree_node.state[0],
                                  tree_node.state[1]))

            for ro in tree_node.rollout_points:
                file.write("1, %f, %f\n" % (ro[0],
                                      ro[1]))

        else:
            for child in tree_node.children:
                self.writeTreePoints(file, child)
            file.write("0, %f, %f\n" % (tree_node.state[0],
                                  tree_node.state[1]))

            for ro in tree_node.rollout_points:
                file.write("0, %f, %f\n" % (ro[0],
                                      ro[1]))

    def createActionCallback(self, event):
        """
            Updates simulatio one step
        """
        speed = 3.0
        # create timestamp


        # create new MCTS instance
        self.rcs.setState(self.odom_state)
        mcts_run = MCTS(self.rcs, self.ps, self.action,
                                    self.car_config['batch_size'], budget=1.0)
        self.action = mcts_run.mcts()

        # logg the tree
        """
        if self.beta ==  5:
            print os.path.abspath(os.getcwd())

            with open('logg_mcts_100_RND_RO.txt', 'w') as f:
                self.writeTreePoints(f, mcts_run.root)
        self.beta += 1

        """
        # update rcs
        self.rcs.drive(speed, self.action)
        self.rcs.updatePose()

        # ackerman cmd stuf
        timestamp = rospy.get_rostime()
        self.action = np.clip(self.action, -0.4189, 0.4189)
        print("Action: ", self.action)
        print("============================NEW ITERATION=============================")

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.action
        drive_msg.drive.speed = speed

        self.drive_pub.publish(drive_msg)


        #if self.rcs.isCrashed(self.scan, self.num_rays):
            #self.rcs.stop()


    def mapCallback(self, map_msg):
        """
            Map is changed, pass new map 2d-scanner,

            fix this for dynamic map updates, origin is not important
            if its changes to the original map
        """

        ros_omap = range_libc.PyOMap(map_msg)

        quat = np.array((map_msg.info.origin.orientation.x,
                         map_msg.info.origin.orientation.y,
                         map_msg.info.origin.orientation.z,
                         map_msg.info.origin.orientation.w))
        (_,_,yaw) = euler_from_quaternion(quat)

        origin = (map_msg.info.origin.position.x,
                  map_msg.info.origin.position.y,
                  yaw)

        # pass map info to racecar instance
        self.rcs.setMap(ros_omap, map_msg.info.resolution, origin)

    def odomCallback(self, odom_msg):
        #self.odom_state = self.rcs.getState()
        quat = euler_from_quaternion((odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z,
                                     odom_msg.pose.pose.orientation.w))
        self.odom_state[0] = odom_msg.pose.pose.position.x
        self.odom_state[1] = odom_msg.pose.pose.position.y
        self.odom_state[2] = quat[2]
        self.odom_state[3] = odom_msg.twist.twist.linear.x
        self.odom_state[4] = odom_msg.twist.twist.linear.z

    def lidarCallback(self, lidar_msg):

        self.scan = lidar_msg.ranges

def run():
    """
        Main func
    """

    rospy.init_node('mcts_driver', anonymous=True)
    MCTSdriver(verbose=False)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()

