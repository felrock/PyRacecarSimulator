#! /usr/bin/python
import sys
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
from MCTS import Node, MCTS

class MCTSdriver:

    def __init__(self, verbose=True):

        # parameters for simulation
        self.buffer_length = rospy.get_param("~buffer_length")
        self.map_frame = rospy.get_param("~map_frame")
        self.base_frame = rospy.get_param("~base_frame")
        self.scan_frame = rospy.get_param("~scan_frame")
        self.update_pose_rate = rospy.get_param("~update_pose_rate")

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
            "mass": rospy.get_param("~mass")
        }

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

        # subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback, queue_size=10)

        # publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)

        if self.verbose:
            print "Driver constructed"

    def updateSimulationCallback(self, event):
        """
            Updates simulatio one step
        """

        # create timestamp
        timestamp = rospy.get_rostime()

        # update simulation
        t1 = time.time()
        mcts_run = MCTS(self.rcs, 1.0)
        action = mcts_run.mcts()
        #self.rcs.updatePose()
        print " time to update pose: %f " %(time.time()-t1)

        # pub pose as transform
        self.poseTransformPub(timestamp)

        # publish steering ang
        self.steerAngTransformPub(timestamp)

        # publish odom
        self.odomPub(timestamp)

        # publish imu
        # todo

        # sim lidar
        t1 = time.time()
        self.rcs.runScan()
        print " time to update scan: %f " %(time.time()-t1)

        if self.rcs.isCrashed(self.scan, self.num_rays):
            self.rcs.stop()

        # publish lidar
        self.lidarPub(timestamp)


        # publish the transform
        self.laserLinkTransformPub(timestamp)


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


def run():
    """
        Main func
    """

    rospy.init_node('MCTSdriver', anonymous=True)
    RunSimulationViz(verbose=False, visualize=False)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()

