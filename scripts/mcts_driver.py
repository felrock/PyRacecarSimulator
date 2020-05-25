#! /usr/bin/python
import sys
import os
import rospkg
import rospy
import numpy as np
import range_libc
import time
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Transform, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Header, Float32MultiArray
from nav_msgs.srv import GetMap
from ackermann_msgs.msg import AckermannDriveStamped

#viz
from visualization_msgs.msg import Marker

from racecar_simulator_v2 import RacecarSimulator
from mcts import Node, MCTS
from policy import Policy


class MCTSdriver:

    def __init__(self, verbose=True, with_global=False):
        self.beta = 0
        self.fex = 1
        pack = rospkg.RosPack()
        self.verbose = verbose
        self.with_global = with_global

        #Global Planner
        if with_global:
            self.csv_path = (pack.get_path('PyRacecarSimulator') +
                                rospy.get_param("~csv_path"))
            self.global_path = self.readGlobalPath()
            self.lookahead_distance = 1.5
            self.unique_id = 0
            #transformations
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)
            #Publishers
            self.wp_viz_pub = rospy.Publisher('/waypoint_vis', Marker, queue_size=100)

        # parameters for simulation
        self.update_pose_rate = rospy.get_param("~update_pose_rate")
        self.update_action_rate = rospy.get_param("~update_action_rate")
        self.budget = rospy.get_param("~budget")

        self.drive_topic = rospy.get_param("~drive_topic") #+ "one"
        self.map_topic = rospy.get_param("~map_topic")
        self.scan_topic = rospy.get_param("~scan_topic") #+ "one"
        self.odom_topic = rospy.get_param("~odom_topic") #+ "one"
        #self.odom_topic = "/pf/pose/odom"

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
        self.graph_path = (pack.get_path('PyRacecarSimulator') +
                rospy.get_param("~graph_path"))
        self.ps = Policy(self.graph_path)

        # racecar object
        self.rcs = RacecarSimulator(self.car_config)

        # read and create OMap
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        new_map = []
        for i in xrange(len(self.map_msg.data)):
            if self.map_msg.data[i] > 0:
                new_map.append(255)
            else:
                new_map.append(0)
        self.map_msg.data = tuple(new_map)
        self.mapCallback(self.map_msg)

        # set ray tracing method
        self.rcs.setRaytracingMethod(rospy.get_param("~scan_method"))

        # run first scan
        self.rcs.runScan()

        # subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        self.lidar_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.lidarCallback)

        # publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)

        # to keep track of position
        self.odom_state = self.rcs.getState()
        self.scan = None
        self.action = 0.0

        if self.verbose:
            print "Driver constructed"

        self.action_update = rospy.Timer(rospy.Duration(self.update_action_rate), self.createActionCallback)

    def readGlobalPath(self):
        wp = []
        file = open(self.csv_path, 'rb')
        for line in file:
            point = line.split(', ')
            wp.append((float(point[0]),float(point[1])))
        if self.verbose:
            print "Global path read"
        return wp


    #Find best point to track with a lookahead, for validating position in map
    def findBestPoint(self, current_pose):

        closest_distance = self.lookahead_distance*2

        for track_point in self.global_path:
            distance = math.sqrt(
            (track_point[0] - current_pose[0])**2 +
            (track_point[1] - current_pose[1])**2)

            diff_distance = abs(self.lookahead_distance - distance)

            if diff_distance < closest_distance :
                closest_distance = diff_distance
                closest_point = track_point

        print "best distance: ", closest_distance


        return closest_point


    def findBestGlobal(self, current_pose):
        try:
            tf_map_to_laser = self.tf_buffer.lookup_transform("laser", "map", rospy.Time())
        except:
            print "Transform Error"

        best_point_index = -1
        best_point_distance = self.lookahead_distance*2

        for i in range(len(self.global_path)):
            goal_wp = PoseStamped()
            goal_wp.pose.position.x = self.global_path[i][0];
            goal_wp.pose.position.y = self.global_path[i][1];
            goal_wp.pose.position.z = 0;
            goal_wp.pose.orientation.x = 0;
            goal_wp.pose.orientation.y = 0;
            goal_wp.pose.orientation.z = 0;
            goal_wp.pose.orientation.w = 1;

            goal_wp = tf2_geometry_msgs.do_transform_pose(goal_wp, tf_map_to_laser)

            #if (goal_wp.pose.position.x == 0):
            #    continue
            distance = abs( self.lookahead_distance - math.sqrt( goal_wp.pose.position.x**2 + goal_wp.pose.position.y**2 ))

            if distance < best_point_distance:
                best_point_distance = distance
                best_point_index = i

        print "best distance: ", best_point_distance

        return self.global_path[best_point_index]


    #Callbacks
    def createActionCallback(self, event):
        """
            Updates simulatio one step
        """
        speed = 2.0

        # create new MCTS instance
        self.rcs.setState(self.odom_state)

        if self.with_global:
            if self.unique_id == 0:
                self.visualize_waypoint_data()

            closest_point = self.findBestPoint((self.odom_state[0], self.odom_state[1]))

            print "Closest point: ", closest_point
            mcts_run = MCTS(self.rcs, self.ps, self.action,
                                        self.car_config['batch_size'],
                                        budget=self.budget,
                                        track_point=closest_point,
                                        with_global=self.with_global)

        else:
            mcts_run = MCTS(self.rcs, self.ps, self.action,
                                        self.car_config['batch_size'],
                                        budget=self.budget)

        self.action = mcts_run.mcts()

        # logg the tree
        """
        if self.beta %5  == 0:
            print os.path.abspath(os.getcwd())

            with open('logg_mcts_01_FG_RO-%i' % self.fex, 'w') as f:
                self.writeTreePoints(f, mcts_run.root)
            self.fex += 1
        self.printDepth(mcts_run.root)

        self.beta += 1
        """
        # update rcs
        self.rcs.drive(speed, self.action)
        self.rcs.updatePose()

        # ackerman cmd stuf
        #timestamp = rospy.get_rostime()
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

    def mapCallback(self, map):
        """
            Map is changed, pass new map 2d-scanner,

            fix this for dynamic map updates, origin is not important
            if its changes to the original map
        """

        ros_omap = range_libc.PyOMap(map)

        quat = np.array((map.info.origin.orientation.x,
                         map.info.origin.orientation.y,
                         map.info.origin.orientation.z,
                         map.info.origin.orientation.w))
        (_,_,yaw) = euler_from_quaternion(quat)

        origin = (map.info.origin.position.x,
                  map.info.origin.position.y,
                  yaw)

        # pass map info to racecar instance
        self.rcs.setMap(ros_omap, map.info.resolution, origin)

    def odomCallback(self, odom_msg):
        self.odom_state = self.rcs.getState()
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

    #Prints
    def writeTreePoints(self, file, tree_node):

        if not tree_node.hasChildren():
            file.write("0, %f, %f\n" % (tree_node.state[0],
                                  tree_node.state[1]))

            for ro in tree_node.ro:
                file.write("1, %f, %f\n" % (ro[0],
                                      ro[1]))

        else:
            for child in tree_node.children:
                self.writeTreePoints(file, child)
            file.write("0, %f, %f\n" % (tree_node.state[0],
                                  tree_node.state[1]))

            for ro in tree_node.ro:
                file.write("1, %f, %f\n" % (ro[0], ro[1]))

    def printDepth(self, node):
        if not node.hasChildren():
            return 1
        else:
            max = 0
            for n in node.children:
                t  = self.printDepth(n) + 1
                if t > max:
                    max = t
            if not node.parent:
                print(max)
            return max


    #VISUALIZATION
    def add_waypoint_visualization(self, point, frame_id, r,  g, b,
                transparency = 0.5, scale_x = 0.2,scale_y = 0.2,scale_z = 0.2):

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = frame_id
        waypoint_marker.header.stamp =  rospy.Time(0)
        waypoint_marker.ns = "pure_pursuit"
        waypoint_marker.id = self.unique_id
        waypoint_marker.type = 2
        waypoint_marker.action = 0
        waypoint_marker.pose.position.x = point[0]
        waypoint_marker.pose.position.y = point[1]
        waypoint_marker.pose.position.z = 0
        waypoint_marker.pose.orientation.x = 0.0
        waypoint_marker.pose.orientation.y = 0.0
        waypoint_marker.pose.orientation.z = 0.0
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale.x = 0.5
        waypoint_marker.scale.y = 0.5
        waypoint_marker.scale.z = 0.5
        waypoint_marker.color.a = transparency
        waypoint_marker.color.r = r
        waypoint_marker.color.g = g
        waypoint_marker.color.b = b

        self.wp_viz_pub.publish(waypoint_marker)
        rospy.sleep(0.1)
        self.unique_id = self.unique_id+ 1


    # visualize all way points in the global path
    def visualize_waypoint_data(self):
        print "Number of waypoints: ",  len(self.global_path)
        self.add_waypoint_visualization((0,0), "map", 0.0, 0.0, 1.0, 0.5)
        for  i in  range(0,len(self.global_path)):
            self.add_waypoint_visualization(self.global_path[i], "map", 0.0, 1.0, 0.0, 0.5)

        print "Published All Global WayPoints."

def run():
    """
        Main func
    """
    rospy.init_node('mcts_driver', anonymous=True)

    MCTSdriver(verbose=False, with_global=False)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()
