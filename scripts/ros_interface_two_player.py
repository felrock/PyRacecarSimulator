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
from rcs_two_player import RacecarSimulatorTwoPlayer

class RunSimulationViz:

    def __init__(self, visualize=True, verbose=True):

        self.visualize = visualize
        self.verbose = verbose

        self.update_pose_rate = rospy.get_param("~update_pose_rate")

        # read parameters
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
            "update_pose_rate": rospy.get_param("~update_pose_rate"),
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

        self.cars = ["one", "two"]
        # create simulation with two players
        self.rcs = RacecarSimulatorTwoPlayer(self.car_config,
                                            [('one', (0,0.5)),
                                             ('two', (0,-0.5))])
        # set Maps
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
        self.rcs.setMap(self.map_msg)

        # driving actios for player one and two
        self.drive_topic_one = rospy.get_param("~drive_topic") + "one"
        self.drive_topic_two = rospy.get_param("~drive_topic") + "two"

        # simulation doesnt need this?
        self.map_topic = rospy.get_param("~map_topic")

        # we dont need to display scan in rviz, but agents need it
        # to make actions
        self.scan_topic_one = rospy.get_param("~scan_topic") + "one"
        self.scan_topic_two = rospy.get_param("~scan_topic") + "two"

        # we dont need pose updates
        self.pose_topic_one = rospy.get_param("~pose_topic") + "one"
        self.pose_topic_two = rospy.get_param("~pose_topic") + "two"

        # this should publish "gt" for both p1 and p2
        self.odom_topic_one = rospy.get_param("~odom_topic") + "one"
        self.odom_topic_two = rospy.get_param("~odom_topic") + "two"

        # since we are not displaying any lidar, do we need to
        # do tfs?
        self.map_frame = rospy.get_param("~map_frame")
        self.base_frame = rospy.get_param("~base_frame")
        self.scan_frame = rospy.get_param("~scan_frame")

        # transform broadcaster, for gt_poses only
        self.br = TransformBroadcaster()

        ## publishers
        # publishers for scans, no tf used
        self.scan_pub_one = rospy.Publisher(self.scan_topic_one,
                                            LaserScan, queue_size=1)
        self.scan_pub_two = rospy.Publisher(self.scan_topic_two,
                                            LaserScan, queue_size=1)

        # odom for rviz, rviz takes poses
        self.pose_pub_one = rospy.Publisher(self.pose_topic_one,
                                        PoseStamped, queue_size=1)
        self.pose_pub_two = rospy.Publisher(self.pose_topic_two,
                                        PoseStamped, queue_size=1)

        self.odom_pub_one = rospy.Publisher(self.odom_topic_one,
                                            Odometry, queue_size=1)
        self.odom_pub_two = rospy.Publisher(self.odom_topic_two,
                                            Odometry, queue_size=1)

        """
        # dont need this prob
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1)
        self.pose_pub = rospy.Publisher(self.gt_pose_topic, PoseStamped, queue_size=1)
        """

        # subscribers
        self.timer = rospy.Timer(rospy.Duration(self.update_pose_rate),
                                            self.updateSimulationCallback)

        self.drive_sub_one = rospy.Subscriber(self.drive_topic_one,
                                            AckermannDriveStamped,
                                            self.driveCallbackOne, queue_size=1)
        self.drive_sub_two = rospy.Subscriber(self.drive_topic_two,
                                            AckermannDriveStamped,
                                            self.driveCallbackTwo, queue_size=1)

        # dont need this prob
        """
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.poseCallback)
        self.pose_rviz_sub = rospy.Subscriber(self.pose_rviz_topic,
                PoseWithCovarianceStamped,  self.poseRvizCallback)
        """

        if self.verbose:
            print "Simulation constructed"


    def updateSimulationCallback(self, event):
        """
            Updates simulatio one step
        """
        # create timestamp
        timestamp = rospy.get_rostime()

        """ update player one """
        # update simulation
        for car in self.cars:
            self.rcs.updatePose(car)
        # write to rviz
        for car in self.cars:
            # pub pose as transform
            self.poseTransformPub(timestamp, car)

            # publish steering ang
            self.steerAngTransformPub(timestamp, car)

            # publish odom
            self.odomPub(timestamp, car)

        # write to agents
        # sim lidar
        t1 = time.time()
        self.rcs.runScan(self.cars[1])
        self.lidarPub(timestamp, self.cars[0])

        self.rcs.runScan(self.cars[0])
        self.lidarPub(timestamp, self.cars[1])

        print "wtf is this time, %f" % (time.time()-t1)


        #if self.rcs.checkCollision() > 0:
            # do other things here too
            #self.rcs.stop()



    def steerAngTransformPub(self, timestamp, player):
        """
            Publish steering transforms, left and right steer the same
        """

        if player == "one":
            prefix = "r1/"
        else:
            prefix = "r2/"

        state = self.rcs.getState(player)

        ts_msg = TransformStamped()
        ts_msg.header.stamp = timestamp

        quat = quaternion_from_euler(0.0, 0.0, state[2])
        ts_msg.transform.rotation.x = quat[0]
        ts_msg.transform.rotation.y = quat[1]
        ts_msg.transform.rotation.z = quat[2]
        ts_msg.transform.rotation.w = quat[3]

        # publish for right and left steering
        ts_msg.header.frame_id = prefix + "front_left_hinge"
        ts_msg.child_frame_id = prefix + "front_left_wheel"

        self.br.sendTransform(ts_msg)

        ts_msg.header.frame_id = prefix + "front_right_hinge"
        ts_msg.child_frame_id = prefix + "front_right_wheel"
        self.br.sendTransform(ts_msg)


    def odomPub(self, timestamp, player):
        """
            Publish simulation odometry
        """

        state = self.rcs.getState(player)

        od_msg = Odometry()
        od_msg.header.stamp = timestamp
        od_msg.header.frame_id = self.map_frame
        od_msg.child_frame_id = self.base_frame
        quat = quaternion_from_euler(0.0, 0.0, state[2])
        od_msg.pose.pose.orientation.x = quat[0]
        od_msg.pose.pose.orientation.y = quat[1]
        od_msg.pose.pose.orientation.z = quat[2]
        od_msg.pose.pose.orientation.w = quat[3]

        od_msg.pose.pose.position.x = state[0]
        od_msg.pose.pose.position.y = state[1]
        od_msg.twist.twist.linear.x = state[3]
        od_msg.twist.twist.linear.z = state[4]

        if player == "one":
            self.odom_pub_one.publish(od_msg)
        else:
            self.odom_pub_two.publish(od_msg)


    def lidarPub(self, timestamp, player):
        """
            Publish lidar
        """

        scan = self.rcs.runScan(player)

        scan_msg = LaserScan()
        scan_msg.header.stamp = timestamp
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = -self.car_config["scan_fov"]/2.0
        scan_msg.angle_max = self.car_config["scan_fov"]/2.0
        scan_msg.angle_increment = self.car_config["scan_fov"]/self.car_config["scan_beams"]
        scan_msg.range_max = self.car_config["scan_max_range"]
        scan_msg.ranges = scan
        scan_msg.intensities = scan

        if player == "one":
            self.scan_pub_one.publish(scan_msg)

        else:
            self.scan_pub_two.publish(scan_msg)

    def driveCallbackOne(self, msg):
        """
            Pass actions for driving
        """

        self.rcs.drive("one", msg.drive.speed, msg.drive.steering_angle)

    def driveCallbackTwo(self, msg):
        """
            Pass actions for driving
        """

        self.rcs.drive("two", msg.drive.speed, msg.drive.steering_angle)


    def poseTransformPub(self, timestamp, player):
        """
            Publish the transform for pose
        """

        if player == "one":
            prefix = "r1/"
        else:
            prefix = "r2/"

        # get state information
        state = self.rcs.getState(player)

        # create the message
        pt_msg = Transform()
        pt_msg.translation.x = state[0]
        pt_msg.translation.y = state[1]
        quat = quaternion_from_euler(0.0, 0.0, state[2])
        pt_msg.rotation.x = quat[0]
        pt_msg.rotation.y = quat[1]
        pt_msg.rotation.z = quat[2]
        pt_msg.rotation.w = quat[3]

        # ground truth
        ps = PoseStamped()
        ps.header.frame_id = self.map_topic
        ps.pose.position.x = state[0]
        ps.pose.position.y = state[1]
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]

        # add a header
        ts = TransformStamped()
        ts.header.stamp = timestamp
        ts.header.frame_id = self.map_frame
        ts.child_frame_id = prefix + self.base_frame
        ts.transform = pt_msg

        self.br.sendTransform(ts)

        if player == "one":
            self.pose_pub_one.publish(ps)
        else:
            self.pose_pub_two.publish(ps)


def run():
    """
        Main func
    """

    rospy.init_node('RunSimulationViz', anonymous=True)
    RunSimulationViz(True, False)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()

