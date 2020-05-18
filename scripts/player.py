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

class PlayerROSInterface:

    def __init__(self, config, visualize=True, verbose=True, name="one"):

        self.visualize = visualize
        self.verbose = verbose

        # parameters for simulation
        self.drive_topic = rospy.get_param("~drive_topic_%s" % name)
        self.scan_topic = rospy.get_param("~scan_topic_%s" % name)
        self.pose_topic = rospy.get_param("~pose_topic_%s" % name)
        self.odom_topic = rospy.get_param("~odom_topic_%s" % name)

        self.map_frame = rospy.get_param("~map_frame")
        self.base_frame = rospy.get_param("~base_frame")
        self.scan_frame = rospy.get_param("~scan_frame")
        self.update_pose_rate = rospy.get_param("~update_pose_rate")

        self.config = config

        # racecar object
        self.rcs = RacecarSimulator(config)

        # transform broadcaster
        self.br = TransformBroadcaster()

        # publishers
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry)

        # subscribers
        self.drive_sub = rospy.Subscriber(self.drive_topic,
                                            AckermannDriveStamped,
                                            self.driveCallback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic,
                                         PoseStamped,
                                         self.poseCallback)

        if self.verbose:
            print "Player %s construted!" % name

    def driveCallback(self, msg):
        """
            Pass actions for driving
        """

        self.rcs.drive(msg.drive.speed, msg.drive.steering_angle)


    def poseTransformPub(self, timestamp):
        """
            Publish the transform for pose
        """

        # get state information
        state = self.rcs.getState()

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
        ts.child_frame_id = self.base_frame
        ts.transform = pt_msg

        if self.broadcast_transform:
            self.br.sendTransform(ts)

        if self.pub_gt_pose:
            self.pose_pub.publish(ps)


    def steerAngTransformPub(self, timestamp):
        """
            Publish steering transforms, left and right steer the same
        """

        state = self.rcs.getState()

        ts_msg = TransformStamped()
        ts_msg.header.stamp = timestamp

        quat = quaternion_from_euler(0.0, 0.0, state[2])
        ts_msg.transform.rotation.x = quat[0]
        ts_msg.transform.rotation.y = quat[1]
        ts_msg.transform.rotation.z = quat[2]
        ts_msg.transform.rotation.w = quat[3]

        # publish for right and left steering
        ts_msg.header.frame_id = "front_left_hinge"
        ts_msg.child_frame_id = "front_left_wheel"
        self.br.sendTransform(ts_msg)

        ts_msg.header.frame_id = "front_right_hinge"
        ts_msg.child_frame_id = "front_right_wheel"
        self.br.sendTransform(ts_msg)


    def laserLinkTransformPub(self, timestamp):
        """
            Publish the lidar transform, from base
        """

        ts_msg = TransformStamped()
        ts_msg.header.stamp = timestamp
        ts_msg.header.frame_id = self.base_frame
        ts_msg.child_frame_id = self.scan_frame
        ts_msg.transform.translation.x = self.car_config["scan_dist_to_base"]
        ts_msg.transform.rotation.w = 1
        self.br.sendTransform(ts_msg)

    def odomPub(self, timestamp):
        """
            Publish simulation odometry
        """

        state = self.rcs.getState()

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
        self.odom_pub.publish(od_msg)


    def lidarPub(self, timestamp):
        """
            Publish lidar
        """

        scan = self.rcs.getScan()

        scan_msg = LaserScan()
        scan_msg.header.stamp = timestamp
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = -self.car_config["scan_fov"]/2.0
        scan_msg.angle_max = self.car_config["scan_fov"]/2.0
        scan_msg.angle_increment = self.car_config["scan_fov"]/self.car_config["scan_beams"]
        scan_msg.range_max = self.car_config["scan_max_range"]
        scan_msg.ranges = scan
        scan_msg.intensities = scan
        self.scan_pub.publish(scan_msg)

if __name__ == '__main__':
    run()

