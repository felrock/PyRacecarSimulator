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

class RunSimulationViz:

    def __init__(self, visualize=True, verbose=True):

        self.visualize = visualize
        self.verbose = verbose

        # parameters for simulation
        self.drive_topic = rospy.get_param("~drive_topic")
        self.map_topic = rospy.get_param("~map_topic")
        self.scan_topic = rospy.get_param("~scan_topic")
        self.pose_topic = rospy.get_param("~pose_topic")
        self.odom_topic = rospy.get_param("~odom_topic")
        self.pose_rviz_topic = rospy.get_param("~pose_rviz_topic")
        self.imu_topic = rospy.get_param("~imu_topic")
        self.gt_pose_topic = rospy.get_param("~ground_truth_pose_topic")

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
            "mass": rospy.get_param("~mass"),
            "batch_size": rospy.get_param("~batch_size")
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

        # other params
        self.speed_clip_dif = rospy.get_param("~speed_clip_diff")
        self.broadcast_transform = rospy.get_param("~broadcast_transform")
        self.pub_gt_pose = rospy.get_param("~publish_ground_truth_pose")

        # transform broadcaster
        self.br = TransformBroadcaster()

        # publishers
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1)
        self.pose_pub = rospy.Publisher(self.gt_pose_topic, PoseStamped, queue_size=1)

        # subscribers
        self.drive_sub = rospy.Subscriber(self.drive_topic, AckermannDriveStamped, self.driveCallback, queue_size=1)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.poseCallback)
        self.pose_rviz_sub = rospy.Subscriber(self.pose_rviz_topic, PoseWithCovarianceStamped,  self.poseRvizCallback)

        if self.verbose:
            print "Driver constructed"

        self.update_simulation = rospy.Timer(rospy.Duration(self.update_pose_rate), self.updateSimulationCallback)
        self.t_start = 0
        self.t_started = False

    def updateSimulationCallback(self, event):
        """
            Updates simulatio one step
        """

        # create timestamp
        timestamp = rospy.get_rostime()

        # update simulation
        self.rcs.updatePose()

        if self.visualize:

            # pub pose as transform
            self.poseTransformPub(timestamp)

            # publish steering ang
            self.steerAngTransformPub(timestamp)

            # publish odom
            self.odomPub(timestamp)

        # sim lidar
        self.rcs.runScan()

        if self.rcs.checkCollision() >= 0:

            print "crash %f" % (time.time() - self.t_start)
            # do other things here too
            self.rcs.stop()
            self.t_started = False

        if self.visualize:
            # publish lidar
            self.lidarPub(timestamp)

            # publish the transform
            self.laserLinkTransformPub(timestamp)

    def driveCallback(self, msg):
        """
            Pass actions for driving
        """

        if not self.t_started:
            self.t_started = True
            self.t_start = time.time()

        self.rcs.drive(msg.drive.speed, msg.drive.steering_angle)


    def poseCallback(self, msg):
        """
            Update current pose on the map

            Only for Rviz changed pose
        """

        state = self.rcs.getState()
        state[0] = msg.pose.position.x
        state[1] = msg.pose.position.y
        quat = np.array((msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w))
        # yaw
        (_,_,yaw) = euler_from_quaternion(quat)
        state[2] = yaw

        self.rcs.setState(state)
        self.rcs.runScan()


    def poseRvizCallback(self, msg):
        """
            same as above
        """
        ps_msg = PoseWithCovarianceStamped()
        ps_msg.header = msg.header
        ps_msg.pose = msg.pose.pose
        self.poseCallback(ps_msg)


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
        ts.child_frame_id =  self.base_frame
        ts.transform = pt_msg

        if self.broadcast_transform:
            self.br.sendTransform(ts)

        if self.pub_gt_pose:
            pass
            #self.pose_pub.publish(ps)


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

