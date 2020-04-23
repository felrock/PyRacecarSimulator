import rospy
import tf2_ros
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Quarternion, Transform, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GetMap
from std_msgs.msg import String, Header, Float32MultiArray

from car_config import CarParams
from racecar_simulator import RacecarSimulator

class RunSimulationViz:

    def __init__(self, vizualize=False):

        self.vizualize = vizualize

        # topics
        self.drive_topic = rospy.get_param("drive_topic")
        self.map_topic = rospy.get_param("map_topic")
        self.scan_topic = rospy.get_param("scan_topic")
        self.pose_topic = rospy.get_param("pose_topic")
        self.odom_topic = rospy.get_param("odom_topic")
        self.pose_rviz_topic = rospy.get_param("pose_rviz_topic")
        self.imu_topic = rospy.get_param("imu_topic")
        self.gt_pose_topic = rospy.get_param("ground_truth_pose_topic")

        self.buffer_length = rospy.get_param("buffer_length")
        self.map_frame = rospy.get_param("map_frame")
        self.base_frame = rospy.get_param("base_frame")
        self.scan_frame = rospy.get_param("scan_frame")
        self.update_pose_rate = rospy.get_param("update_pose_rate")

        # config
        self.car_config = {
            "scan_beams": rospy.get_param("scan_beams"),
            "scan_fov": rospy.get_param("scan_fov"),
            "scan_std": rospy.get_param("scan_std"),
            "free_thresh": rospy.get_param("free_thresh"),
            "scan_dist_to_base": rospy.get_param("scan_dist_to_base"),
            "max_speed": rospy.get_param("max_speed"),
            "max_accel": rospy.get_param("max_accel"),
            "max_steer_ang": rospy.get_param("max_steer_ang"),
            "steer_ang_vel": rospy.get_param("steer_ang_vel"),
            "col_thresh": rospy.get_param("coll_threshold"),
            "ttc_threshold": rospy.get_param("ttc_threshold"),
            "width": rospy.get_param("width"),
            "scan_max_range": rospy.get_param("scan_max_range")
        }

        self.car_param = CarParams({
            "wheelbase": rospy.get_param("wheelbase"),
            "friction_coeff": rospy.get_param("friction_coeff"),
            "height_cg": rospy.get_param("height_cg"),
            "l_cg2rear": rospy.get_param("l_cg2rear"),
            "l_cg2front": rospy.get_param("l_cg2front"),
            "C_S_front": rospy.get_param("C_S_front"),
            "C_S_rear": rospy.get_param("C_S_rear"),
            "moment_inertia": rospy.get_param("moment_inertia"),
            "mass": rospy.get_param("mass")
        })

        self.speed_clip_dif = rospy.get_param("speed_clip_diff")
        self.broadcast_transform = rospy.get_param("broadcast_transform")
        self.pub_gt_pose = rospy.get_param("publish_ground_truth_pose")

        # publishers
        self.scan_pub = rospy.Publisher(self.scan_topic, queue_size=1)
        self.odom_pub = rospy.Publisher(self.odom_topic, queue_size=1)
        self.map_pub = rospy.Publisher(self.map_topic, queue_size=1)
        self.pose_pub = rospy.Publisher(self.gt_pose_topic, queue_size=1)

        # subscribers
        self.update_pose_timer = rospy.Timer(rospy.Duration(self.update_pose_rate), self.updatePoseCallback)
        ##self.drive_sub = rospy.Subscriber(self.drive_topic, self.driveCallback)
        self.map_sub = rospy.Subscriber(self.map_topic, self.mapCallback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, self.poseCallback)
        self.pose_rviz_sub = rospy.Subscriber(self.pose_rviz_topic, self.poseRvizCallback)
        #self.obs_sub = rospy.Subscriber("/clicked_point", self.obsCallback)

        # read map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        self.car_config["resoltion"] = map_msg.info.resolution

        # racecar object
        self.rcs = RacecarSimulator(self.car_config, self.car_param, map_msg.data)

        # transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        rospy.INFO("Driver constructed.")

    def updatePoseCallback(self):
        """
            Updates simulatio one step
        """

        # update simulation
        self.rcs.update()

        if self.vizualize:

            # publish things for rviz
            timestamp = rospy.Time.now()

            # pub pose as transform
            self.poseTransformPub(timestamp)

            # publish steering ang
            self.steerAngTransformPub(timestamp)

            # publish odom
            self.pubOdom(timestamp)

            # publish imu
            # todo

            # publish lidar
            self.lidarPub(timestamp)

            # publish the transform
            self.laserLinkTransformPub(timestamp)


    def poseCallback(self, msg):
        """
            Update current pose on the map

            Only for Rviz changed pose
        """

        state = self.rcs.getState()
        state.x = msg.pose.position.x
        state.y = msg.pose.position.y
        quat = Quarternion(msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)
        state.theta = tf2_ros.impl.getYaw(quat)

        self.rcs.setState(state)

    def poseRvizCallback(self, msg):
        """
            same as above
        """
        ps_msg = PoseStamped()
        ps_msg.header = msg.header
        ps_msg.pose = msg.pose.pose
        poseCallback(ps_msg)

    def mapCallback(self, msg):
        """
            Map is changed, pass new map 2d-scanner
        """

        height = msg.info.height
        width = msg.info.width
        resolution = msg.info.resolution

        quat = Quarternion(msg.info.origin.orientation.x,
                           msg.info.origin.orientation.y,
                           msg.info.origin.orientation.z,
                           msg.info.origin.orientation.w)

        origin = (msg.info.origin.position.x,
                  msg.info.origin.position.y,
                  tf2_ros.impl.getYaw(quat))
        ros_map = np.array(msg.data)

        # prob done a faster way
        ros_map = ros_map.map(lambda x: 0.5 if x > 100 or x < 0 else x/100.0)

        # create this funciton in scan_sim
        self.rcs.scan_simulator.setMap(
                    ros_map,
                    resolution,
                    origin)


    def poseTransformPub(self, timestamp):
        """
            Publish the transform for pose
        """
        # get state information
        state = self.rcs.getState()

        # create the message
        pt_msg = Transform()
        pt_msg.translation.x = state.x
        pt_msg.translation.y = state.y
        quat = Quarternion()
        quat.setEuler(0.0, 0.0, state.theta)
        pt_msg.rotation.x = quat.x()
        pt_msg.rotation.y = quat.y()
        pt_msg.rotation.z = quat.z()
        pt_msg.rotation.w = quat.w()

        # ground truth
        ps = PoseStamped()
        ps.header.frame_id = self.map_topic
        ps.pose.position.x = state.x
        ps.pose.position.y = state.y
        ps.pose.orientation.x = quat.x()
        ps.pose.orientation.y = quat.y()
        ps.pose.orientation.z = quat.z()
        ps.pose.orientation.w = quat.w()

        # add a header
        ts = TranformedStamped()
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

        quat = Quarternion()
        quat.setEuler(0.0, 0.0, state.theta)
        ts_msg.rotation.x = quat.x()
        ts_msg.rotation.y = quat.y()
        ts_msg.rotation.z = quat.z()
        ts_msg.rotation.w = quat.w()

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
        ts_msg.transform.x = self.car_config["scan_dist_to_base"]
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
        quat = Quarternion()
        quat.setEuler(0.0, 0.0, state.theta)
        od_msg.pose.pose.orientation.x = quat.x()
        od_msg.pose.pose.orientation.y = quat.y()
        od_msg.pose.pose.orientation.z = quat.z()
        od_msg.pose.pose.orientation.w = quat.w()
        od_msg.pose.pose.position.x = state.x
        od_msg.pose.pose.position.y = state.y
        od_msg.twist.twist.linear.x = state.velocity
        od_msg.twist.twist.linear.z = state.angular_velocity
        self.odom_pub.Publish(od_msg)


    def lidarPub(self, timestamp):
        """
            Publish lidar
        """

        scan_msg = LaserScan()
        scan_msg.header.stamp = timestamp
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = -self.car_config["scan_fov"]/2.0
        scan_msg.angle_max = self.car_config["scan_fov"]/2.0
        scan_msg.angle_increment =0
        scan_msg.range_max = self.car_config["scan_range_max"]
        scan_msg.ranges = self.rcs.getScan()
        scan_msg.intensities = self.rcs.getScan()

        self.scan_pub.publish(scan_msg)

def run():
    rospy.init_node('RunSimulationViz', anonymous=True)
    RunSimulationViz()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()

