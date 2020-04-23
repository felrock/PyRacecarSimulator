import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GetMap
from std_msgs.msg import String, Header, Float32MultiArray

from car_config import CarParams

class RunSimulationViz:

    def __init__(self):

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
        self.drive_sub = rospy.Subscriber(self.drive_topic, self.driveCallback)
        self.map_sub = rospy.Subscriber(self.map_topic, self.mapCallback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, self.poseCallback)
        self.pose_rviz_sub = rospy.Subscriber(self.pose_rviz_topic, self.poseRvizCallback)
        #self.obs_sub = rospy.Subscriber("/clicked_point", self.obsCallback)

        # read map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        ros_map = rospy.ServiceProxy(map_service_name, GetMap)().map.data
        self.car_config["resoltion"] = map_msg.info.resolution

        # racecar object
        self.rcs = RacecarSimulator(config, params, ros_map)

        rospy.INFO("Driver constructed.")

    def updatePoseCallback(self):

        # update simulation
        self.rcs.update()

        # publish lidar

    def obsCallback(self):
        pass

    def poseCallback(self):
        pass

    def poseRvizCallback(self):
        pass

    def mapCallback(self):
        pass

    def poseTransformPub(self):
        pass

    def steerAngTransformPub(self);
        pass

    def laserLinkTransformPub(self):
        pass

    def odomPub(self):
        pass

    def pubOdom(self):
        pass

def run():
    rospy.init_node('RunSimulationViz', anonymous=True)
    RunSimulationViz()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    run()

