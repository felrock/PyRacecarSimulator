#!/usr/bin/env python
import numpy as np
import sys

#ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import followgap


#PARAMS
VELOCITY = 2.0 # meters per second

class SimpleDriver:
    """ Implement Simple driving on the car
    """

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~drive_topic')
        self.max_steer = rospy.get_param('~max_steer_ang')
        self.fov = rospy.get_param('~scan_fov')
        self.num_rays = rospy.get_param('~scan_beams')

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=20)

        # 0.004 is from somewhere idk
        self.fg = followgap.PyFollowGap(10, 15.0, self.max_steer, 0.004)


    def steer(self, angle):
        global VELOCITY

        angle = np.clip(angle, -0.4189, 0.4189)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        self.drive_pub.publish(drive_msg)


    def lidar_callback(self, data):

        lidar_np = np.array(data.ranges, dtype=np.float32)
        prediction = self.fg.eval(lidar_np, len(lidar_np))
        print(prediction)
        self.steer(prediction)

def main(args):
    rospy.init_node("Simple_driver_node", anonymous=True)
    pd = SimpleDriver()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
