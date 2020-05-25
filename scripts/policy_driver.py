#!/usr/bin/env python
import numpy as np
import policy as pl
import sys

#ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


#PARAMS
VELOCITY = 2.0 # meters per second

class PolicyDriver:
    """ Implement Policy driving on the car
    """

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

	self.pol = pl.Policy()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=20)


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

        #processesing
        prediction = self.pol.predict_action(data.ranges)
        print(prediction)
        self.steer(prediction)

def main(args):
    rospy.init_node("Policy_driver_node", anonymous=True)
    pd = PolicyDriver()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
