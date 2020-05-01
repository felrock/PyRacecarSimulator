#!/usr/bin/env python
from __future__ import print_function

import math
import sys

import numpy as np

import policy as pl

#ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

#PID CONTROL PARAMS
kp = -0.4#TODO
kd = 0.0#TODO
ki = 0.0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#PARAMS
VELOCITY = 2.50 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

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


    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global VELOCITY
        angle = kp*error + ki*error + kd*error

        #MESSAGE
        angle = np.clip(angle, -0.4189, 0.4189)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        #angle = abs(math.degrees(angle))
        """if angle >= 0 and angle < 10:
            if VELOCITY < 4.0:
                VELOCITY = VELOCITY + 0.02
        elif angle >= 10 and angle < 20:
            if VELOCITY < 4.0 and VELOCITY > 3.5:
                VELOCITY = VELOCITY - 0.1
        else: 
            if VELOCITY < 3.5 and VELOCITY > 3.0:
                VELOCITY = VELOCITY - 0.1
            else:
                VELOCITY = 3.0"""
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


    def lidar_callback(self, data):
	proc_lidar = []

	for i in range(180,900):
		if i % 2 == 0:
			proc_lidar.append(np.clip(data.ranges[i], 0, 15.0))
	prediction = self.pol.predict_action(proc_lidar)
        print(prediction)
	self.pid_control(prediction, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    pd = PolicyDriver()
   
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
