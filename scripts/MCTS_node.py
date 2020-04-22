from racecar_simulator import RacecarSimulator
from car_config import CarParam

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import tensorflow as tf
import numpy as np

"""
    MCTS running simulatios with a NN policy
"""

class MCTSNode:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self):

        # read params
        # create CarParams instance
        # create subscriber/publishers
        # load map
        # create RacecarSim instance
        pass



def main():
    rospy.init_node("MCTS_node", anonymous=True)
    MCTSNode()
    rospy.sleep()
    rospy.spin()

if __name__ == '__name__':
    pass

