import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid


class RunSimulationViz:

    def __init__(self):
        pass

    def updatePoseCallback(self):
        pass

    def obsCallback(self):
        pass

    def poseCallback(self):
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

