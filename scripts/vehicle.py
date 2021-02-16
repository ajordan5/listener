import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

class Vehicle():
    def __init__(self):
        self.poseMsg = PoseStamped()

    def update(self,x,y,z,qx,qy,qz,qw):
        self.poseMsg.header.stamp = rospy.Time.now()
        self.poseMsg.pose.position.x = y #convert from ENU fram to NED
        self.poseMsg.pose.position.y = x
        self.poseMsg.pose.position.z = -z
        self.poseMsg.pose.orientation.x = qx
        self.poseMsg.pose.orientation.y = qy
        self.poseMsg.pose.orientation.z = qz
        self.poseMsg.pose.orientation.w = qw
        self.publish_pose()

    def publish_pose(self):
        print('Nothing to publish in parent class')


    