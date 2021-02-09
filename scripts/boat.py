import numpy as np
import rospy
import navpy

from geometry_msgs.msg import Point

class Boat():
    def __init__(self):
        print('init boat')
        
        self.lla = Point()
        self.ref_lla_set = False
        rospy.init_node('boat', anonymous=True)
        self.boat_pos = Point()
        self.boat_pub_ = rospy.Publisher('boat_pos', Point, queue_size=5, latch=True)
        self.ref_sub_ = rospy.Subscriber('ref', Point, self.setRefCallback, queue_size=5)


    def setRefCallback(self,msg):
        print("in set ref callback")
        self.ref_lla = [msg.x,msg.y,msg.z]
        self.ref_lla_set = True

    def update(self,x,y,z):
        if self.ref_lla_set:
            print("in update if statement")
            ned = [y,x,z]
            navpyLla = navpy.ned2lla(ned,self.ref_lla[0],self.ref_lla[1],self.ref_lla[2])
            self.lla.x = navpyLla[0]
            self.lla.y = navpyLla[1]
            self.lla.z = navpyLla[2]
            self.boat_pub_.publish(self.lla)

    