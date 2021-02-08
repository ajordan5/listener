import numpy as np
import rospy

from geometry_msgs.msg import Point

class Boat():
    def __init__(self):
        print('init boat')
        
        rospy.init_node('boat', anonymous=True)
        self.boat_pos = Point()
        self.boat_pub_ = rospy.Publisher('boat_pos', Point, queue_size=5, latch=True)

    def update(self,x,y,z):
        print('x = ', x)
        self.boat_pos.x = x
        self.boat_pos.y = y
        self.boat_pos.z = z

        self.boat_pub_.publish(self.boat_pos)