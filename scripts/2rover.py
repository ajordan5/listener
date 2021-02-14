import numpy as np
import rospy
import navpy

from nav_msgs.msg import Odometry

class Rover():
    def __init__(self):
        print('init rover')
        # rospy.init_node('rover', anonymous=True) #only one class needs to start a rospy node.  boat.py is doing that right now.

        self.rover_pos = [0.0,0.0,0.0]
        self.prev_rover_pos = [0.0,0.0,0.0]
        self.rover_vel = [0.0,0.0,0.0]
        self.prev_time = 0.0

        self.rover_odom = Odometry()

        self.rover_odom_pub_ = rospy.Publisher('rover_odom', Odometry, queue_size=5, latch=True)

    def update(self,x,y,z):
        self.rover_pos = np.array([y,x,z])
        # self.calc_derivatives()
        self.publish_rover_odom()

    # def calc_derivatives(self):
    #     time = rospy.get_time()
    #     dt = time-self.prev_time
    #     if dt < 0.00001:
    #         return [0.0,0.0,0.0]
    #     self.rover_vel = (np.array(self.rover_pos)-np.array(self.prev_rover_pos))/dt
    #     self.prev_time = time
    #     self.prev_rover_pos = self.rover_pos

    def publish_rover_odom(self):
        # self.rover_odom.pose.pose.position.x = self.boat_pos[0]
        # self.rover_odom.pose.pose.position.y = self.boat_pos[1]
        # self.rover_odom.pose.pose.position.z = self.boat_pos[2]
        # self.rover_pub_.publish(self.boat_pos_msg)