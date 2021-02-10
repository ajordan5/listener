import numpy as np
import rospy
import navpy

from geometry_msgs.msg import Point

class Boat():
    def __init__(self):
        print('init boat')
        rospy.init_node('boat', anonymous=True)

        self.boat_pos = [0.0,0.0,0.0]
        self.boat_vel = [0.0,0.0,0.0]
        self.prev_boat_pos = [0.0,0.0,0.0]
        self.simRoverStartPosition = [0.0,0.0,0.0]
        self.commonRefOffset = [0.0,0.0,0.0]
        self.prev_time = 0.0
        self.refLlaSet = False

        self.boat_pos_msg = Point()
        self.boat_vel_msg = Point()

        self.boat_pub_ = rospy.Publisher('boat_pos', Point, queue_size=5, latch=True)
        self.boat_vel_pub_ = rospy.Publisher('boat_vel', Point, queue_size=5, latch=True)

    def update(self,x,y,z):
        self.boat_pos = np.array([y,x,z]) - np.array(self.simRoverStartPosition)
        self.calc_velocities()
        self.publish_boat_pos()
        self.publish_boat_vel()

    def set_rover_start_position(self,x,y,z):
        print('setting rover start position')
        self.simRoverStartPosition = [y,x,z]

    def calc_velocities(self):
        time = rospy.get_time()
        dt = time-self.prev_time
        if dt < 0.00001:
            return [0.0,0.0,0.0]
        self.boat_vel = (np.array(self.boat_pos)-np.array(self.prev_boat_pos))/dt
        self.prev_time = time
        self.prev_boat_pos = self.boat_pos

    def publish_boat_pos(self):
        self.boat_pos_msg.x = self.boat_pos[0]
        self.boat_pos_msg.y = self.boat_pos[1]
        self.boat_pos_msg.z = self.boat_pos[2]
        self.boat_pub_.publish(self.boat_pos_msg)

    def publish_boat_vel(self):
        self.boat_vel_msg.x = self.boat_vel[0]
        self.boat_vel_msg.y = self.boat_vel[1]
        self.boat_vel_msg.z = self.boat_vel[2]
        self.boat_vel_pub_.publish(self.boat_vel_msg)


    