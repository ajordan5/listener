import rospy
from geometry_msgs.msg import PoseStamped
from vehicle import Vehicle

class Rover(Vehicle):
    def __init__(self):
        super().__init__()
        rospy.init_node('vehicles', anonymous=True) #This must be done in one and only one of Rover or Boat
        self.pose_pub_ = rospy.Publisher('rover_pose', PoseStamped, queue_size=5, latch=True)

    def publish_pose(self):
        self.pose_pub_.publish(self.poseMsg)


    