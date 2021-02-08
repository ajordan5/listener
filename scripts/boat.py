import numpy as np

class Boat():
    def __init__(self):
        print('init boat')
        self.boat_odom = np.zeros(3)

    def update(self,x,y,z):
        self.boat_odom[0] = x
        self.boat_odom[1] = y
        self.boat_odom[2] = z
        print('boat odom = ', self.boat_odom)