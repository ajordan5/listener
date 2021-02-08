#!/usr/bin/env python3

#This example shows how to use the follow me plugin

import asyncio
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)

# import rospy

# from geometry_msgs.msg import Point


class FollowMe:
    async def initialize(self):
        # print('before subscriber')
        # self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
        # print('after subscriber')
        self.default_height = 8.0 #in Meters
        self.follow_distance = 2.0 #in Meters, this is the distance that the drone will remain away from Target while following it 
        #Direction relative to the Target 
        #Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
        self.direction = Config.FollowDirection.BEHIND
        self.responsiveness =  0.02

        #This list contains fake location coordinates (These coordinates are obtained from mission.py example)
        self.fake_location = [[47.398039859999997,8.5455725400000002],[47.398036222362471,8.5450146439425509],[47.397825620791885,8.5450092830163271]]

    # async def boatCallback(self,msg):
    #     print('in subscriber')
    #     self.x = msg.x
    #     self.y = msg.y
    #     self.z = msg.z
    #     print('self.x = ', self.x)

    async def fly_drone(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        #This waits till a mavlink based drone is connected
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone with UUID: {state.uuid}")
                break

        await self.initialize()

        #Checking if Global Position Estimate is ok
        async for global_lock in drone.telemetry.health():
            if global_lock.is_global_position_ok:
                print("-- Global position state is good enough for flying.")
                break
        #Arming the drone
        print ("-- Arming")
        await drone.action.arm()
        
        #Follow me Mode requires some configuration to be done before starting the mode
        conf = Config(self.default_height, self.follow_distance, self.direction, self.responsiveness)
        await drone.follow_me.set_config(conf)
        
        print ("-- Taking Off")
        await drone.action.takeoff()
        await asyncio.sleep(8)
        print ("-- Starting Follow Me Mode")
        await drone.follow_me.start()
        await asyncio.sleep(8)

        #This for loop provides fake coordinates from the fake_location list for the follow me mode to work
        #In a simulator it won't make much sense though
        for latitude,longitude in self.fake_location:
            target = TargetLocation(latitude, longitude, 0, 0, 0, 0)
            print ("-- Following Target")
            await drone.follow_me.set_target_location(target)
            await asyncio.sleep(2)
        
        #Stopping the follow me mode
        print ("-- Stopping Follow Me Mode")
        await drone.follow_me.stop()
        await asyncio.sleep(5)
        
        print ("-- Landing")
        await drone.action.land()

if __name__ == "__main__":
    followMe = FollowMe()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(followMe.fly_drone())
