#!/usr/bin/env python
import rospy
import numpy as np # For random numbers
import time
 
from std_msgs.msg import String
from gazebo_msgs import msg 
 
class position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 0.0

#Create callback. This is what happens when a new message is received
def sub_cal(msg):
    n = len(msg.name)-2
    global objects
    objects = [position() for i in range(n)] 
    for i in range(n-1,-1,-1):
        objects[n-1-i].x = msg.pose[i+2].position.x
        objects[n-1-i].y = msg.pose[i+2].position.y
        objects[n-1-i].z = msg.pose[i+2].position.z
        objects[n-1-i].ox = msg.pose[i+2].orientation.x
        objects[n-1-i].oy = msg.pose[i+2].orientation.y
        objects[n-1-i].oz = msg.pose[i+2].orientation.z
        objects[n-1-i].ow = msg.pose[i+2].orientation.w


#Initialize publisher
def initSubscriber():
    rospy.Subscriber('gazebo/model_states', msg.ModelStates, sub_cal, queue_size=1000)

def get_positions():
    print("get object positions")
    n = 1
    global objects
    objects = 0
    while(n == 1):
        rospy.sleep(0.1)
        n = len(objects)
        print('Number of objects: {}'.format(n))
        if(n > 0):
            print('bucket\nx: {}\ny: {}\nz: {}'.format(objects[0].x, objects[0].y, objects[0].z))
            for i in range(1,n):
                    print('cube{}\nx: {}\ny: {}\nz: {}'.format(n-1-i, objects[i].x, objects[i].y, objects[i].z))
    return objects
